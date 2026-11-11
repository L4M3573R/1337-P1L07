"""Per-feature decision logs for remote route review.

Each StarPilot feature logs what it decided and why, once per change, as a "starpilot_feature" event in the app log
(logMessage, present in rlogs). Decisions that change at most a few times a second stay cheap and readable; the
planner's features (Pulse and Glide, CEM stop detection) are logged every plan in starpilotLongitudinalPlan instead.

The schema policy keeps new Event slots out of cereal, so these use the log rather than new messages. Logging never
raises into the caller: a failure only drops that decision record.
"""
from __future__ import annotations

import math

EVENT = 'starpilot_feature'


def _finite(value, default: float = 0.0) -> float:
  try:
    number = float(value)
  except (TypeError, ValueError):
    return default
  return round(number, 3) if math.isfinite(number) else default


class ChangeLog:
  """Logs one event whenever a feature's decision (the key) changes; context fields ride along."""

  def __init__(self, feature: str, log=None):
    self.feature = feature
    self.last_key = None
    self.log = log

  def observe(self, key: dict, context: dict | None = None) -> bool:
    if key == self.last_key:
      return False
    self.last_key = key
    try:
      log = self.log
      if log is None:
        from openpilot.common.swaglog import cloudlog
        log = cloudlog.event
      log(EVENT, feature=self.feature, **key, **(context or {}))
    except Exception:
      return False
    return True


def turn_decision(turn_intent, desire) -> dict:
  """modeld: the turn desire sent to the model and why (or the gate that held it back)."""
  try:
    desire_value = int(desire)
  except (TypeError, ValueError):
    desire_value = -1
  return {'desire': desire_value, 'source': str(getattr(turn_intent, 'source', '')),
          'blocked_by': str(getattr(turn_intent, 'blocked_by', '')),
          'signal_stop_hold': bool(getattr(turn_intent, 'signal_stop_hold', False)),
          'nav_stop_hold': bool(getattr(turn_intent, 'stop_hold', False))}


def radar_decision(tandem) -> dict:
  """radard: what the camera+radar tandem policy did to leadOne ("camera", "hold", "pickup", "none")."""
  held = getattr(tandem, 'held', None)
  return {'action': str(getattr(tandem, 'last_action', 'none')), 'held_track': int(getattr(held, 'track_id', -1)),
          'pickup_track': int(getattr(tandem, 'last_pickup_id', -1))}


def radar_context(tandem, lead_one) -> dict:
  context = {'lead_prob': _finite(getattr(tandem, 'last_lead_prob', 0.0)),
             'frames_without_camera': int(getattr(getattr(tandem, 'held', None), 'frames_without_camera', 0))}
  present = lead_one.get('present', False) if isinstance(lead_one, dict) else getattr(lead_one, 'present', False)
  if present:
    context['lead_drel'] = _finite(lead_one['dRel'] if isinstance(lead_one, dict) else lead_one.dRel)
  return context


def aol_decision(intent) -> dict:
  """card: the AOL latch, the arm kept for the Panda, why the latch last dropped, and a pending gear re-arm."""
  retained = getattr(intent, 'lateral_arm_retained', None)
  settings = getattr(intent, 'settings', None)
  return {'allowed_latch': bool(getattr(intent, 'allowed_latch', False)),
          'lateral_armed': bool(retained() if retained is not None else getattr(intent, 'allowed_latch', False)),
          'latch_clear_reason': str(getattr(intent, 'latch_clear_reason', '')),
          'latch_clear_count': int(getattr(intent, 'latch_clear_count', 0)),
          'gear_rearm_pending': bool(getattr(intent, 'gear_rearm_pending', False)),
          'fault_inhibit': bool(getattr(intent, '_fault_inhibit', False)),
          'settings_enabled': bool(getattr(settings, 'enabled', False))}
