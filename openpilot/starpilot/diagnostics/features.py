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
          'nav_stop_hold': bool(getattr(turn_intent, 'stop_hold', False)),
          # blinker turns timed to the road: what that logic decided (in shadow mode it only logs)
          'timed': str(getattr(turn_intent, 'shadow_reason', '')),
          'timed_turning': bool(getattr(turn_intent, 'shadow_turning', False)),
          'timed_start_m': getattr(turn_intent, 'shadow_start_m', None)}


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


SLOW_LOG_MIN_GAP_S = 10.0
_slow_last: dict[str, float] = {}


def log_if_slow(feature: str, seconds: float, threshold_s: float, *, log=None, now: float | None = None, **fields) -> bool:
  """Decision log for background work that took longer than it should (at most one event per feature per 10 s)."""
  try:
    if not seconds > threshold_s:
      return False
    import time
    now = time.monotonic() if now is None else now
    if now - _slow_last.get(feature, -1e9) < SLOW_LOG_MIN_GAP_S:
      return False
    _slow_last[feature] = now
    if log is None:
      from openpilot.common.swaglog import cloudlog
      log = cloudlog.event
    log(EVENT, feature=feature, slow_s=round(seconds, 3), threshold_s=threshold_s, **fields)
    return True
  except Exception:
    return False


LEAD_LOSS_NEAR_M = 30.0        # a lead this close that disappears is always logged
LEAD_LOSS_MAX_M = 80.0         # ...and one up to this far if the car was closing in on it
LEAD_LOSS_CLOSING_MPS = -1.0
LEAD_REGAIN_WINDOW_S = 20.0    # a lead appearing within this long after a loss is logged as its return
LEAD_SAMPLE_MAX_GAP_S = 0.5


def _lead_fields(lead_one):
  get = (lambda k, d=None: lead_one.get(k, d)) if isinstance(lead_one, dict) else (lambda k, d=None: getattr(lead_one, k, d))
  present = bool(get('present', False)) if lead_one is not None else False
  if not present:
    return False, None, None, None, None
  return True, _finite(get('dRel')), _finite(get('vRel')), _finite(get('modelProb')), bool(get('radar', False))


class LeadLossLog:
  """radard: one event when a lead that mattered (near, or being closed on) disappears, and one when a lead returns,
  with where it would have been had it been tracked all along (a stopped car lost and found again shows up as
  "returned at about the expected distance")."""

  def __init__(self, log=None):
    self.log = log
    self.last: dict | None = None
    self.lost: dict | None = None

  def _emit(self, **fields) -> None:
    try:
      log = self.log
      if log is None:
        from openpilot.common.swaglog import cloudlog
        log = cloudlog.event
      log(EVENT, feature='lead_loss', **fields)
    except Exception:
      pass

  def observe(self, t: float, lead_one, v_ego: float) -> None:
    present, d, v, prob, radar = _lead_fields(lead_one)
    if present:
      if self.lost is not None and t - self.lost['t'] <= LEAD_REGAIN_WINDOW_S:
        gap = t - self.lost['t']
        expected = max(self.lost['drel'] + self.lost['vrel'] * gap, 0.0)
        self._emit(what='returned', gap_s=round(gap, 2), drel=d, vrel=v, model_prob=prob, radar=radar,
                   expected_drel=round(expected, 1), v_ego=_finite(v_ego),
                   likely_same_object=abs(d - expected) <= max(10.0, 0.3 * self.lost['drel']))
      self.lost = None
      self.last = {'t': t, 'drel': d, 'vrel': v, 'model_prob': prob, 'radar': radar}
      return
    last, self.last = self.last, None
    if last is None or t - last['t'] > LEAD_SAMPLE_MAX_GAP_S:
      return
    if last['drel'] < LEAD_LOSS_NEAR_M or (last['drel'] < LEAD_LOSS_MAX_M and last['vrel'] < LEAD_LOSS_CLOSING_MPS):
      self.lost = last
      self._emit(what='lost', drel=last['drel'], vrel=last['vrel'], model_prob=last['model_prob'], radar=last['radar'],
                 v_ego=_finite(v_ego), time_to_reach_s=round(last['drel'] / -last['vrel'], 1) if last['vrel'] < -0.1 else None)


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
