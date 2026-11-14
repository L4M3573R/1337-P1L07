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
  """Logs one event whenever a feature's decision (the key) changes; context fields ride along.

  min_gap_s > 0 keeps a flickering decision readable: changes within that long of the last event are not logged one
  by one; the latest decision is logged once the gap has passed (if it still differs), with `flickers` = how many
  changes were folded into it."""

  def __init__(self, feature: str, log=None, min_gap_s: float = 0.0):
    self.feature = feature
    self.last_key = None
    self.log = log
    self.min_gap_s = min_gap_s
    self.last_emit = -1e18
    self.seen_key = None
    self.flickers = 0

  def observe(self, key: dict, context: dict | None = None, now: float | None = None) -> bool:
    if self.min_gap_s > 0:
      try:
        if now is None:
          import time
          now = time.monotonic()
        if key != self.seen_key:
          self.seen_key = key
          if now - self.last_emit < self.min_gap_s:
            self.flickers += 1
            return False
        if key == self.last_key or now - self.last_emit < self.min_gap_s:
          return False
        self.last_emit = now
        flickers, self.flickers = self.flickers, 0
        if flickers:
          context = {**(context or {}), 'flickers': flickers}
      except Exception:
        return False
    elif key == self.last_key:
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


ERROR_MIN_GAP_S = 30.0
_event_last: dict[tuple[str, str], list] = {}


def log_event(feature: str, slot: str, min_gap_s: float = ERROR_MIN_GAP_S, *, log=None, now: float | None = None,
              **fields) -> bool:
  """A repeating occurrence (not a decision change): at most one event per feature and slot per min_gap_s, carrying
  how many more happened in between (repeats_since_last)."""
  try:
    import time
    now = time.monotonic() if now is None else now
    last = _event_last.setdefault((feature, slot), [-1e18, 0])
    if now - last[0] < min_gap_s:
      last[1] += 1
      return False
    repeats, last[0], last[1] = last[1], now, 0
    if log is None:
      from openpilot.common.swaglog import cloudlog
      log = cloudlog.event
    log(EVENT, feature=feature, **fields, repeats_since_last=repeats)
    return True
  except Exception:
    return False


def log_error(feature: str, error: BaseException | str, *, where: str = '', log=None, now: float | None = None) -> bool:
  """A feature caught an error it can live with (it must never crash its host): log it instead of hiding it.

  At most one event per feature and place per 30 s, carrying how many more happened in between. The `error` field
  puts it at error level, so it also reaches errorLogMessage and the daily trace."""
  try:
    text = f'{type(error).__name__}: {error}' if isinstance(error, BaseException) else str(error)
  except Exception:
    text = 'unprintable error'
  return log_event(feature, 'error:' + where, log=log, now=now, what='error', where=where, error=text[:300])


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
    return False, None, None, None, None, None
  radar = bool(get('radar', False))
  track = get('radarTrackId', None) if radar else None
  return True, _finite(get('dRel')), _finite(get('vRel')), _finite(get('modelProb')), radar, (int(track) if track is not None else None)


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

  def observe(self, t: float, lead_one, v_ego: float, track_alive=None) -> None:
    """track_alive(track_id) -> bool: whether radard still holds that radar track (the lead-hold shadow)."""
    present, d, v, prob, radar, track = _lead_fields(lead_one)
    if present:
      if self.lost is not None and t - self.lost['t'] <= LEAD_REGAIN_WINDOW_S:
        gap = t - self.lost['t']
        expected = max(self.lost['drel'] + self.lost['vrel'] * gap, 0.0)
        self._emit(what='returned', gap_s=round(gap, 2), drel=d, vrel=v, model_prob=prob, radar=radar,
                   expected_drel=round(expected, 1), v_ego=_finite(v_ego),
                   likely_same_object=abs(d - expected) <= max(10.0, 0.3 * self.lost['drel']),
                   same_radar_track=track is not None and track == self.lost.get('track'))
      self.lost = None
      self.last = {'t': t, 'drel': d, 'vrel': v, 'model_prob': prob, 'radar': radar, 'track': track}
      return
    last, self.last = self.last, None
    if last is None or t - last['t'] > LEAD_SAMPLE_MAX_GAP_S:
      return
    if last['drel'] < LEAD_LOSS_NEAR_M or (last['drel'] < LEAD_LOSS_MAX_M and last['vrel'] < LEAD_LOSS_CLOSING_MPS):
      self.lost = last
      still = None
      if last.get('track') is not None and track_alive is not None:
        try:
          still = bool(track_alive(last['track']))
        except Exception as e:
          still = None
          log_error('lead_loss', e, where='track_alive')
      # lead-hold shadow: radar_still_tracking=True means a hold would have kept following this car through the gap
      self._emit(what='lost', drel=last['drel'], vrel=last['vrel'], model_prob=last['model_prob'], radar=last['radar'],
                 v_ego=_finite(v_ego), time_to_reach_s=round(last['drel'] / -last['vrel'], 1) if last['vrel'] < -0.1 else None,
                 radar_still_tracking=still)


LEAD_BRAKE_ONSET = -0.8      # m/s2: the lead counts as braking below this
LEAD_BRAKE_REARM = -0.3      # ...and a new onset can start once both estimates are back above this
LEAD_BRAKE_WAIT_S = 3.0      # how long to wait for radard's estimate to follow


class LeadAccelShadow:
  """radard (shadow): the radar's own lead acceleration (REL_ACCEL + ego accel) against radard's Kalman aLeadK. One
  event per lead braking onset: how much earlier the radar showed it, or that radard's estimate never did."""

  def __init__(self, log=None):
    self.log = log
    self.pending: dict | None = None
    self.armed = True

  def _emit(self, **fields) -> None:
    try:
      log = self.log
      if log is None:
        from openpilot.common.swaglog import cloudlog
        log = cloudlog.event
      log(EVENT, feature='lead_accel_shadow', **fields)
    except Exception:
      pass

  def observe(self, t: float, track: int | None, radar_accel: float | None, kalman_accel: float, d_rel: float, v_ego: float) -> None:
    try:
      if track is None or radar_accel is None or not math.isfinite(radar_accel) or not math.isfinite(kalman_accel):
        self.pending = None
        return
      if self.pending is not None and self.pending['track'] != track:
        self.pending = None  # a different lead: that onset can no longer be compared
      if self.pending is None and self.armed and radar_accel < LEAD_BRAKE_ONSET:
        self.armed = False
        if kalman_accel < LEAD_BRAKE_ONSET:
          self._emit(what='braking seen', radar_earlier_s=0.0, radar_accel=_finite(radar_accel), kalman_accel=_finite(kalman_accel),
                     drel=_finite(d_rel), v_ego=_finite(v_ego))
        else:
          self.pending = {'t': t, 'track': track, 'peak': radar_accel, 'drel': d_rel, 'v_ego': v_ego}
      if self.pending is not None:
        self.pending['peak'] = min(self.pending['peak'], radar_accel)
        waited = t - self.pending['t']
        if kalman_accel < LEAD_BRAKE_ONSET or waited > LEAD_BRAKE_WAIT_S:
          self._emit(what='braking seen', radar_earlier_s=round(waited, 2) if kalman_accel < LEAD_BRAKE_ONSET else None,
                     radar_peak=_finite(self.pending['peak']), kalman_accel=_finite(kalman_accel), drel=_finite(self.pending['drel']),
                     v_ego=_finite(self.pending['v_ego']))
          self.pending = None
      if radar_accel > LEAD_BRAKE_REARM and kalman_accel > LEAD_BRAKE_REARM:
        self.armed = True
    except Exception as e:
      self.pending = None
      log_error('lead_accel_shadow', e, where='observe')


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
