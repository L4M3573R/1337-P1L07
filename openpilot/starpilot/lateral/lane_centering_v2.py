"""Lane centering v2 (2026-11-17): the same controller, limits and resets, but it keeps the lane through brief doubt.

The driver found centering "sporadic". On recorded drives (21 min, then 23 held-out min on 9 other routes) the cause
was centering letting go and grabbing again several times a minute: the single-line tracker blocks whenever neither
lane line passes 60 % / 0.3 m for one frame, and its correction fades out and back in. v2 changes three things:
  - sticky trust: a line trusted once stays trusted until it clearly drops (45 % / 0.4 m), new lines still need 60 % / 0.3 m;
  - a short memory: when both lines vanish for a moment, it keeps steering from the lane it just saw, fading over
    MEMORY_S, instead of blocking (the hold is what removed most drop-outs);
  - the lane is judged over 0.8-2.0 s ahead (weighted toward near) with a soft dead zone, not one point with a hard edge.
Replay (current vs v2): drop-outs 4.7 -> 3.1 and, held out, 6.5 -> 3.8 per minute; time lost to line doubt 7.0 -> 4.7 %;
pushes toward the car's center 99 % for both; near a line both push away the same share; jitter unchanged.

LaneShadow runs one controller beside the one that steers and logs, once a minute, how each did (feature
'lane_centering_shadow'). lane_centering_v2.json {"mode": "shadow" | "on" | "off"} (default shadow): shadow steers
with the current controller and shadows v2; on steers with v2 and shadows the current one.
"""
from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path

from openpilot.starpilot.lateral import lane_centering as lc
from openpilot.starpilot.lateral.lane_centering import LaneCenteringController, LaneCenteringInput, LaneCenteringResult, _at, _path, _path_xy
from openpilot.starpilot.lateral.lane_tracking import SingleLineTracker, TrackingTarget

ENTER = (0.6, 0.3)      # a new line: probability at least, std at most (as the current controller)
STAY = (0.45, 0.4)      # a trusted line stays trusted down to this
HOLD_S = 1.0            # both lines gone: keep steering from the remembered lane this long
MEMORY_S = 1.0
TIMES = (0.8, 1.2, 1.6, 2.0)
WEIGHTS = (0.4, 0.3, 0.2, 0.1)
DEAD_M, SOFT_M = 0.04, 0.12
MODES = ('shadow', 'on', 'off')


class StickyModel:
  """The model as the controller sees it, with the sticky line confidences."""

  def __init__(self, model, probs, stds):
    self._model, self.laneLineProbs, self.laneLineStds = model, probs, stds

  def __getattr__(self, name):
    return getattr(self._model, name)


class StickyTrust:
  def __init__(self):
    self.reset()

  def reset(self):
    self.trusted, self.clock, self.last_trusted = [False, False], 0.0, None

  def view(self, model, dt: float):
    self.clock += dt
    probs, stds = [float(x) for x in model.laneLineProbs], [float(x) for x in model.laneLineStds]
    if len(probs) < 3 or len(stds) < 3:
      return model
    for i in (0, 1):
      p, s = probs[i + 1], stds[i + 1]
      limit = STAY if self.trusted[i] else ENTER
      self.trusted[i] = math.isfinite(p) and math.isfinite(s) and p >= limit[0] and 0.0 <= s <= limit[1]
      if self.trusted[i]:
        probs[i + 1], stds[i + 1] = max(p, ENTER[0] + 0.01), min(s, ENTER[1] - 0.01)
    if any(self.trusted):
      self.last_trusted = self.clock
    return StickyModel(model, probs, stds)

  def holding(self) -> bool:
    return not any(self.trusted) and self.last_trusted is not None and self.clock - self.last_trusted < HOLD_S


class HoldingTracker(SingleLineTracker):
  """The current single-line tracker, but a brief loss of both lines does not block: the memory carries it."""

  def __init__(self, sticky: StickyTrust):
    self.sticky = sticky
    super().__init__()

  def target(self, *args, **kwargs) -> TrackingTarget:
    result = super().target(*args, **kwargs)
    if result.blocked and self.sticky.holding():
      return TrackingTarget()
    return result


class PreviewReading:
  """Both lines: lane center against the planned path over TIMES ahead; one line: the current single-line reading;
  none (briefly): the remembered errors, fading."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.memory = None   # (errors per lookahead, age in s)

  def raw(self, model, speed, offset, authority, *, paths=None, single_target=None, dt=0.01, update=True,
          trusted=(False, False)) -> float | None:
    try:
      if speed < lc.MIN_SPEED:
        return None
      left, right, path = _path(model.laneLines[1]), _path(model.laneLines[2]), _path(model.position)
      if path is None:
        return None
      xs = [min(60.0, max(8.0, speed * t)) for t in TIMES]
      errors = None
      if all(trusted) and left is not None and right is not None:
        found = []
        for x in xs:
          l, r, y = _at(left, x), _at(right, x), _at(path, x)
          if l is None or r is None or y is None:
            found.append(None)   # beyond a line's reach: the other lookaheads still count
            continue
          if not lc.MIN_LANE_WIDTH <= r - l <= lc.MAX_LANE_WIDTH:
            found = []
            break
          safe = min(lc.MAX_OFFSET, max(0.0, (r - l) * 0.5 - lc.MIN_CENTER_TO_LINE))
          found.append((l + r) * 0.5 + max(-safe, min(safe, offset)) - y)
        if found and any(e is not None for e in found):
          errors = found
      if errors is None and any(trusted):
        single = lc.raw_correction(model, speed, offset, authority, _paths=paths, _single_target=single_target)
        if single is not None:
          return single
      if errors is not None:
        if update:
          self.memory = (errors, 0.0)
      elif self.memory is not None and self.memory[1] < MEMORY_S:
        fade = 1.0 - self.memory[1] / MEMORY_S
        errors = [None if e is None else e * fade for e in self.memory[0]]
        if update:
          self.memory = (self.memory[0], self.memory[1] + dt)
      else:
        return None
      std_path = _path_xy(model.position.x, model.position.yStd)
      total = weight_sum = 0.0
      for x, w, e in zip(xs, WEIGHTS, errors, strict=True):
        if e is None:
          continue
        size = max(0.0, abs(e) - DEAD_M) * min(1.0, abs(e) / SOFT_M)   # soft dead zone: no on/off edge
        e2 = math.copysign(size, e)
        ps = _at(std_path, x) if std_path is not None else None
        if ps is not None and 0.0 <= ps <= lc.E2E_MAX_PATH_STD:
          break_in = min(1.0, max(0.0, (abs(e) - lc.E2E_BREAK_IN_START) / (lc.E2E_BREAK_IN_FULL - lc.E2E_BREAK_IN_START)))
          e2 *= 1.0 - min(1.0, max(0.0, authority)) * break_in
        total += w * 2.0 * e2 / (x * x)
        weight_sum += w
      if weight_sum <= 0:
        return None
      out = total / weight_sum
      return out if math.isfinite(out) else None
    except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
      return None


class LaneCenteringControllerV2(LaneCenteringController):
  # The model changes 20 times a second and the controller runs 100: the sticky view and the lane reading are worked
  # out once per model frame and reused until the next one (the smoothing still runs every tick). Without this v2 cost
  # 2.3x the current controller per tick.

  def __init__(self, **kwargs):
    self.sticky = StickyTrust()
    self.preview = PreviewReading()
    super().__init__(**kwargs)
    self._tracker = HoldingTracker(self.sticky)
    self._clear_frame()

  def _clear_frame(self) -> None:
    self._frame, self._view, self._cache, self._since_frame, self._frame_dt, self._fresh = None, None, {}, 0.0, 0.01, False

  def reset(self) -> None:
    super().reset()
    self.sticky.reset()
    self.preview.reset()
    self._clear_frame()

  def _raw_correction(self, model, speed, offset, authority, *, _paths=None, _single_target=None):
    # per frame: the single-line target drifts a hair with speed each tick, which must not defeat the reuse
    key = (authority == 0.0, _single_target is None, offset)
    if key not in self._cache:
      # only the first reading of a new frame moves the memory (the controller also asks without the path preference)
      self._cache[key] = self.preview.raw(model, speed, offset, authority, paths=_paths, single_target=_single_target,
                                          dt=self._frame_dt, update=self._fresh, trusted=tuple(self.sticky.trusted))
      self._fresh = False
    return self._cache[key]

  def update(self, observation: LaneCenteringInput) -> LaneCenteringResult:
    if isinstance(observation, LaneCenteringInput):
      dt = lc._finite_number(observation.elapsed_seconds)
      self._since_frame += dt if dt is not None and dt > 0 else 0.0
      frame = (observation.model_sample_id, id(observation.model))
      if frame != self._frame or self._view is None:
        self._frame, self._frame_dt, self._since_frame = frame, max(self._since_frame, 0.01), 0.0
        self._cache, self._fresh = {}, True
        try:
          self._view = self.sticky.view(observation.model, self._frame_dt)
        except (AttributeError, TypeError, ValueError):
          self._view = observation.model
      observation = replace(observation, model=self._view)
    return super().update(observation)


def read_mode(path: Path | None = None) -> str:
  if path is None:
    from openpilot.starpilot.storage import starpilot_storage_root
    path = starpilot_storage_root() / 'lane_centering_v2.json'
  try:
    value = json.loads(path.read_bytes()[:4096]).get('mode')
  except (OSError, ValueError, AttributeError):
    return 'shadow'
  return value if value in MODES else 'shadow'


class LaneShadow:
  """Runs another controller on the same inputs as the one that steers (never touching what steers) and logs a
  one-minute summary: how much each centered, its drop-outs, how far apart their corrections were, and near a line
  how often each pushed toward it."""
  WINDOW_S = 60.0
  NEAR_LINE_M = 0.30

  def __init__(self, controller: LaneCenteringController, name: str, steering_name: str, body_width_m: float = 1.83,
               log=None):
    self.controller, self.name, self.steering_name = controller, name, steering_name
    self.half_width = body_width_m / 2.0
    self.log = log
    self.last_id = None
    self.pending_dt = 0.0
    self._new_window()

  def _new_window(self):
    self.seconds = 0.0
    self.samples = 0
    self.centering = {'steering': 0, 'shadow': 0}
    self.dropouts = {'steering': 0, 'shadow': 0}
    self.was = {'steering': None, 'shadow': None}
    self.diff_sum, self.diff_max = 0.0, 0.0
    self.near = {'steering': [0, 0], 'shadow': [0, 0]}   # toward, away

  def observe(self, observation: LaneCenteringInput, steering: LaneCenteringResult | None) -> None:
    try:
      self._observe(observation, steering)
    except Exception:   # a shadow must never cost the drive anything
      self.last_id = None

  def _observe(self, observation, steering):
    if not isinstance(observation, LaneCenteringInput) or steering is None:
      return
    dt = lc._finite_number(observation.elapsed_seconds) or 0.0
    self.pending_dt += max(dt, 0.0)
    if observation.model_sample_id == self.last_id:
      return   # the shadow runs once per model frame (20 Hz), on the time since the last one
    self.last_id = observation.model_sample_id
    shadow = self.controller.update(replace(observation, elapsed_seconds=max(self.pending_dt, 0.01)))
    self.seconds += self.pending_dt
    self.pending_dt = 0.0
    if observation.lateral_active and not observation.driver_override:
      speed = max(float(observation.speed_mps), 1.0)
      self.samples += 1
      room = self._room(observation)
      for key, result in (('steering', steering), ('shadow', shadow)):
        on = result.reason in ('qualified', 'clearance_limited')
        self.centering[key] += on
        if self.was[key] is True and not on:
          self.dropouts[key] += 1
        self.was[key] = on
        if room is not None and min(room) < self.NEAR_LINE_M and abs(result.correction) > 1e-6:
          left, right = room
          toward = (result.correction > 0 and right <= left) or (result.correction < 0 and left < right)
          self.near[key][0 if toward else 1] += 1
      diff = abs(shadow.correction - steering.correction) * speed * speed
      self.diff_sum += diff
      self.diff_max = max(self.diff_max, diff)
    if self.seconds >= self.WINDOW_S:
      self.flush()

  def _room(self, observation):
    model = observation.model
    if model.laneLineProbs[1] < 0.5 or model.laneLineProbs[2] < 0.5:
      return None
    left, right = _path(model.laneLines[1]), _path(model.laneLines[2])
    if left is None or right is None:
      return None
    lv, rv = _at(left, 2.0), _at(right, 2.0)
    if lv is None or rv is None:
      return None
    car = -float(observation.settings.offset_m)   # the car's center, right of the lens by the camera offset
    return (car - self.half_width) - lv, rv - (car + self.half_width)

  def flush(self) -> dict | None:
    if self.samples == 0:
      self._new_window()
      return None
    minutes = self.seconds / 60.0
    doc = {'feature': 'lane_centering_shadow', 'shadow': self.name, 'steering': self.steering_name,
           'minutes': round(minutes, 2),
           f'{self.steering_name}_centering': round(self.centering['steering'] / self.samples, 3),
           f'{self.name}_centering': round(self.centering['shadow'] / self.samples, 3),
           f'{self.steering_name}_dropouts': self.dropouts['steering'], f'{self.name}_dropouts': self.dropouts['shadow'],
           'correction_diff_mean': round(self.diff_sum / self.samples, 4), 'correction_diff_max': round(self.diff_max, 4),
           f'{self.steering_name}_near_line_toward_away': self.near['steering'],
           f'{self.name}_near_line_toward_away': self.near['shadow']}
    if self.log is not None:
      self.log(doc)
    else:
      from openpilot.common.swaglog import cloudlog
      from openpilot.starpilot.diagnostics.features import EVENT
      cloudlog.event(EVENT, **doc)
    self._new_window()
    return doc
