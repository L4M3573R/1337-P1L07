"""Remember every curve driven and learn from how it went, above all from the driver taking over.

A pass is one curve actually driven (measured curvature from the steering angle above CURVE_K for >= MIN_PASS_S).
What happened in the approach and through the curve decides what is learned:

  brake or cruise cancel  -> too fast: this curve and the global comfort curve come down
  gas pressed while the comma drove the speed -> too slow: comfort goes up
  driver steering while the comma steered, or steering at its torque limit -> the comma could not hold it:
                             this curve gets more steering margin next time
  driven by the driver (no cruise) -> the driver's own speed is the best comfort sample there is
  comma drove it, no takeover -> a light confirmation that the speed was acceptable

Pure: no messaging. The daemon feeds samples and owns persistence timing.
"""
from __future__ import annotations

import json
import math
import os
import tempfile
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from openpilot.starpilot.curve_speed.learning import LearnedCurve
from openpilot.starpilot.curve_speed.map_target import STRUGGLE_MARGIN, TORQUE_MARGIN
from openpilot.starpilot.curve_speed.steering_capability import SteeringCapability

CURVE_K = 0.002            # 500 m radius
MIN_SPEED = 7.0            # m/s; slower is a turn or parking, not a curve
MIN_PASS_S = 2.0
APPROACH_S = 6.0           # takeovers this long before the curve count for it
STRUGGLE_SATURATED_S = 0.3
GRID_DEG = 0.00025         # ~25 m apex cells
MATCH_RADIUS_M = 40.0
MATCH_BEARING_DEG = 30.0
MAX_CURVES = 5000
EMA_NEW = 0.3

ASSIST_S = 0.5             # driver adding steering into the curve this long = the comma was not turning enough
SLOWED_MPS = 2.0           # dropping this much from the approach to the curve = the speed was chosen for the curve
FOLLOW_GAP_S = 3.0         # a car this close (in seconds) at no more than our speed sets the speed, not the curve
FOLLOW_SHARE = 0.3         # ...for this share of the approach and curve
PROFILE_POINTS = 11        # a curve is learned at 0%, 10% ... 100% of the way through it

# comfort-curve weights per verdict (LearnedCurve history caps at 600 samples per tightness bucket). Your own speed
# through a curve teaches your comfort when the curve set it (you slowed for it, no car ahead). A pass where the curve
# did not set the speed (cruising, the comma's own speed, a car ahead) only shows that speed was fine: a lower bound
# that may raise comfort, never lower it.
WEIGHTS = {'manual': 10, 'confirmed': 2, 'too fast': 20, 'too slow': 20, 'struggled': 0, 'following': 0}
LOWER_BOUND_WEIGHT = 2


@dataclass
class Sample:
  t: float
  v: float
  curvature: float          # measured, from the steering angle (signed)
  lat_active: bool
  long_active: bool
  brake: bool
  gas: bool
  steering_pressed: bool
  saturated: bool
  cruise_enabled: bool
  latitude: float
  longitude: float
  bearing: float
  driver_torque: float = 0.0  # signed like curvature (left positive)
  torque: float = 0.0         # the comma's steering command, -1..1
  lead_gap_s: float = math.inf  # time gap to the car ahead (inf: none)
  lead_speed: float = math.nan  # its speed, m/s


@dataclass
class CurvePass:
  start_t: float
  duration: float
  curvature: float          # 80th percentile |k| through the curve
  lateral: float            # 90th percentile lateral acceleration felt
  apex_speed: float
  latitude: float
  longitude: float
  bearing: float
  comma_speed: bool         # the comma controlled speed through it
  comma_steering: bool
  verdict: str              # manual / confirmed / too fast / too slow / struggled
  events: list[str] = field(default_factory=list)
  slowed: bool = False      # speed came down for this curve (so the lateral felt is a chosen comfort, not a bound)
  following: bool = False   # a car ahead set the speed (its brake and gas presses say nothing about the curve)
  chosen_lateral: float = 0.0  # lateral acceleration at the tightest point: the speed actually taken there
  # [position 0..1 through the curve, |curvature|, lateral acceleration, 1 if the curve set the speed there] per point:
  # how tight each part really was and how fast it was taken, so tightening and opening curves are learned as driven
  profile: list[list[float]] = field(default_factory=list)


def pass_profile(curve: list, approach_speed: float) -> list[list[float]]:
  """Bin a driven curve into PROFILE_POINTS positions by distance travelled through it."""
  t = np.array([x.t for x in curve])
  v = np.array([x.v for x in curve])
  k = np.array([abs(x.curvature) for x in curve])
  s = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(t))])
  if s[-1] <= 0:
    return []
  pos = s / s[-1]
  out = []
  for target in np.linspace(0.0, 1.0, PROFILE_POINTS):
    near = np.abs(pos - target) <= 0.5 / (PROFILE_POINTS - 1) + 1e-9
    kk, vv = float(np.median(k[near])), float(np.median(v[near]))
    out.append([round(float(target), 2), kk, kk * vv * vv, 1.0 if vv <= approach_speed - SLOWED_MPS else 0.0])
  return out


def verdict_for(events: set[str], comma_speed: bool, following: bool = False) -> str:
  if following and not ({'steer', 'saturated'} & events):
    return 'following'
  if 'brake' in events or 'cancel' in events:
    return 'too fast'
  if 'steer' in events or 'saturated' in events:
    return 'struggled'
  if 'gas' in events and comma_speed:
    return 'too slow'
  return 'confirmed' if comma_speed else 'manual'


class PassTracker:
  """Feed one Sample per frame; returns a CurvePass when a curve ends."""

  def __init__(self):
    self._recent: deque[Sample] = deque()
    self._curve: list[Sample] = []
    self._approach: list[Sample] = []  # the APPROACH_S before the current curve began, kept for the whole curve
    # Which way the car reports driver torque relative to curvature varies by port (this Elantra: opposite in 99% of
    # 3,597 hand-steered curve samples, 2026-11-13). Learned from hand steering; starts with that measurement.
    self._same_sign, self._opposite_sign = 0, 1

  @property
  def torque_sign(self) -> int:
    return 1 if self._same_sign > self._opposite_sign else -1

  def add(self, s: Sample) -> CurvePass | None:
    in_curve = abs(s.curvature) >= CURVE_K and s.v >= MIN_SPEED
    if s.steering_pressed and not s.lat_active and abs(s.curvature) >= 2 * CURVE_K and s.driver_torque != 0:
      if s.driver_torque * s.curvature > 0:
        self._same_sign += 1
      else:
        self._opposite_sign += 1
    self._recent.append(s)
    while self._recent and s.t - self._recent[0].t > APPROACH_S:
      self._recent.popleft()
    if in_curve:
      if not self._curve:
        self._approach = [x for x in self._recent if x.t < s.t]  # snapshot now: a long curve outlasts the window
      self._curve.append(s)
      return None
    if not self._curve:
      return None
    curve, self._curve = self._curve, []
    approach, self._approach = self._approach, []
    if curve[-1].t - curve[0].t < MIN_PASS_S:
      return None
    return self._summarize(curve, approach, self.torque_sign)

  @staticmethod
  def _summarize(curve: list[Sample], approach: list[Sample], torque_sign: int = -1) -> CurvePass:
    k = np.array([abs(x.curvature) for x in curve])
    v = np.array([x.v for x in curve])
    lat = k * v * v
    apex = int(np.argmax(k))
    events: set[str] = set()
    window = approach + curve
    for prev, x in zip(window, window[1:], strict=False):
      if x.brake and not prev.brake and prev.long_active:  # driving yourself, braking is just choosing your speed
        events.add('brake')
      if prev.cruise_enabled and not x.cruise_enabled and prev.long_active:
        events.add('cancel')
      if x.gas and not prev.gas and x.long_active:
        events.add('gas')
    pairs = list(zip(curve, curve[1:], strict=False))
    saturated_s = sum(max(0.0, b.t - a.t) for a, b in pairs if b.saturated and b.lat_active)
    # Hands resting on the wheel trip steeringPressed; only steering added in the curve's direction means the comma
    # was not turning enough. Pushing the other way is about the line, not the speed.
    assist_s = sum(max(0.0, b.t - a.t) for a, b in pairs
                   if b.steering_pressed and b.lat_active and torque_sign * b.driver_torque * b.curvature > 0)
    if assist_s >= ASSIST_S:
      events.add('steer')
    if saturated_s >= STRUGGLE_SATURATED_S:
      events.add('saturated')
    recent = [x for x in approach if x.t >= curve[0].t - 3.0] + curve
    following = sum(x.lead_gap_s < FOLLOW_GAP_S and not x.lead_speed > x.v + 1.0 for x in recent) > FOLLOW_SHARE * len(recent)
    if following:
      events -= {'brake', 'cancel', 'gas'}  # those were about the car ahead, not the curve
    comma_speed = sum(x.long_active for x in curve) > 0.8 * len(curve)
    comma_steering = sum(x.lat_active for x in curve) > 0.8 * len(curve)
    approach_speed = max((x.v for x in approach), default=float(v[0]))
    a = curve[apex]
    return CurvePass(curve[0].t, curve[-1].t - curve[0].t, float(np.percentile(k, 80)), float(np.percentile(lat, 90)),
                     float(v[apex]), a.latitude, a.longitude, a.bearing, comma_speed, comma_steering,
                     verdict_for(events, comma_speed, following), sorted(events), approach_speed - float(v.min()) >= SLOWED_MPS,
                     following, float(lat[apex]), pass_profile(curve, approach_speed))


@dataclass
class CurveRecord:
  latitude: float
  longitude: float
  bearing: float
  curvature: float
  comfort: float
  passes: int = 0
  too_fast: int = 0
  too_slow: int = 0
  struggles: int = 0
  # [position 0..1, |curvature| measured there, comfort there] per PROFILE_POINTS point; empty until driven
  profile: list[list[float]] = field(default_factory=list)

  @property
  def margin(self) -> float:
    return STRUGGLE_MARGIN if self.struggles > 0 else TORQUE_MARGIN


def _distance_m(lat1, lon1, lat2, lon2) -> float:
  dy = (lat2 - lat1) * 111_320.0
  dx = (lon2 - lon1) * 111_320.0 * math.cos(math.radians((lat1 + lat2) / 2))
  return math.hypot(dx, dy)


def _bearing_diff(a: float, b: float) -> float:
  return abs((a - b + 180.0) % 360.0 - 180.0)


class CurveMemory:
  def __init__(self, path: Path | None = None):
    self.path = path
    self.records: dict[str, CurveRecord] = {}
    self.comfort = LearnedCurve()
    self.capability = SteeringCapability()
    self.dirty = False

  @staticmethod
  def _cell(latitude: float, longitude: float) -> tuple[int, int]:
    return int(math.floor(latitude / GRID_DEG)), int(math.floor(longitude / GRID_DEG))

  def find(self, latitude: float, longitude: float, bearing: float) -> CurveRecord | None:
    cy, cx = self._cell(latitude, longitude)
    best, best_d = None, MATCH_RADIUS_M
    for dy in (-2, -1, 0, 1, 2):
      for dx in (-2, -1, 0, 1, 2):
        for key in (f'{cy + dy}:{cx + dx}:{h}' for h in range(8)):
          r = self.records.get(key)
          if r is None or _bearing_diff(r.bearing, bearing) > MATCH_BEARING_DEG:
            continue
          d = _distance_m(latitude, longitude, r.latitude, r.longitude)
          if d <= best_d:
            best, best_d = r, d
    return best

  def learn(self, p: CurvePass) -> CurveRecord:
    # A takeover teaches the speed you actually took the curve at afterwards: braked down to it, or pressed on to it.
    if p.verdict == 'too fast':
      sample = min(p.chosen_lateral or p.lateral, 0.95 * p.lateral)
    elif p.verdict == 'too slow':
      sample = max(p.chosen_lateral, p.lateral)
    else:
      sample = p.lateral
    # Your own speed sets comfort only when the curve set it: you drove it, slowed for it, and no car was ahead.
    bound_only = p.verdict in ('following', 'confirmed') or (p.verdict == 'manual' and not p.slowed)
    if bound_only:
      weight = LOWER_BOUND_WEIGHT if sample > self.comfort.comfort(p.curvature) else 0
    else:
      weight = WEIGHTS[p.verdict]
    if p.profile and p.verdict in ('manual', 'confirmed', 'following'):
      # every part of the curve teaches comfort at its own tightness: a tightening curve covers several at once
      for _pos, kk, lat_here, chosen in p.profile:
        if kk < CURVE_K:
          continue
        if p.verdict == 'manual' and chosen:
          self.comfort.observe(kk, lat_here)
        elif lat_here > self.comfort.comfort(kk):
          self.comfort.observe(kk, lat_here)  # a part taken faster than the comfort curve: at least this
    else:
      for _ in range(weight // 20):
        self.comfort.nudge(p.curvature, sample)
      for _ in range(weight % 20):
        self.comfort.observe(p.curvature, sample)
    record = self.find(p.latitude, p.longitude, p.bearing)
    if record is None:
      cy, cx = self._cell(p.latitude, p.longitude)
      key = f'{cy}:{cx}:{int(((p.bearing % 360.0) + 22.5) // 45) % 8}'
      record = self.records[key] = CurveRecord(p.latitude, p.longitude, p.bearing, p.curvature,
                                               self.comfort.comfort(p.curvature))
    else:
      record.curvature = (1 - EMA_NEW) * record.curvature + EMA_NEW * p.curvature
    before = record.comfort
    if p.verdict == 'too fast':
      record.comfort = min(record.comfort, sample)
      record.too_fast += 1
    elif p.verdict == 'too slow':
      record.comfort = max(record.comfort, sample)
      record.too_slow += 1
    elif p.verdict == 'struggled':
      record.struggles += 1
    elif bound_only:
      if sample > record.comfort:
        record.comfort = (1 - EMA_NEW) * record.comfort + EMA_NEW * sample
    elif p.verdict == 'manual':  # the driver slowed for it: their chosen comfort for this curve
      record.comfort = (1 - 2 * EMA_NEW) * record.comfort + 2 * EMA_NEW * sample
    self._learn_profile(record, p, before)
    record.passes += 1
    self.dirty = True
    if len(self.records) > MAX_CURVES:  # forget the least-driven curves first
      for key in sorted(self.records, key=lambda k: self.records[k].passes)[:len(self.records) - MAX_CURVES]:
        del self.records[key]
    return record

  def _learn_profile(self, record: CurveRecord, p: CurvePass, before: float) -> None:
    """Keep each curve's shape and the speed it is taken at along it, point by point."""
    if not p.profile:
      return
    if not record.profile:
      record.profile = [[pos, kk, max(lat_here, self.comfort.comfort(max(kk, CURVE_K))) if not chosen or p.verdict != 'manual'
                         else lat_here] for pos, kk, lat_here, chosen in p.profile]
    else:
      for point, (_pos, kk, lat_here, chosen) in zip(record.profile, p.profile, strict=False):
        point[1] = (1 - EMA_NEW) * point[1] + EMA_NEW * kk
        if p.verdict == 'manual' and chosen:
          point[2] = (1 - 2 * EMA_NEW) * point[2] + 2 * EMA_NEW * lat_here
        elif p.verdict in ('manual', 'confirmed', 'following') and lat_here > point[2]:
          point[2] = (1 - EMA_NEW) * point[2] + EMA_NEW * lat_here
    if p.verdict in ('too fast', 'too slow') and before > 0:
      ratio = record.comfort / before  # a takeover moves the whole curve's speed, keeping its shape
      for point in record.profile:
        point[2] *= ratio

  def remembered(self, latitude: float, longitude: float, bearing: float):
    """(curvature, comfort, steering margin, profile) for a curve driven before, for map_target; profile is
    [[position 0..1, curvature, comfort], ...] or empty."""
    r = self.find(latitude, longitude, bearing)
    return None if r is None else (r.curvature, r.comfort, r.margin, r.profile)

  def save(self) -> None:
    if self.path is None or not self.dirty:
      return
    self.path.parent.mkdir(parents=True, exist_ok=True)
    doc = {'version': 1, 'comfort': self.comfort.document(), 'steering': self.capability.document(),
           'curves': {k: asdict(r) for k, r in self.records.items()}}
    fd, tmp = tempfile.mkstemp(prefix='.curves-', dir=self.path.parent)
    try:
      with os.fdopen(fd, 'w') as f:
        json.dump(doc, f, separators=(',', ':'))
      os.replace(tmp, self.path)
      self.dirty = False
    finally:
      if os.path.exists(tmp):
        os.unlink(tmp)

  @classmethod
  def load(cls, path: Path) -> CurveMemory:
    memory = cls(path)
    try:
      doc = json.loads(path.read_text())
      if doc.get('version') != 1:
        return memory
      result = LearnedCurve.load(doc.get('comfort'))
      records = {str(key): CurveRecord(**value) for key, value in doc.get('curves', {}).items()}
      if result.valid:
        memory.comfort = result.curve
      memory.records = records
      memory.capability = SteeringCapability.load(doc.get('steering'))
    except (OSError, ValueError, TypeError, KeyError, AttributeError):
      pass  # a missing or corrupt file starts fresh; it is never partially trusted
    return memory
