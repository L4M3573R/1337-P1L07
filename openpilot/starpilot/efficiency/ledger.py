"""Energy ledger: split a drive into short stretches, tag each with its driving context, and total
where the wheel energy went. Observe only; this is the measurement layer the efficiency learner uses.
"""
from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass, field

from openpilot.starpilot.efficiency.energy import step

MPH = 0.44704
STRETCH_M = 250.0
STRETCH_S = 30.0
SPEED_BANDS = ((15 * MPH, "<15 mph"), (30 * MPH, "15-30 mph"), (45 * MPH, "30-45 mph"), (60 * MPH, "45-60 mph"),
               (math.inf, "60+ mph"))
GRADE_BAND = 0.02  # +-2 % grade separates uphill / flat / downhill
CURVE_GENTLE = 0.002  # 1/m (500 m radius)
CURVE_TIGHT = 0.01  # 1/m (100 m radius)


@dataclass(frozen=True)
class Sample:
  t: float  # s, monotonic
  speed: float  # m/s
  accel: float  # m/s^2
  pitch: float | None  # rad, nose up positive
  mass: float  # kg
  curvature: float = 0.0  # 1/m, absolute value is used
  lead: bool = False
  standstill: bool = False
  long_active: bool = False
  experimental: bool = False
  battery_raw: float | None = None  # uncalibrated hybrid battery signal, logged for later calibration


def make_sample(t: float, mass: float, car_state, car_control=None, radar_state=None, selfdrive_state=None, live_pose=None,
                battery_raw: float | None = None) -> Sample:
  """Build one ledger sample from the latest messages (shared by the on-device daemon and the log report)."""
  cs, cc = car_state, car_control
  pitch = cc.orientationNED[1] if cc is not None and len(cc.orientationNED) == 3 else None
  yaw_rate = live_pose.angularVelocityDevice.z if live_pose is not None else 0.0
  lead = radar_state.leadOne if radar_state is not None else None
  return Sample(t=t, speed=cs.vEgo, accel=cs.aEgo, pitch=pitch, mass=mass, curvature=abs(yaw_rate) / max(cs.vEgo, 1.0),
                lead=bool(lead is not None and lead.present and lead.dRel < 80.0), standstill=bool(cs.standstill),
                long_active=bool(cc is not None and cc.longActive),
                experimental=bool(selfdrive_state is not None and selfdrive_state.experimentalMode), battery_raw=battery_raw)


def mode_of(sample: Sample) -> str:
  if not sample.long_active:
    return "driver"
  return "experimental" if sample.experimental else "chill"


@dataclass
class Stretch:
  mode: str
  start: float
  end: float = 0.0
  distance: float = 0.0
  propulsion: float = 0.0
  regen: float = 0.0
  friction: float = 0.0
  steady: float = 0.0
  pitch_m: float = 0.0  # distance-weighted pitch sum
  curve_m: float = 0.0  # distance-weighted |curvature| sum
  lead_s: float = 0.0
  stopped: bool = False
  battery_sum: float = 0.0
  battery_n: int = 0

  @property
  def duration(self) -> float:
    return max(self.end - self.start, 0.0)

  @property
  def mean_speed(self) -> float:
    return self.distance / self.duration if self.duration > 0 else 0.0

  def context(self) -> tuple[str, str, str, str, str]:
    speed = next(label for limit, label in SPEED_BANDS if self.mean_speed < limit)
    pitch = self.pitch_m / self.distance if self.distance > 0 else 0.0
    grade = "uphill" if math.tan(pitch) > GRADE_BAND else "downhill" if math.tan(pitch) < -GRADE_BAND else "flat"
    curve_avg = self.curve_m / self.distance if self.distance > 0 else 0.0
    curve = "curvy" if curve_avg > CURVE_TIGHT else "gentle curves" if curve_avg > CURVE_GENTLE else "straight"
    traffic = "stop-and-go" if self.stopped else "following" if self.lead_s > 0.5 * self.duration else "open road"
    return speed, grade, curve, traffic, self.mode


@dataclass
class Ledger:
  stretches: list[Stretch] = field(default_factory=list)
  _open: Stretch | None = None
  _last: Sample | None = None

  def add(self, sample: Sample) -> None:
    last, self._last = self._last, sample
    mode = mode_of(sample)
    if self._open is None:
      self._open = Stretch(mode, sample.t, sample.t)
      return
    dt = sample.t - last.t if last is not None else 0.0
    if not 0.0 < dt <= 1.0:  # gap in the log: start a fresh stretch
      self._close(last.t if last is not None else sample.t)
      self._open = Stretch(mode, sample.t, sample.t)
      return
    s = self._open
    speed = 0.5 * (sample.speed + last.speed)
    e = step(speed, sample.accel, sample.pitch, sample.mass, dt)
    d = max(speed, 0.0) * dt
    s.end = sample.t
    s.distance += d
    s.propulsion += e.propulsion_j
    s.regen += e.regen_j
    s.friction += e.friction_j
    s.steady += e.steady_j
    s.pitch_m += (sample.pitch or 0.0) * d
    s.curve_m += abs(sample.curvature) * d
    s.lead_s += dt if sample.lead else 0.0
    s.stopped |= sample.standstill
    if sample.battery_raw is not None and math.isfinite(sample.battery_raw):
      s.battery_sum += sample.battery_raw
      s.battery_n += 1
    if mode != s.mode or s.distance >= STRETCH_M or s.duration >= STRETCH_S:
      self._close(sample.t)
      self._open = Stretch(mode, sample.t, sample.t)

  def finish(self) -> list[Stretch]:
    if self._open is not None and self._last is not None:
      self._close(self._last.t)
    return self.stretches

  def _close(self, t: float) -> None:
    if self._open is not None and self._open.distance > 0:
      self._open.end = max(self._open.end, t)
      self.stretches.append(self._open)
    self._open = None


@dataclass
class Totals:
  distance: float = 0.0
  duration: float = 0.0
  propulsion: float = 0.0
  regen: float = 0.0
  friction: float = 0.0
  steady: float = 0.0

  def add(self, s: Stretch) -> None:
    self.distance += s.distance
    self.duration += s.duration
    self.propulsion += s.propulsion
    self.regen += s.regen
    self.friction += s.friction
    self.steady += s.steady

  def per_km(self, joules: float) -> float:
    return joules / 3600.0 / (self.distance / 1000.0) if self.distance > 0 else 0.0  # Wh/km

  @property
  def excess(self) -> float:
    """Propulsion beyond what steady cruising at the same speeds and grades needed (speed changes)."""
    return max(self.propulsion - self.steady, 0.0)


def summarize(stretches: list[Stretch]) -> tuple[Totals, dict[tuple, Totals]]:
  total, by_context = Totals(), defaultdict(Totals)
  for s in stretches:
    total.add(s)
    by_context[s.context()].add(s)
  return total, dict(by_context)
