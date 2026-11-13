"""A speed plan for the road ahead, chunk by chunk: radius, a safe speed for every chunk, then one smooth profile.

Pure functions. Each 10 m chunk of the mapped road gets the lowest of three limits:

  comfort   the driver's learned lateral acceleration for that chunk's tightness
  steering  what the comma's steering torque can hold at that speed (measured on this car from 6.3 h of logged
            steering, 2026-11-13) times a margin: on highway curves comfort is the limit, on tight curves under
            ~45 mph the torque is
  jerk      how fast the tightness changes there: S-bends and quickly tightening curves need a slower, flowing
            transition (lateral jerk = v^3 * dk/ds)

then a backward pass makes the plan reachable with a gentle, coasting-like slowdown, so a tight spot 600 m ahead
starts an early ease-off instead of late braking, a tightening curve is eased into progressively, and an opening
curve lets the car speed back up as it opens. The allowed speed now is the plan's first value.
"""
from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from openpilot.starpilot.curve_speed.map_curves import MapCurve

MPH = 0.44704
# lateral acceleration (m/s^2) the comma holds at full steering torque, by speed (m/s)
TORQUE_CAP_V = (10 * MPH, 20 * MPH, 30 * MPH, 40 * MPH, 50 * MPH, 60 * MPH, 72 * MPH)
TORQUE_CAP_A = (1.45, 1.85, 2.89, 3.39, 3.85, 4.65, 4.30)
TORQUE_MARGIN = 0.8          # never plan a curve needing more than 80% of the steering the comma has
STRUGGLE_MARGIN = 0.65       # a curve where the comma needed help is planned with more room
MAX_LATERAL = 3.0            # the car's lateral acceleration limit (never clipped in practice)
APPROACH_DECEL = 0.8         # m/s^2, about what lifting off the pedal does
MIN_SPEED = 15 * MPH
UNKNOWN_MAP_MARGIN = 1.1     # a stretch known only from the map is assumed 10% tighter until driven
JERK_MAX = 1.5               # m/s^3 lateral jerk through changes in tightness (tuned later from the shadow logs)
JERK_SMOOTH_M = 50.0         # tightness changes are measured on a 50 m average: map polyline corners are not real
STRAIGHT_K = 1 / 3000        # gentler than this needs no comfort or steering limit
SCALE_RANGE = (0.5, 2.0)     # how far a remembered curve may correct the map's tightness


CapOf = Callable[[float], float]  # speed -> lateral acceleration at full steering torque (the learned capability)


def torque_cap(speed: float, margin: float = TORQUE_MARGIN, cap_of: CapOf | None = None) -> float:
  return (cap_of(speed) if cap_of is not None else float(np.interp(speed, TORQUE_CAP_V, TORQUE_CAP_A))) * margin


@dataclass(frozen=True)
class CurveSpeed:
  apex_speed: float          # m/s through the curve
  limit: str                 # 'comfort' or 'steering'
  comfort: float             # m/s^2 used
  curvature: float           # 1/m used


def apex_speed(curvature: float, comfort: float, margin: float = TORQUE_MARGIN, cap_of: CapOf | None = None) -> CurveSpeed:
  """Fastest speed whose lateral acceleration stays within comfort and the steering capability at that speed."""
  k = max(abs(curvature), 1e-5)
  comfort = min(comfort, MAX_LATERAL)
  v = math.sqrt(comfort / k)
  limit = 'comfort'
  for _ in range(6):  # the torque cap depends on speed: settle the fixed point
    cap = torque_cap(v, margin, cap_of)
    if v * v * k <= cap + 1e-6:
      break
    v = math.sqrt(cap / k)
    limit = 'steering'
  return CurveSpeed(max(v, MIN_SPEED), limit, comfort, k)


# (curvature scale, comfort or None for the learned comfort curve, steering margin) for one chunk
Adjust = tuple[float, float | None, float]


def adjustments(distances: np.ndarray, curves: list[MapCurve], memory_of: Callable | None = None,
                curvature: np.ndarray | None = None) -> list[Adjust]:
  """Per-chunk corrections. Inside a curve driven before: its measured tightness (as a scale on the map's shape) and
  comfort, point by point along the curve when its profile is known (so a tightening or opening curve is planned the
  way it was actually driven), and its steering margin. Everywhere else the unknown-map margin."""
  out: list[Adjust] = [(UNKNOWN_MAP_MARGIN, None, TORQUE_MARGIN)] * len(distances)
  for curve in curves:
    remembered = memory_of(curve) if memory_of is not None else None
    if remembered is None:
      continue
    measured, comfort, margin = remembered[:3]
    profile = remembered[3] if len(remembered) > 3 else None
    whole = float(np.clip(measured / max(curve.curvature, 1e-6), *SCALE_RANGE))
    length = max(curve.end_m - curve.start_m, 1e-6)
    for i, d in enumerate(distances):
      if not curve.start_m <= d <= curve.end_m:
        continue
      if profile:
        pos = (d - curve.start_m) / length
        here_k = float(np.interp(pos, [q[0] for q in profile], [q[1] for q in profile]))
        here_comfort = float(np.interp(pos, [q[0] for q in profile], [q[2] for q in profile]))
        map_k = abs(float(curvature[i])) if curvature is not None else curve.curvature
        scale = float(np.clip(here_k / map_k, *SCALE_RANGE)) if map_k >= STRAIGHT_K * 3 else whole
        out[i] = (scale, here_comfort, margin)
      else:
        out[i] = (whole, comfort, margin)
  return out


@dataclass(frozen=True)
class Plan:
  distances: np.ndarray      # m along the road from the car
  curvature: np.ndarray      # |1/m| used per chunk (after corrections)
  limit: np.ndarray          # each chunk's own speed limit, m/s (inf on straights)
  reason: tuple[str, ...]    # 'comfort', 'steering', 'jerk' or '' per chunk
  speed: np.ndarray          # the reachable plan after the backward pass, m/s

  @property
  def now(self) -> float:
    return float(self.speed[0]) if len(self.speed) else math.inf

  @property
  def binding(self) -> int | None:
    """The chunk that sets the speed now, or None when nothing ahead limits it."""
    if not len(self.limit) or not np.isfinite(self.limit).any():
      return None
    reach = np.sqrt(np.square(self.limit) + 2 * APPROACH_DECEL * self.distances)
    return int(np.argmin(reach))


def plan(distances: np.ndarray, curvature: np.ndarray, comfort_of: Callable[[float], float],
         adjust: list[Adjust] | None = None, cap_of: CapOf | None = None) -> Plan:
  """The speed plan for the road ahead. curvature is signed (left positive) so S-bend transitions count as change."""
  n = len(distances)
  if n == 0:
    return Plan(distances, curvature, np.zeros(0), (), np.zeros(0))
  adjust = adjust or [(UNKNOWN_MAP_MARGIN, None, TORQUE_MARGIN)] * n
  signed = np.array([k * a[0] for k, a in zip(curvature, adjust, strict=True)], dtype=float)
  k = np.abs(signed)
  if n > 1:
    step = max(float(np.median(np.diff(distances))), 1e-3)
    width = max(1, int(round(JERK_SMOOTH_M / step)))
    smooth = np.convolve(np.pad(signed, (width // 2, width - 1 - width // 2), mode='edge'), np.ones(width) / width, mode='valid')
    dkds = np.abs(np.gradient(smooth, distances))
  else:
    dkds = np.zeros(1)
  limit, reason = np.full(n, math.inf), [''] * n
  for i in range(n):
    if k[i] >= STRAIGHT_K:
      _, comfort, margin = adjust[i]
      ki = float(k[i])  # the learned comfort curve takes plain floats only
      cs = apex_speed(ki, comfort if comfort is not None else comfort_of(ki), margin, cap_of)
      limit[i], reason[i] = cs.apex_speed, cs.limit
    if dkds[i] > 1e-9:
      vj = max((JERK_MAX / dkds[i]) ** (1 / 3), MIN_SPEED)
      if vj < limit[i]:
        limit[i], reason[i] = vj, 'jerk'
  speed = limit.copy()
  for i in range(n - 2, -1, -1):  # backward: every later chunk must be reachable with a gentle slowdown
    speed[i] = min(speed[i], math.sqrt(speed[i + 1] ** 2 + 2 * APPROACH_DECEL * (distances[i + 1] - distances[i])))
  return Plan(distances, k, limit, tuple(reason), speed)
