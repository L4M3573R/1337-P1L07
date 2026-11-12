"""Curve guard: in a curve, keep the car's inside body off a confidently seen inside lane line and inside road edge.

The driving model sometimes plans a path that cuts the inside of a curve. Lane centering cannot help there when
the outside line fades (it needs both lines), so this guard needs only the inside reference. When the car's body,
following the planned path, would come within the clearance of the inside line or edge, it takes a bounded amount
of turn out of the desired curvature, never more than the room on the outside allows. It never adds turn.

Model frame: x forward, y to the right, origin at the comma. Positive curvature turns right (toward +y). The car's
center sits `camera_offset_left_m` to the right of the comma.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

DEFAULT_WIDTH_M = 1.85        # when the car interface gives no body width (2024-26 Elantra: 1.83 m)
LINE_CLEARANCE_M = 0.10       # the body stays this far inside the inside lane line
EDGE_CLEARANCE_M = 0.75       # and this far from the inside road edge
OUTSIDE_LINE_CLEARANCE_M = 0.10
MIN_LINE_PROB = 0.7
MAX_LINE_STD_M = 0.3
MAX_EDGE_STD_M = 1.0
MIN_SPEED_MPS = 8.0           # above ~18 mph: curves, not intersection turns
MIN_CURVATURE = 0.002         # 1/m (500 m radius)
MAX_SHIFT_M = 0.5
UNKNOWN_OUTSIDE_SHIFT_M = 0.2  # with no confident outside reference, move at most this far out
MAX_REDUCTION_FRACTION = 0.15  # never take away more than 15% of the planned turn (0.5 m out in ~2 s)
MIN_LEVER_M = 10.0
# Look about one second ahead: far-ahead lane lines bend less than the road in tight curves and read as false crossings.
LOOKAHEAD_S = (0.3, 1.2)
LOOKAHEAD_M = (4.0, 20.0)
RISE_TAU_S = 0.3
RELEASE_TAU_S = 0.8
DT = 0.01


@dataclass(frozen=True)
class GuardResult:
  curvature: float
  active: bool = False
  inside: str = ''
  gap_m: float = math.nan       # smallest inside gap from the car's body over the lookahead (line or edge, whichever binds)
  at_m: float = math.nan
  shift_m: float = 0.0
  reason: str = 'idle'


def _y(line, x):
  lx, ly = list(line.x), list(line.y)
  if len(lx) < 2 or len(lx) != len(ly):
    return np.full_like(x, np.nan)
  return np.interp(x, lx, ly, left=np.nan, right=np.nan)


class CurveGuard:
  def __init__(self, width_m: float | None = None, camera_offset_left_m: float = 0.0, enabled: bool = True):
    width = width_m if isinstance(width_m, float) and math.isfinite(width_m) and 1.3 <= width_m <= 2.6 else DEFAULT_WIDTH_M
    offset = camera_offset_left_m if isinstance(camera_offset_left_m, (int, float)) and math.isfinite(camera_offset_left_m) else 0.0
    self.half_width = width / 2.0
    self.camera_offset_left = min(max(float(offset), -0.3), 0.3)
    self.enabled = enabled
    self.delta = 0.0  # smoothed curvature taken out (always toward less turn)

  def reset(self):
    self.delta = 0.0

  def _target(self, model, curvature: float, v_ego: float) -> GuardResult:
    right_turn = curvature > 0
    inside = 'right' if right_turn else 'left'
    lines, edges = model.laneLines, model.roadEdges
    if len(lines) != 4 or len(edges) != 2 or len(model.laneLineProbs) != 4 or len(model.roadEdgeStds) != 2:
      return GuardResult(curvature, reason='no lines')
    window = np.linspace(max(LOOKAHEAD_M[0], LOOKAHEAD_S[0] * v_ego), min(LOOKAHEAD_M[1], max(LOOKAHEAD_S[1] * v_ego, LOOKAHEAD_M[0] + 1.0)), 10)
    center = _y(model.position, window) + self.camera_offset_left
    near_side = center + self.half_width if right_turn else center - self.half_width
    far_side = center - self.half_width if right_turn else center + self.half_width
    sign = 1.0 if right_turn else -1.0  # gap = sign * (reference - near side) is positive while clear

    in_i, out_i = (2, 1) if right_turn else (1, 2)
    in_edge, out_edge = (1, 0) if right_turn else (0, 1)
    probs, line_stds, edge_stds = model.laneLineProbs, model.laneLineStds, model.roadEdgeStds
    line_ok = lambda i: probs[i] >= MIN_LINE_PROB and (len(line_stds) != 4 or line_stds[i] <= MAX_LINE_STD_M)  # noqa: E731

    needs = []
    if line_ok(in_i):
      gap = sign * (_y(lines[in_i], window) - near_side)
      if np.isfinite(gap).any():
        k = int(np.nanargmin(gap))
        needs.append((LINE_CLEARANCE_M - gap[k], gap[k], window[k]))
    if edge_stds[in_edge] <= MAX_EDGE_STD_M:
      gap = sign * (_y(edges[in_edge], window) - near_side)
      if np.isfinite(gap).any():
        k = int(np.nanargmin(gap))
        needs.append((EDGE_CLEARANCE_M - gap[k], gap[k], window[k]))
    if not needs:
      return GuardResult(curvature, inside=inside, reason='no confident inside reference')
    need, gap, at = max(needs, key=lambda n: n[0])
    if need <= 0.0:
      return GuardResult(curvature, inside=inside, gap_m=float(gap), at_m=float(at), reason='clear')

    rooms = []
    if line_ok(out_i):
      room = -sign * (_y(lines[out_i], window) - far_side)
      if np.isfinite(room).any():
        rooms.append(float(np.nanmin(room)) - OUTSIDE_LINE_CLEARANCE_M)
    if edge_stds[out_edge] <= MAX_EDGE_STD_M:
      room = -sign * (_y(edges[out_edge], window) - far_side)
      if np.isfinite(room).any():
        rooms.append(float(np.nanmin(room)) - EDGE_CLEARANCE_M)
    room = min(rooms) if rooms else UNKNOWN_OUTSIDE_SHIFT_M
    shift = max(0.0, min(float(need), room, MAX_SHIFT_M))
    if shift <= 0.0:
      return GuardResult(curvature, inside=inside, gap_m=float(gap), at_m=float(at), reason='no room outside')
    lever = max(float(at), MIN_LEVER_M)
    delta = min(2.0 * shift / lever ** 2, MAX_REDUCTION_FRACTION * abs(curvature))
    return GuardResult(curvature - math.copysign(delta, curvature), True, inside, float(gap), float(at), shift, 'guarding')

  def update(self, model, curvature: float, v_ego: float, *, lat_active: bool, blinker: bool, lane_change: bool) -> GuardResult:
    if (not self.enabled or not lat_active or blinker or lane_change or not math.isfinite(curvature) or
        not math.isfinite(v_ego) or v_ego < MIN_SPEED_MPS or abs(curvature) < MIN_CURVATURE or model is None):
      target = GuardResult(curvature, reason='inactive')
    else:
      try:
        target = self._target(model, curvature, v_ego)
      except (AttributeError, IndexError, TypeError, ValueError):
        target = GuardResult(curvature, reason='bad model data')
    wanted = abs(curvature - target.curvature)
    tau = RISE_TAU_S if wanted > self.delta else RELEASE_TAU_S
    self.delta += (wanted - self.delta) * min(1.0, DT / tau)
    if not math.isfinite(curvature) or abs(curvature) < 1e-9:
      self.delta = 0.0
      return GuardResult(curvature, reason=target.reason)
    # never more than the cap for the current curvature, never past straight
    applied = min(self.delta, MAX_REDUCTION_FRACTION * abs(curvature))
    guarded = curvature - math.copysign(applied, curvature)
    return GuardResult(guarded, applied > 1e-6, target.inside, target.gap_m, target.at_m, target.shift_m, target.reason)
