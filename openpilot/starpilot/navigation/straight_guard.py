"""No-blinker straight guard, shadow only: the driving model starting to turn onto a mapped side road with no blinker on.

With lane lines gone at an intersection, the end-to-end model can follow a car ahead into its turn (2026-11-15, a right
turn onto a minor road with the blinker off). The blinker-turn logic cannot help there: it works through the model's
desire input, and with no blinker the desire is already none. Any fix has to act on the steering output, so it starts
here as a logger:

  candidate:  blinker off, comma steering, below CUTOFF_MPS, the model asking for at least TURN_CURVATURE to one side
              while the wheel is still near centre and the driver has not steered for PRESS_QUIET_S: the model
              starting a turn, not following one the driver is making without a blinker
  would hold: the stored road tiles (side_road.crossing) show a branch to that side within BRANCH_AHEAD_M and the car's
              own road going on straight through that junction

Each would-hold episode logs once ("would_hold", with the map and model evidence), then OUTCOME_S later what the driver
did ("outcome": took the wheel or not, the largest steering angle). Model turns with no stored road are logged too
("no_map", rate limited) to show where tiles are missing. Nothing here publishes control. Curvature is positive to the
right, as in controls (curvature = -calc_curvature(steering angle)).
"""
from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

from openpilot.common.constants import CV

FEATURE = 'straight_guard'
CUTOFF_MPS = 40 * CV.MPH_TO_MS
MIN_SPEED_MPS = 3.0
TURN_CURVATURE = 0.005  # 1/m, a 200 m radius: well past lane keeping at intersection speeds
BRANCH_AHEAD_M = 40.0
BRANCH_BEHIND_M = -5.0  # GPS lag: a junction just passed still counts
LOOKUP_PERIOD_S = 0.25
OUTCOME_S = 3.0
QUIET_S = 1.0  # a candidate must be gone this long before the same turn can log again
NO_MAP_MIN_GAP_S = 30.0
START_ANGLE_DEG = 15.0  # past this the car is already turning; the guard is about the model starting a turn
PRESS_QUIET_S = 1.5


@dataclass(frozen=True)
class Sample:
  t: float  # monotonic seconds
  speed: float
  left_blinker: bool
  right_blinker: bool
  lat_active: bool
  steering_pressed: bool
  steering_angle: float
  model_curvature: float  # the model's desired curvature, positive to the right
  lane_prob: float  # the weaker of the two lane lines next to the car
  edge_std: float  # the larger road-edge standard deviation (the model's uncertainty about the road)
  position: tuple[float, float, float] | None = None  # latitude, longitude, bearing (degrees)


def turning_side(s: Sample) -> str | None:
  """The side the model is turning to when every gate other than the map and the driver's recent steering passes."""
  if s.left_blinker or s.right_blinker or not s.lat_active or not MIN_SPEED_MPS <= s.speed <= CUTOFF_MPS:
    return None
  if s.steering_pressed or not abs(s.steering_angle) < START_ANGLE_DEG:
    return None
  if not math.isfinite(s.model_curvature) or abs(s.model_curvature) < TURN_CURVATURE:
    return None
  return 'right' if s.model_curvature > 0 else 'left'


def would_hold(side: str, crossing) -> bool:
  """The map shows a branch to that side close ahead while the car's own road goes on straight."""
  return (crossing is not None and crossing.straight_through and crossing.branch(side) is not None and
          BRANCH_BEHIND_M <= crossing.distance_m <= BRANCH_AHEAD_M)


@dataclass
class _Episode:
  t0: float
  side: str
  pressed: bool = False
  max_angle: float = 0.0
  lat_lost: bool = False


@dataclass
class StraightGuardShadow:
  """Feed it a Sample per loop; it looks up the map only while a candidate turn is under way."""
  log: Callable[..., object] | None = None  # log_event-like (feature, slot, min_gap_s, **fields); defaults to it
  _episode: _Episode | None = None
  _last_lookup: float = -math.inf
  _quiet_since: float | None = None
  _armed: bool = True
  _last_pressed: float = -math.inf

  def _emit(self, slot: str, min_gap_s: float, **fields) -> None:
    if self.log is None:
      from openpilot.starpilot.diagnostics.features import log_event
      log_event(FEATURE, slot, min_gap_s, what=slot, **fields)
    else:
      self.log(FEATURE, slot, min_gap_s, what=slot, **fields)

  def step(self, s: Sample, crossing_at: Callable[[float, float, float], tuple[bool, object]]) -> str | None:
    """crossing_at(lat, lon, bearing) -> (map_known, Crossing | None). Returns the slot logged this step, if any."""
    if self._episode is not None:
      ep = self._episode
      ep.pressed = ep.pressed or s.steering_pressed
      ep.max_angle = max(ep.max_angle, abs(s.steering_angle))
      ep.lat_lost = ep.lat_lost or not s.lat_active
      if s.t - ep.t0 >= OUTCOME_S:
        self._emit('outcome', 0.0, side=ep.side, driver_took_wheel=ep.pressed, max_steering_deg=round(ep.max_angle, 1),
                   lat_lost=ep.lat_lost)
        self._episode = None
        return 'outcome'
      return None

    if s.steering_pressed:
      self._last_pressed = s.t
    side = turning_side(s) if s.t - self._last_pressed >= PRESS_QUIET_S else None
    if side is None:
      self._quiet_since = s.t if self._quiet_since is None else self._quiet_since
      if s.t - self._quiet_since >= QUIET_S:
        self._armed = True
      return None
    self._quiet_since = None
    if not self._armed or s.position is None or s.t - self._last_lookup < LOOKUP_PERIOD_S:
      return None
    self._last_lookup = s.t
    known, crossing = crossing_at(*s.position)
    if not known:
      self._emit('no_map', NO_MAP_MIN_GAP_S, side=side, curvature=round(s.model_curvature, 4),
                 latitude=round(s.position[0], 5), longitude=round(s.position[1], 5))
      return 'no_map'
    if not would_hold(side, crossing):
      return None
    self._armed = False
    self._episode = _Episode(s.t, side, s.steering_pressed, abs(s.steering_angle), not s.lat_active)
    self._emit('would_hold', 0.0, side=side, curvature=round(s.model_curvature, 4), speed_mph=round(s.speed * CV.MS_TO_MPH, 1),
               junction_m=crossing.distance_m, branch_deg=crossing.branch(side), road_bend_deg=crossing.bend_deg,
               road_continues_m=crossing.continues_m, lane_prob=round(s.lane_prob, 2), edge_std=round(s.edge_std, 2),
               steering_deg=round(s.steering_angle, 1), latitude=round(s.position[0], 5), longitude=round(s.position[1], 5))
    return 'would_hold'


def sample_from(sm, now_ns: int) -> Sample:
  """A Sample from a SubMaster with carState, carControl, modelV2 and the GPS services (starpilot.gps.source)."""
  from openpilot.starpilot.gps.source import bearing as gps_bearing, select_source
  cs, cc, md = sm['carState'], sm['carControl'], sm['modelV2']
  lanes = list(md.laneLineProbs)
  edges = list(md.roadEdgeStds)
  position = None
  fix = select_source(sm, now_ns)
  if fix is not None:
    gps = fix[1][1]
    heading = gps_bearing(gps)
    if heading is not None and gps.speed > 1.0:
      position = (float(gps.latitude), float(gps.longitude), heading)
  return Sample(t=now_ns / 1e9, speed=float(cs.vEgo), left_blinker=bool(cs.leftBlinker), right_blinker=bool(cs.rightBlinker),
                lat_active=bool(cc.latActive), steering_pressed=bool(cs.steeringPressed), steering_angle=float(cs.steeringAngleDeg),
                model_curvature=float(md.action.desiredCurvature),
                lane_prob=min(lanes[1], lanes[2]) if len(lanes) >= 3 else 0.0,
                edge_std=max(edges) if edges else 0.0, position=position)
