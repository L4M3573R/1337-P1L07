"""Turn yield: below turn speed, with a blinker on, the comma's steering lets go while the driver steers toward the blinker.

In 2026-11-14/15 turns the driver started steering into the turn and the controller, still holding the planned path
straight, pushed back at full torque against them (7 of 37 comma-steered turns; e.g. -246 driver torque right against a
+1.0 command left with the wheel at -362 degrees). Below CUTOFF_MPS a blinker means a turn, so the driver's steering that
way is the turn: the lateral controller runs inactive (no torque, its state reset) while the driver steers, and takes
back over RELEASE_S after they let go. The car stays engaged (latActive), so there is no alert and no re-engage.
Steering the other way, or above the cutoff (a lane-change nudge), never yields.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from openpilot.starpilot.navigation.blinker_turns import CUTOFF_MPS

RELEASE_S = 0.3
# During a slow signalled turn with the wheel turned toward the blinker, the comma may add turn but never push the wheel
# back: on 2026-11-17 it came back at full torque the other way each time the driver eased off (the driver had turned
# tighter than the model's wide plan), three times in one turn. Lifted once the wheel is back under this angle, the
# blinker goes off or the car is above the turn cutoff; the wheel's own return and the driver straighten it.
HOLD_ANGLE_DEG = 20.0


def no_push_back(torque: float, *, v_ego: float, left_blinker: bool, right_blinker: bool, steering_angle_deg: float,
                 lat_active: bool) -> float:
  """Torque (positive left) with any part against a slow signalled turn removed. Angle positive left."""
  side = 1 if left_blinker and not right_blinker else -1 if right_blinker and not left_blinker else 0
  if (not lat_active or side == 0 or not v_ego < CUTOFF_MPS or not math.isfinite(steering_angle_deg) or
      steering_angle_deg * side < HOLD_ANGLE_DEG):
    return torque
  return torque if torque * side >= 0.0 else 0.0


@dataclass
class TurnYield:
  yielding: bool = False
  released_at: float | None = None

  def update(self, t: float, *, v_ego: float, left_blinker: bool, right_blinker: bool, steering_pressed: bool,
             steering_torque: float, lat_active: bool) -> bool:
    """True while the comma should put no torque on the wheel. Driver torque is positive to the left."""
    side = 1 if left_blinker and not right_blinker else -1 if right_blinker and not left_blinker else 0
    if not lat_active or side == 0 or not v_ego < CUTOFF_MPS:
      self.yielding, self.released_at = False, None
      return False
    if steering_pressed and steering_torque * side > 0:
      self.yielding, self.released_at = True, None
    elif self.yielding and not steering_pressed:
      self.released_at = t if self.released_at is None else self.released_at
      if t - self.released_at >= RELEASE_S:
        self.yielding, self.released_at = False, None
    return self.yielding
