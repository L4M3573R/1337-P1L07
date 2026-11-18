"""Slow signalled turns: the comma never pushes the wheel back against the turn.

Turn yield (2026-11-16) let go of the wheel whenever the driver steered toward the blinker below turn speed, to stop the
controller pushing back against them (2026-11-14/15). But a nudge to help a turn then switched the comma off for the
rest of it, and when the driver let go to hand the turn back it restarted from zero torque and the wheel unwound
(2026-11-17, second stop sign on the drive home: nudged at 329.5 s, let go at 120 deg, wheel back to 76 deg in 0.5 s,
the driver steered the rest by hand). Now the comma keeps steering through a nudge and only the push-back is removed:
its torque toward the turn adds to the driver's, its torque against the turn is dropped.
"""
from __future__ import annotations

import math

from openpilot.starpilot.navigation.blinker_turns import CUTOFF_MPS

# During a slow signalled turn with the wheel turned toward the blinker (or the driver pushing toward it), the comma may
# add turn but never push the wheel back: on 2026-11-17 it came back at full torque the other way each time the driver
# eased off (the driver had turned tighter than the model's wide plan), three times in one turn. Lifted once the wheel
# is back under this angle with no push toward the turn, the blinker goes off or the car is above the turn cutoff.
HOLD_ANGLE_DEG = 20.0


def no_push_back(torque: float, *, v_ego: float, left_blinker: bool, right_blinker: bool, steering_angle_deg: float,
                 lat_active: bool, steering_pressed: bool = False, steering_torque: float = 0.0) -> float:
  """The command with any torque against a slow signalled turn removed. Torque and angle are positive to the left."""
  side = 1 if left_blinker and not right_blinker else -1 if right_blinker and not left_blinker else 0
  pushing_toward = steering_pressed and steering_torque * side > 0
  if (not lat_active or side == 0 or not v_ego < CUTOFF_MPS or not math.isfinite(steering_angle_deg) or
      (steering_angle_deg * side < HOLD_ANGLE_DEG and not pushing_toward)):
    return torque
  return torque if torque * side >= 0.0 else 0.0
