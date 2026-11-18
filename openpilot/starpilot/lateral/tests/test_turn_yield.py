"""Slow signalled turns: the comma keeps steering through a nudge and never pushes the wheel back against the turn."""
from openpilot.starpilot.lateral.turn_yield import no_push_back

LEFT = {'v_ego': 3.0, 'left_blinker': True, 'right_blinker': False, 'lat_active': True}
RIGHT = {'v_ego': 3.0, 'left_blinker': False, 'right_blinker': True, 'lat_active': True}


def test_in_a_slow_turn_the_comma_never_pushes_the_wheel_back():
  assert no_push_back(-1.0, steering_angle_deg=200.0, **LEFT) == 0.0      # 2026-11-17: full torque back against the left turn
  assert no_push_back(0.6, steering_angle_deg=200.0, **LEFT) == 0.6       # helping the turn is fine
  assert no_push_back(-1.0, steering_angle_deg=10.0, **LEFT) == -1.0      # wheel near straight, no push: no limit
  assert no_push_back(-1.0, steering_angle_deg=200.0, **{**LEFT, 'left_blinker': False}) == -1.0   # blinker off
  assert no_push_back(-1.0, steering_angle_deg=200.0, **{**LEFT, 'v_ego': 15.0}) == -1.0           # above the turn cutoff
  assert no_push_back(1.0, steering_angle_deg=-150.0, **RIGHT) == 0.0     # right turn: no push to the left


def test_a_nudge_toward_the_turn_keeps_the_comma_steering():
  # 2026-11-17 drive home, second stop sign: the nudge (+183 at 5 deg) used to drop the comma's torque to zero (turn yield);
  # now its torque toward the turn stays and adds to the driver's
  assert no_push_back(0.4, steering_angle_deg=5.0, steering_pressed=True, steering_torque=183.0, **LEFT) == 0.4
  assert no_push_back(1.0, steering_angle_deg=120.0, steering_pressed=True, steering_torque=288.0, **LEFT) == 1.0


def test_a_nudge_toward_the_turn_removes_push_back_even_near_straight():
  # the 2026-11-14/15 case turn yield was for: the driver starts the turn and the controller pushes back at full torque
  assert no_push_back(-1.0, steering_angle_deg=5.0, steering_pressed=True, steering_torque=183.0, **LEFT) == 0.0
  assert no_push_back(1.0, steering_angle_deg=-8.0, steering_pressed=True, steering_torque=-246.0, **RIGHT) == 0.0


def test_steering_against_the_blinker_is_not_a_nudge():
  assert no_push_back(-1.0, steering_angle_deg=5.0, steering_pressed=True, steering_torque=-200.0, **LEFT) == -1.0
  assert no_push_back(-1.0, steering_angle_deg=5.0, steering_pressed=False, steering_torque=200.0, **LEFT) == -1.0
