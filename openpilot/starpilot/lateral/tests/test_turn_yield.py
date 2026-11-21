"""Slow signalled turns: the comma keeps steering through a nudge and never pushes the wheel back against the turn."""
from openpilot.starpilot.lateral.turn_yield import DRIVER_YIELD_TORQUE, driver_yield, no_push_back

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


def test_the_comma_stops_fighting_a_driver_who_has_the_wheel():
  # 2026-11-20 12:36:32 parking lot: wheel at full left lock, driver torque +1022, comma -1.00 -> EPS fault
  assert driver_yield(-1.0, steering_pressed=True, steering_torque=1022.0, lat_active=True) == 0.0
  assert driver_yield(1.0, steering_pressed=True, steering_torque=-400.0, lat_active=True) == 0.0   # same to the right


def test_driver_yield_keeps_help_in_the_drivers_direction_so_letting_go_never_unwinds():
  assert driver_yield(0.7, steering_pressed=True, steering_torque=600.0, lat_active=True) == 0.7
  assert driver_yield(-0.3, steering_pressed=True, steering_torque=-300.0, lat_active=True) == -0.3


def test_driver_yield_ignores_hands_resting_on_the_wheel_and_inactive_steering():
  just_under = DRIVER_YIELD_TORQUE - 1.0
  assert driver_yield(-1.0, steering_pressed=True, steering_torque=just_under, lat_active=True) == -1.0  # resting hands
  assert driver_yield(-1.0, steering_pressed=False, steering_torque=900.0, lat_active=True) == -1.0     # not pressed
  assert driver_yield(-1.0, steering_pressed=True, steering_torque=900.0, lat_active=False) == -1.0     # comma not steering
  assert driver_yield(-1.0, steering_pressed=True, steering_torque=float('nan'), lat_active=True) == -1.0
