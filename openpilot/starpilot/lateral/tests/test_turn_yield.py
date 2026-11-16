"""Turn yield: no comma torque while the driver steers a turn toward the blinker below turn speed."""
from openpilot.common.constants import CV
from openpilot.starpilot.lateral.turn_yield import RELEASE_S, TurnYield

SLOW = 8 * CV.MPH_TO_MS


def step(y, t, **kw):
  args = {'v_ego': SLOW, 'left_blinker': False, 'right_blinker': True, 'steering_pressed': False, 'steering_torque': 0.0,
          'lat_active': True, **kw}
  return y.update(t, **args)


def test_yields_while_the_driver_steers_into_the_turn_then_takes_back_over():
  # 2026-11-14 14:58: right blinker, the driver steering right (-246) while the comma pushed full left
  y = TurnYield()
  assert not step(y, 0.0)
  assert step(y, 0.1, steering_pressed=True, steering_torque=-246)
  assert step(y, 0.5, steering_pressed=True, steering_torque=-120)
  assert step(y, 0.6)                              # let go: still yielding for a moment
  assert step(y, 0.6 + RELEASE_S - 0.01)
  assert not step(y, 0.6 + RELEASE_S + 0.01)       # back to the comma


def test_steering_against_the_blinker_never_yields():
  assert not step(TurnYield(), 0.0, steering_pressed=True, steering_torque=200)


def test_above_turn_speed_a_push_is_a_lane_change_nudge_not_a_yield():
  assert not step(TurnYield(), 0.0, v_ego=25 * CV.MPH_TO_MS, steering_pressed=True, steering_torque=-246)


def test_no_blinker_or_not_engaged_never_yields():
  assert not step(TurnYield(), 0.0, right_blinker=False, steering_pressed=True, steering_torque=-246)
  assert not step(TurnYield(), 0.0, left_blinker=True, steering_pressed=True, steering_torque=-246)   # hazards
  assert not step(TurnYield(), 0.0, lat_active=False, steering_pressed=True, steering_torque=-246)


def test_blinker_off_or_speeding_up_ends_it_at_once():
  y = TurnYield()
  assert step(y, 0.0, steering_pressed=True, steering_torque=-246)
  assert not step(y, 0.1, right_blinker=False, steering_pressed=True, steering_torque=-246)
  assert step(y, 0.2, steering_pressed=True, steering_torque=-246)
  assert not step(y, 0.3, v_ego=21 * CV.MPH_TO_MS, steering_pressed=True, steering_torque=-246)


def test_left_turns_use_positive_torque():
  y = TurnYield()
  assert step(y, 0.0, left_blinker=True, right_blinker=False, steering_pressed=True, steering_torque=300)
