"""Turn speed for signalled turns: learned per side and sharpness, eased into, only ever a lower target."""
import json
import math
from types import SimpleNamespace as NS

import pytest

from openpilot.starpilot.navigation import turn_speed as ts
from openpilot.starpilot.navigation.turn_speed import TurnLearner, TurnSpeedHost, TurnSpeeds

MPH = ts.MPH


def test_sharpness_from_the_side_road_angle():
  assert ts.sharpness(45.0) == 'shallow' and ts.sharpness(-90.0) == 'square' and ts.sharpness(135.0) == 'sharp'
  assert ts.sharpness(None) == 'unknown'


def test_defaults_then_learned_speed():
  speeds = TurnSpeeds()
  assert speeds.driver_speed('right', 'square') == pytest.approx(11 * MPH)
  assert speeds.driver_speed('left', 'square') > speeds.driver_speed('right', 'square')
  speeds.learn('right', 'square', 9 * MPH)
  assert speeds.driver_speed('right', 'square') == pytest.approx(9 * MPH, abs=0.01)
  speeds.learn('right', 'square', 13 * MPH)
  assert 9 * MPH < speeds.driver_speed('right', 'square') < 13 * MPH
  # learned only without a map angle: scaled for the sharpness instead of the default
  speeds.learn('left', 'unknown', 12 * MPH)
  assert speeds.driver_speed('left', 'sharp') < speeds.driver_speed('left', 'unknown') < speeds.driver_speed('left', 'shallow')


def test_turn_speed_never_beyond_the_steering():
  speeds = TurnSpeeds()
  speeds.learn('right', 'square', 11 * MPH)
  # a 9 m radius right turn: the comma holds about 1.16 m/s2 there, so about 7 mph even though the driver takes it at 11
  assert speeds.speed('right', 'square') == pytest.approx(ts.steer_limit_speed(1 / 9), abs=0.01)
  assert 6.5 * MPH < speeds.speed('right', 'square') < 7.8 * MPH
  # a gentle, wide turn: the driver's speed is within the steering
  speeds.learn('left', 'shallow', 12 * MPH)
  assert speeds.speed('left', 'shallow') == pytest.approx(12 * MPH, abs=0.01)
  # a measured tight turn lowers the cap
  speeds.learn_curvature('right', 'square', 1 / 6)
  assert speeds.speed('right', 'square') < ts.steer_limit_speed(1 / 9)


def test_ceiling_eases_to_the_turn_speed_where_the_turn_begins():
  speeds = TurnSpeeds()
  v_turn = speeds.speed('right', 'square')   # the steering-limited default
  far, _ = ts.ceiling({'side': 'right', 'turning': False, 'distance_m': 140.0, 'angle_deg': -90.0}, speeds)
  near, _ = ts.ceiling({'side': 'right', 'turning': False, 'distance_m': 30.0, 'angle_deg': -90.0}, speeds)
  at, _ = ts.ceiling({'side': 'right', 'turning': False, 'distance_m': ts.trigger_distance(v_turn), 'angle_deg': -90.0}, speeds)
  turning, _ = ts.ceiling({'side': 'right', 'turning': True, 'distance_m': None, 'angle_deg': -90.0}, speeds)
  assert far > near > at == pytest.approx(v_turn) and turning == pytest.approx(v_turn)
  assert ts.ceiling({'side': 'right', 'turning': False, 'distance_m': 500.0}, speeds)[0] is None   # too far yet
  assert ts.ceiling({'side': 'right', 'turning': False, 'distance_m': None}, speeds)[0] is None    # no road known
  assert ts.ceiling(None, speeds)[0] is None


def drive_turn(learner, *, v_apex, long_active, brake=False, gas=False, side='right', turn_deg=90.0, angle=-90.0):
  """A 6 s signalled turn: approach, turn through turn_deg at v_apex, then straighten."""
  t, key = 0.0, None
  rate = math.radians(turn_deg) / 3.0 * (-1 if side == 'right' else 1)
  for i in range(120):
    t = i * 0.05
    turning = 1.5 <= t < 4.5
    v = v_apex if turning else v_apex + 2.0
    out = learner.observe(t, side, v, rate if turning else 0.0, long_active, gas and turning, brake and turning, angle)
    key = out or key
  return key


def test_learns_the_drivers_own_turn_speed():
  speeds = TurnSpeeds()
  learner = TurnLearner(speeds)
  assert drive_turn(learner, v_apex=8 * MPH, long_active=False) == 'right|square'
  assert speeds.driver_speed('right', 'square') == pytest.approx(8 * MPH, abs=0.05)
  # the turn's tightness was measured too: 90 degrees in 3 s at 8 mph is about a 6.8 m radius
  assert speeds.curvature('right', 'square') == pytest.approx(math.radians(90) / 3 / (8 * MPH), rel=0.1)
  assert learner.learned == 1   # one sample per turn


def test_comma_turns_learn_from_the_pedals():
  speeds = TurnSpeeds({'left|square': [14 * MPH, 3]})
  drive_turn(TurnLearner(speeds), v_apex=12 * MPH, long_active=True, brake=True, side='left', angle=90.0)
  assert speeds.driver_speed('left', 'square') < 14 * MPH   # braked: slower
  before = speeds.driver_speed('left', 'square')
  drive_turn(TurnLearner(speeds), v_apex=before, long_active=True, side='left', angle=90.0)
  assert speeds.driver_speed('left', 'square') == pytest.approx(before, abs=0.2)   # no pedal: about the same


def test_lane_change_or_cancelled_turn_is_not_learned():
  speeds = TurnSpeeds()
  learner = TurnLearner(speeds)
  assert drive_turn(learner, v_apex=8 * MPH, long_active=False, turn_deg=20.0) is None
  assert speeds.table == {}


def test_save_load(tmp_path):
  speeds = TurnSpeeds()
  speeds.learn('right', 'sharp', 7 * MPH)
  speeds.save(tmp_path / 'turn_speed.json')
  assert TurnSpeeds.load(tmp_path / 'turn_speed.json').driver_speed('right', 'sharp') == pytest.approx(7 * MPH, abs=0.01)
  (tmp_path / 'bad.json').write_text('nope')
  assert TurnSpeeds.load(tmp_path / 'bad.json').table == {}


class Clock:
  def __init__(self):
    self.t = 100.0

  def __call__(self):
    return self.t


def sm(v=8.0, right=True, long_active=True, yaw=0.0):
  return {'carState': NS(vEgo=v, yawRate=yaw, leftBlinker=False, rightBlinker=right, gasPressed=False, brakePressed=False),
          'carControl': NS(longActive=long_active, enabled=long_active)}


def test_host_caps_only_with_fresh_turn_state_and_long_control(tmp_path):
  clock = Clock()
  host = TurnSpeedHost(path=tmp_path / 'turn_speed.json', status_path=tmp_path / 'status.json', clock=clock)
  assert host.sample(sm()) is None                                   # no turn state from modeld
  ts.write_status({'t': clock.t, 'side': 'right', 'turning': True, 'distance_m': None, 'angle_deg': None}, tmp_path / 'status.json')
  assert host.sample(sm()) == pytest.approx(TurnSpeeds().speed('right', 'unknown'))
  assert host.sample(sm(right=False)) is None                        # blinker off: the state is not for this
  assert host.sample(sm(long_active=False)) is None                  # the driver controls speed: no cap
  clock.t += 1.0
  assert host.sample(sm()) is None                                   # stale
  assert host.sample({}) is None                                     # broken input never raises


def test_host_saves_what_it_learns(tmp_path):
  clock = Clock()
  host = TurnSpeedHost(path=tmp_path / 'turn_speed.json', status_path=tmp_path / 'status.json', clock=clock)
  rate = -math.radians(90.0) / 3.0
  for i in range(120):
    clock.t = 100.0 + i * 0.05
    turning = 1.5 <= i * 0.05 < 4.5
    host.sample(sm(v=7 * MPH, long_active=False, yaw=rate if turning else 0.0))
  saved = json.loads((tmp_path / 'turn_speed.json').read_text())['speeds']
  assert saved['right|unknown'][0] == pytest.approx(7 * MPH, abs=0.05)


def test_a_turn_from_a_stop_teaches_no_speed():
  # 2026-11-17: pull-aways from stop signs had dragged left square turns to 8.9 mph and right unknown to 6.3 mph
  speeds = TurnSpeeds({'right|square': [11 * MPH, 5]})
  learner = TurnLearner(speeds)
  rate = -math.radians(90.0) / 3.0
  for i in range(120):
    t = i * 0.05
    turning = 1.5 <= t < 4.5
    v = 0.0 if t < 1.5 else min(6.0, 1.5 * (t - 1.5))   # waits at the line, then pulls away through the turn
    assert learner.observe(t, 'right', v, rate if turning else 0.0, False, False, False, 90.0) is None
  assert speeds.driver_speed('right', 'square') == pytest.approx(11 * MPH)
  assert learner.learned == 0


def test_a_comma_turn_with_no_pedal_never_lowers_the_speed():
  speeds = TurnSpeeds({'right|square': [11 * MPH, 5]})
  for _ in range(10):   # ten turns behind a slow car
    drive_turn(TurnLearner(speeds), v_apex=6.7 * MPH, long_active=True)
  assert speeds.driver_speed('right', 'square') == pytest.approx(11 * MPH)


def test_an_old_table_is_reset_to_the_seed_and_kept_beside(tmp_path):
  path = tmp_path / 'turn_speed.json'
  path.write_text(json.dumps({'version': 1, 'speeds': {'right|unknown': [2.815, 33], 'left|square': [3.989, 34]},
                              'curvatures': {'right|unknown': [0.0988, 33]}}))
  host = TurnSpeedHost(path=path, status_path=tmp_path / 'status.json', clock=Clock())
  host.sample(sm())
  assert host.speeds.driver_speed('right', 'unknown') == pytest.approx(10.7 * MPH, abs=0.01)
  assert host.speeds.curvature('right', 'unknown') == pytest.approx(0.0988)       # the measured tightness stays
  assert json.loads(path.read_text())['version'] == ts.TABLE_VERSION
  assert json.loads((tmp_path / 'turn_speed.v1.json').read_text())['speeds']['left|square'] == [3.989, 34]
  assert not TurnSpeeds.load(path).migrated


def test_the_ceiling_ends_once_the_car_has_turned(tmp_path):
  # a turn started from a stop has no GPS heading in modeld, so its 'turning' stayed on until the blinker went off
  clock = Clock()
  status = tmp_path / 'status.json'
  host = TurnSpeedHost(path=tmp_path / 'turn_speed.json', status_path=status, clock=clock)
  rate = -math.radians(90.0) / 3.0
  caps = []
  for i in range(160):
    clock.t = 100.0 + i * 0.05
    ts.write_status({'t': clock.t, 'side': 'right', 'turning': True, 'distance_m': None, 'angle_deg': 90.0}, status)
    caps.append(host.sample(sm(v=4.0, yaw=rate if i * 0.05 < 3.0 else 0.0)))
  assert caps[10] is not None and caps[40] is not None     # capped through the first 70 degrees
  assert caps[55] is None and caps[-1] is None              # 70 of the 90 degrees turned: released, and stays so
  host.sample(sm(right=False))                              # blinker off
  ts.write_status({'t': clock.t, 'side': 'right', 'turning': True, 'distance_m': None, 'angle_deg': 90.0}, status)
  assert host.sample(sm(v=4.0)) is not None                 # the next blinker gets its ceiling again
  assert ts.release_deg(45.0) == 30.0 and ts.release_deg(None) == ts.TURN_DONE_DEG
