"""The parking-lot EPS test switch: the >90 degree cut-out angle moves only with a valid test file."""
import json

from opendbc.car.hyundai.carcontroller import MAX_ANGLE, eps_test_max_angle


def test_no_file_keeps_the_stock_cut_out(tmp_path):
  assert eps_test_max_angle(str(tmp_path / 'missing.json')) == MAX_ANGLE


def test_a_test_file_moves_it(tmp_path):
  path = tmp_path / 'eps_test.json'
  path.write_text(json.dumps({'max_angle': 360}))
  assert eps_test_max_angle(str(path)) == 360.0


def test_bad_values_keep_the_stock_cut_out(tmp_path):
  path = tmp_path / 'eps_test.json'
  for doc in ({'max_angle': 30}, {'max_angle': 9000}, {'max_angle': 'off'}, [], {}):
    path.write_text(json.dumps(doc))
    assert eps_test_max_angle(str(path)) == MAX_ANGLE
  path.write_text('not json')
  assert eps_test_max_angle(str(path)) == MAX_ANGLE


class _Params:
  def __init__(self, steer_max=384, delta_up=3, delta_down=7):
    self.STEER_MAX, self.STEER_DELTA_UP, self.STEER_DELTA_DOWN = steer_max, delta_up, delta_down


def test_the_file_changes_torque_ramp_and_cut_out_live_within_the_firmware_ceiling(tmp_path):
  import os
  from opendbc.car.hyundai.carcontroller import EPS_TEST_RELOAD_FRAMES, EpsTestConfig, MAX_ANGLE_FRAMES
  path = tmp_path / 'eps_test.json'
  params = _Params()
  cfg = EpsTestConfig(params, str(path))
  cfg.step(0, params)
  assert (params.STEER_MAX, params.STEER_DELTA_UP, params.STEER_DELTA_DOWN) == (384, 3, 7)   # no file: stock
  assert (cfg.values['max_angle'], cfg.values['max_angle_frames']) == (MAX_ANGLE, MAX_ANGLE_FRAMES)
  path.write_text(json.dumps({'steer_max': 640, 'delta_up': 10, 'delta_down': 12, 'max_angle': 120, 'max_angle_frames': 95}))
  os.utime(path, ns=(1, 1))
  cfg.step(1, params)
  assert params.STEER_MAX == 384   # only re-read on the reload frame
  cfg.step(EPS_TEST_RELOAD_FRAMES, params)
  assert (params.STEER_MAX, params.STEER_DELTA_UP, params.STEER_DELTA_DOWN) == (640, 10, 12)
  assert (cfg.values['max_angle'], cfg.values['max_angle_frames']) == (120.0, 95)
  path.write_text(json.dumps({'steer_max': 900, 'delta_up': 20, 'delta_down': True, 'max_angle_frames': 50}))
  os.utime(path, ns=(2, 2))
  cfg.step(2 * EPS_TEST_RELOAD_FRAMES, params)
  assert (params.STEER_MAX, params.STEER_DELTA_UP, params.STEER_DELTA_DOWN) == (384, 3, 7)   # over the ceiling: stock
  assert cfg.values['max_angle_frames'] == MAX_ANGLE_FRAMES   # never below the panda's 89 valid frames
  path.unlink()
  cfg.step(3 * EPS_TEST_RELOAD_FRAMES, params)
  assert params.STEER_MAX == 384


def test_cars_on_other_limit_sets_are_never_raised(tmp_path):
  from opendbc.car.hyundai.carcontroller import EpsTestConfig
  path = tmp_path / 'eps_test.json'
  path.write_text(json.dumps({'steer_max': 768, 'delta_up': 12}))
  params = _Params(270, 2, 3)   # ALT limits
  EpsTestConfig(params, str(path)).step(0, params)
  assert (params.STEER_MAX, params.STEER_DELTA_UP, params.STEER_DELTA_DOWN) == (270, 2, 3)


def test_the_cut_out_length_can_drop_to_one_frame_but_never_exceed_the_panda_tolerance(tmp_path):
  from opendbc.car.hyundai.carcontroller import EpsTestConfig, MAX_ANGLE_CONSECUTIVE_FRAMES
  path = tmp_path / 'eps_test.json'
  params = _Params()
  for value, expected in ((1, 1), (2, 2), (0, MAX_ANGLE_CONSECUTIVE_FRAMES), (3, MAX_ANGLE_CONSECUTIVE_FRAMES)):
    path.write_text(json.dumps({'cut_frames': value}))
    assert EpsTestConfig(params, str(path)).values['cut_frames'] == expected


def test_a_one_frame_cut_drops_the_request_for_exactly_one_frame():
  from opendbc.car.lateral import common_fault_avoidance
  counter, requests = 0, []
  for _ in range(200):
    counter, req = common_fault_avoidance(True, True, counter, 89, 1)
    requests.append(req)
  drops = [i for i, r in enumerate(requests) if not r]
  assert drops == [89, 179]   # one frame each, 90 frames apart


def _panda_ok(requests, torques):
  """safety/lateral.h steer-request tolerance: a cut (request low with torque) needs >= 89 valid frames before it,
  >= 0.81 s since the last cut frame, and at most 2 cut frames in a row."""
  valid, invalid, last_cut = 0, 0, -10_000
  for frame, (req, torque) in enumerate(zip(requests, torques, strict=True)):
    if not req and torque != 0:
      if invalid == 0 and (valid < 89 or frame - last_cut < 81):
        return False, frame
      if invalid >= 2:
        return False, frame
      valid, invalid, last_cut = 0, invalid + 1, frame
    else:
      valid, invalid = min(valid + 1, 89), 0
  return True, None


def _config(tmp_path, values):
  from opendbc.car.hyundai.carcontroller import EpsTestConfig
  path = tmp_path / 'eps_test.json'
  path.write_text(json.dumps(values))
  return EpsTestConfig(_Params(), str(path))


def test_default_cut_out_matches_stock_common_fault_avoidance(tmp_path):
  from opendbc.car.lateral import common_fault_avoidance
  cfg = _config(tmp_path, {})
  counter, ours, stock = 0, [], []
  pattern = [False] * 50 + [True] * 400 + [False] * 30 + [True] * 300
  for above in pattern:
    counter, req = common_fault_avoidance(above, True, counter, 89, 2)
    stock.append(req)
    ours.append(cfg.cut_out(above, True, 300))
  assert ours == stock


def test_every_sag_shaping_mode_stays_inside_the_panda_rules(tmp_path):
  import random
  rng = random.Random(7)
  for values in ({}, {'cut_frames': 1}, {'max_angle_frames': 150}, {'cut_early_pct': 100}, {'cut_early_pct': 60, 'cut_frames': 1},
                 {'cut_boost': 200, 'cut_boost_frames': 30, 'cut_preload_frames': 20, 'cut_early_pct': 80}):
    cfg = _config(tmp_path, values)
    reqs, torques = [], []
    above, active = False, True
    for _ in range(20_000):
      if rng.random() < 0.02:
        above = not above
      if rng.random() < 0.002:
        active = not active
      torque = rng.randint(-384, 384) if active else 0
      reqs.append(cfg.cut_out(above, active, torque))
      torques.append(torque)
    ok, frame = _panda_ok(reqs, torques)
    assert ok, (values, frame)
    assert reqs.count(False) > 0


def test_an_early_cut_lands_on_low_torque_instead_of_waiting_for_the_deadline(tmp_path):
  cfg = _config(tmp_path, {'cut_early_pct': 40})
  for _ in range(100):
    cfg.cut_out(False, True, 380)               # straight: 100 valid frames
  reqs = [cfg.cut_out(True, True, 380 if i < 30 else 100) for i in range(60)]   # past the angle, demand drops at 30
  assert reqs.index(False) == 30                # cut as soon as the demand is low, not at frame 90


def test_boost_stays_within_steer_max_unless_cut_boost_max_allows_more(tmp_path):
  capped = _config(tmp_path, {'steer_max': 640, 'cut_boost': 200, 'cut_boost_frames': 10})
  params = _Params()
  capped.step(0, params)
  capped.boost_left = 5
  assert capped.boost(300, params) == 500 and capped.boost(600, params) == 640 and params.STEER_MAX == 640


def test_boost_adds_torque_after_the_cut_and_may_pass_steer_max_up_to_the_ceiling(tmp_path):
  cfg = _config(tmp_path, {'steer_max': 640, 'cut_boost': 200, 'cut_boost_max': 768, 'cut_boost_frames': 10})
  params = _Params()
  cfg.step(0, params)
  assert cfg.boost(500, params) == 500          # no cut yet: no boost
  for _ in range(100):
    cfg.cut_out(False, True, 500)
  for _ in range(92):
    cfg.cut_out(True, True, 500)                # the scheduled cut happens in here
  assert cfg.boost_left > 0
  cfg.step(1, params)
  assert cfg.boost(600, params) == 768 and params.STEER_MAX == 768   # 600 + 200 capped at the firmware ceiling
  assert cfg.boost(-300, params) == -500
  cfg.boost_left = 0
  cfg.step(2, params)
  assert cfg.boost(600, params) == 600 and params.STEER_MAX == 640
