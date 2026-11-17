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
