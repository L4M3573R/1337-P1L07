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
  def __init__(self, steer_max=384, delta_up=3):
    self.STEER_MAX, self.STEER_DELTA_UP = steer_max, delta_up


def test_torque_and_ramp_rise_only_with_the_file_and_only_to_the_firmware_ceiling(tmp_path):
  from opendbc.car.hyundai.carcontroller import eps_test_limits
  path = tmp_path / 'eps_test.json'
  params = _Params()
  eps_test_limits(params, str(tmp_path / 'missing.json'))
  assert (params.STEER_MAX, params.STEER_DELTA_UP) == (384, 3)   # normal drives
  path.write_text(json.dumps({'steer_max': 404, 'delta_up': 4}))
  eps_test_limits(params, str(path))
  assert (params.STEER_MAX, params.STEER_DELTA_UP) == (404, 4)
  for bad in ({'steer_max': 500, 'delta_up': 9}, {'steer_max': 300, 'delta_up': 1}, {'steer_max': 400.5, 'delta_up': True}):
    params = _Params()
    path.write_text(json.dumps(bad))
    eps_test_limits(params, str(path))
    assert (params.STEER_MAX, params.STEER_DELTA_UP) == (384, 3)


def test_cars_on_other_limit_sets_are_never_raised(tmp_path):
  from opendbc.car.hyundai.carcontroller import eps_test_limits
  path = tmp_path / 'eps_test.json'
  path.write_text(json.dumps({'steer_max': 409, 'delta_up': 5}))
  params = _Params(270, 2)   # ALT limits
  eps_test_limits(params, str(path))
  assert (params.STEER_MAX, params.STEER_DELTA_UP) == (270, 2)
