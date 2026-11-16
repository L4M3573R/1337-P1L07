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
