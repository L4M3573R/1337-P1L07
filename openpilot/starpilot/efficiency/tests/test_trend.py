"""Per-drive MPG trend for the comma-vs-you graph."""
import json
import os

import pytest

from openpilot.starpilot.efficiency import trend

MILE = 1609.344


def doc(you=None, comma=None):
  """you / comma: (miles, gallons, soc points) for the drive."""
  contexts = {}
  if you:
    contexts['30-45 mph|flat|straight|open road|driver'] = {'obd_distance': you[0] * MILE, 'obd_fuel_gal': you[1], 'obd_soc_delta': you[2]}
  if comma:
    contexts['30-45 mph|flat|straight|open road|pulse-glide'] = {'obd_distance': comma[0] * MILE, 'obd_fuel_gal': comma[1],
                                                                'obd_soc_delta': comma[2]}
  return {'version': 1, 'drive': 'x', 'total': {}, 'contexts': contexts}


def test_drive_point_per_side_battery_adjusted():
  p = trend.drive_point(0.0, doc(you=(10, 0.25, 0.0), comma=(20, 0.40, 0.0)))
  assert p.you_mpg == pytest.approx(40.0) and p.comma_mpg == pytest.approx(50.0)
  drained = trend.drive_point(0.0, doc(comma=(20, 0.40, -10.0)))
  assert drained.you_mpg is None and drained.comma_mpg < 50.0


def test_short_or_unmeasured_drives_have_no_point():
  assert trend.drive_point(0.0, doc(you=(0.5, 0.01, 0.0))) is None
  assert trend.drive_point(0.0, doc()) is None


def test_load_points_sorted_by_time_and_skips_bad_files(tmp_path):
  drives = tmp_path / 'drives'
  drives.mkdir()
  for name, t, d in (('b', 200, doc(you=(10, 0.2, 0))), ('a', 100, doc(comma=(10, 0.25, 0)))):
    path = drives / f'{name}.json'
    path.write_text(json.dumps(d))
    os.utime(path, (t, t))
  (drives / 'bad.json').write_text('{nope')
  points = trend.load_points(tmp_path)
  assert [p.t for p in points] == [100, 200]
  assert points[0].comma_mpg == pytest.approx(40.0) and points[1].you_mpg == pytest.approx(50.0)


def test_rolling_average():
  assert trend.rolling([None, 40.0, None, 50.0], n=2) == [None, 40.0, 40.0, 45.0]


def test_pc_summary_from_a_local_copy(tmp_path):
  from openpilot.starpilot.efficiency.pc_summary import summarize
  drives = tmp_path / 'drives'
  drives.mkdir()
  d = doc(you=(10, 0.25, 0.0), comma=(10, 0.20, 0.0))
  for i in range(3):
    path = drives / f'{i}.json'
    path.write_text(json.dumps(d))
    os.utime(path, (100 + i, 100 + i))
  out = summarize(tmp_path, None)
  assert len(out['drives']) == 3 and out['you_avg'][-1] == pytest.approx(40.0) and out['comma_avg'][-1] == pytest.approx(50.0)
  assert out['comparison']['pulse-glide']['matched_miles'] == pytest.approx(30.0)
  assert out['comparison']['all']['comma_saving'] == pytest.approx(0.2)
  assert out['comparison']['chill']['matched_miles'] == 0
