"""Engine temperature filled in for older drives from the BlueDriver logs, and cold-engine stretches by coolant."""
import csv
import json

from openpilot.starpilot.efficiency import coolant_backfill as cb
from openpilot.starpilot.efficiency.store import COLD_START_S, cold_stretches


def stretch(t, dur=30.0, **kw):
  return {'t': t, 'dur': dur, 'm': 500.0, 'ctx': ['flat', 'straight', '45-60', 'free'], 'mode': 'chill', **kw}


def write_drive(root, name, stretches):
  (root / 'drives').mkdir(parents=True, exist_ok=True)
  path = root / 'drives' / f'{name}.json'
  path.write_text(json.dumps({'version': 1, 'drive': name, 'stretches': stretches}))
  return path


def write_log(root, name, rows):
  (root / 'bluedriver').mkdir(parents=True, exist_ok=True)
  with open(root / 'bluedriver' / f'{name}.csv', 'w', newline='') as f:
    w = csv.DictWriter(f, ('t', 'rpm', 'coolant'))
    w.writeheader()
    for t, c in rows:
      w.writerow({'t': t, 'rpm': 1200, 'coolant': '' if c is None else c})


def test_a_drive_gets_the_coldest_coolant_logged_in_each_stretch(tmp_path):
  path = write_drive(tmp_path, '20261115-112556-1910', [stretch(110.0), stretch(1000.0), stretch(2000.0)])
  write_log(tmp_path, '20261115-112401-1913', [(t, 12.0 + t * 0.02) for t in range(30, 2300)])
  assert cb.backfill(tmp_path / 'drives', tmp_path / 'bluedriver') == 1
  doc = json.loads(path.read_text())
  assert [s['coolant'] for s in doc['stretches']] == [14, 32, 52]
  assert doc['coolant_from'] == 'bluedriver log'
  # and a second run leaves it alone
  assert cb.backfill(tmp_path / 'drives', tmp_path / 'bluedriver') == 0


def test_a_log_from_another_boot_or_time_is_not_used(tmp_path):
  write_drive(tmp_path, '20261115-112556-1910', [stretch(110.0)])
  write_log(tmp_path, '20261115-150000-1913', [(t, 20.0) for t in range(30, 300)])    # hours later
  write_log(tmp_path, '20261115-112401-1999', [(t, 20.0) for t in range(5000, 6000)])  # times never overlap
  assert cb.backfill(tmp_path / 'drives', tmp_path / 'bluedriver') == 0


def test_a_log_that_fits_two_drives_is_skipped(tmp_path):
  # The clock was not set yet: every boot names its files the same minute, and the times overlap.
  a = write_drive(tmp_path, '20261012-033633-1856', [stretch(100.0)])
  b = write_drive(tmp_path, '20261012-033634-1855', [stretch(120.0)])
  write_log(tmp_path, '20261012-033638-1858', [(t, 30.0) for t in range(30, 400)])
  assert cb.backfill(tmp_path / 'drives', tmp_path / 'bluedriver') == 0
  assert 'coolant' not in json.loads(a.read_text())['stretches'][0]
  assert 'coolant' not in json.loads(b.read_text())['stretches'][0]


def test_the_drive_that_started_nearest_gets_the_log(tmp_path):
  # The comma restarted for every drive: both drives' clocks start near zero and overlap the log's times.
  a = write_drive(tmp_path, '20261114-142921-8156', [stretch(100.0)])
  b = write_drive(tmp_path, '20261114-144745-1866', [stretch(100.0)])
  write_log(tmp_path, '20261114-144650-1867', [(t, 43.0) for t in range(40, 1200)])
  assert cb.backfill(tmp_path / 'drives', tmp_path / 'bluedriver') == 1
  assert 'coolant' not in json.loads(a.read_text())['stretches'][0]
  assert json.loads(b.read_text())['stretches'][0]['coolant'] == 43


def test_drives_that_already_have_coolant_are_left_alone(tmp_path):
  path = write_drive(tmp_path, '20261125-054539-2471', [stretch(100.0, coolant=88)])
  write_log(tmp_path, '20261125-054500-2470', [(t, 10.0) for t in range(30, 400)])
  assert cb.backfill(tmp_path / 'drives', tmp_path / 'bluedriver') == 0
  assert json.loads(path.read_text())['stretches'][0]['coolant'] == 88


def test_rows_without_a_coolant_reading_are_ignored(tmp_path):
  write_log(tmp_path, '20261115-112401-1913', [(40.0, None), (41.0, 18.0), (42.0, 999.0)])
  assert cb.read_log(tmp_path / 'bluedriver' / '20261115-112401-1913.csv') == [(41.0, 18.0)]


def test_cold_is_the_warm_up_only():
  # Cold until the first warm reading (a gap before it too); cooling back down in EV later is not a cold start.
  s = [stretch(20.0), stretch(60.0, coolant=40), stretch(400.0, coolant=70), stretch(500.0), stretch(900.0, coolant=45)]
  assert [x['t'] for x in cold_stretches({'stretches': s})] == [20.0, 60.0]


def test_a_drive_that_never_warms_is_warm_up_to_its_last_reading():
  late = 20.0 + COLD_START_S + 300
  s = [stretch(20.0, coolant=30), stretch(late, coolant=48), stretch(late + 100), stretch(late + 200)]
  assert [x['t'] for x in cold_stretches({'stretches': s})] == [20.0, late]


def test_cold_by_time_when_the_drive_has_no_coolant():
  s = [stretch(20.0), stretch(20.0 + COLD_START_S - 1), stretch(20.0 + COLD_START_S + 1)]
  assert [x['t'] for x in cold_stretches({'stretches': s})] == [20.0, 20.0 + COLD_START_S - 1]
