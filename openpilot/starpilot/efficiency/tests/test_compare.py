"""Comma versus driver fuel: matched contexts only, battery-adjusted, weighted by the driver's miles."""
import pytest

from openpilot.starpilot.efficiency.compare import MIN_MATCHED_MILES, compare, headline, stored_comparison
from openpilot.starpilot.efficiency.ledger import Totals
from openpilot.starpilot.efficiency.store import save_drive

MILE = 1609.344


def t(miles, gal, soc=0.0):
  return Totals(distance=miles * MILE, obd_distance=miles * MILE, obd_fuel_gal=gal, obd_soc_delta=soc)


def test_comma_rate_applied_to_the_drivers_miles():
  ctx = {
    ('45-60 mph', 'flat', 'straight', 'open road', 'driver'): t(10, 0.25),  # 40 mpg
    ('45-60 mph', 'flat', 'straight', 'open road', 'chill'): t(20, 0.40),  # 50 mpg
    ('15-30 mph', 'flat', 'straight', 'stop-and-go', 'driver'): t(4, 0.10),  # 40 mpg
    ('15-30 mph', 'flat', 'straight', 'stop-and-go', 'experimental'): t(1, 0.04),  # 25 mpg
  }
  c = compare(ctx)
  assert c.matched_miles == pytest.approx(14) and c.contexts == 2
  assert c.driver_gal == pytest.approx(0.35)
  assert c.comma_gal == pytest.approx(10 * 0.02 + 4 * 0.04)
  assert c.comma_saving == pytest.approx(1 - 0.36 / 0.35)


def test_contexts_only_one_side_drove_are_ignored():
  ctx = {
    ('60+ mph', 'flat', 'straight', 'open road', 'chill'): t(50, 1.0),  # comma highway only
    ('15-30 mph', 'flat', 'straight', 'stop-and-go', 'driver'): t(10, 0.5),  # driver city only
  }
  c = compare(ctx)
  assert c.matched_miles == 0 and c.comma_saving is None
  assert headline(c).startswith('collecting')


def test_battery_drain_counts_against_the_side_that_used_it():
  base = ('30-45 mph', 'flat', 'straight', 'open road')
  fair = compare({base + ('driver',): t(10, 0.25), base + ('pulse-glide',): t(10, 0.25)})
  drained = compare({base + ('driver',): t(10, 0.25), base + ('pulse-glide',): t(10, 0.25, soc=-20.0)})
  assert fair.comma_saving == pytest.approx(0.0, abs=1e-9)
  assert drained.comma_saving < 0  # same gallons but it borrowed battery


def test_headline_and_stored_ledger_round_trip(tmp_path):
  from openpilot.starpilot.efficiency.ledger import Stretch
  def stretch(mode, miles, gal):
    s = Stretch(mode, 0.0, 600.0, distance=miles * MILE, obd_first=0.0, obd_last=gal)
    return s
  save_drive('d1', [stretch('driver', MIN_MATCHED_MILES, 0.2), stretch('chill', MIN_MATCHED_MILES, 0.1)], tmp_path)
  c = stored_comparison(tmp_path)
  assert c.ready and c.comma_saving == pytest.approx(0.5)
  assert headline(c) == f'comma saves 50% fuel · {MIN_MATCHED_MILES:.0f} mi'


def test_drive_modes_are_never_mixed():
  ctx = {
    ('45-60 mph', 'flat', 'straight', 'open road', 'driver', 'eco'): t(10, 0.25),
    ('45-60 mph', 'flat', 'straight', 'open road', 'chill', 'sport'): t(10, 0.40),  # comma only drove Sport here
  }
  assert compare(ctx).matched_miles == 0
  ctx[('45-60 mph', 'flat', 'straight', 'open road', 'chill', 'eco')] = t(10, 0.20)
  c = compare(ctx)
  assert c.matched_miles == pytest.approx(10) and c.comma_saving == pytest.approx(0.2)


# ---- per tank: the driver's numbers keep averaging, the comma's start over at each fill-up ----
def _drive(root, drive_id, who, miles, gal):
  from openpilot.starpilot.efficiency.ledger import Stretch
  save_drive(drive_id, [Stretch(who, 0.0, 600.0, distance=miles * MILE, obd_first=0.0, obd_last=gal)], root)


def _tank(miles_since_fill, fill_drive=None):
  from openpilot.starpilot.efficiency.tank import TankState
  return TankState(miles_since_fill=miles_since_fill, last_range_mi=380, fills_seen=1, fill_drive=fill_drive)


def test_the_comma_counts_only_since_the_fill_up_and_the_driver_counts_every_drive(tmp_path):
  from openpilot.starpilot.efficiency.compare import tank_comparison
  _drive(tmp_path, '20261101-080000-1', 'driver', 10, 0.25)   # 40 mpg, before the fill-up: still counts
  _drive(tmp_path, '20261102-080000-1', 'chill', 10, 0.50)    # 20 mpg, the old tank: left out
  _drive(tmp_path, '20261110-080000-1', 'chill', 10, 0.20)    # 50 mpg, this tank
  c = tank_comparison(tmp_path, _tank(10.0, fill_drive='20261110-080000-1'))
  assert c.ready and c.comma_miles == pytest.approx(10) and c.matched_miles == pytest.approx(10)
  assert c.comma_saving == pytest.approx(1 - 0.20 / 0.25)
  everything = tank_comparison(tmp_path, None)   # no fill-up seen yet: every drive counts for both
  assert everything.comma_miles == pytest.approx(20)


def test_right_after_a_fill_up_the_tile_starts_collecting_again(tmp_path):
  from openpilot.starpilot.efficiency.compare import tank_headline
  _drive(tmp_path, '20261101-080000-1', 'driver', 10, 0.25)
  _drive(tmp_path, '20261102-080000-1', 'chill', 10, 0.20)
  assert tank_headline(tmp_path, _tank(0.0, fill_drive='20261103-080000-1')) == 'this tank: collecting 0.0 of 5 mi'


def test_an_older_fill_up_without_its_drive_is_found_by_walking_back_the_miles(tmp_path):
  from openpilot.starpilot.efficiency.compare import tank_comparison
  _drive(tmp_path, '20261101-080000-1', 'driver', 10, 0.25)
  _drive(tmp_path, '20261102-080000-1', 'chill', 10, 0.50)    # before the fill-up (the tank has had 20 mi)
  _drive(tmp_path, '20261110-080000-1', 'chill', 10, 0.20)
  _drive(tmp_path, '20261111-080000-1', 'chill', 10, 0.20)
  c = tank_comparison(tmp_path, _tank(20.0))
  assert c.comma_miles == pytest.approx(20) and c.comma_saving == pytest.approx(1 - 0.02 / 0.025)


def test_each_finished_tank_is_kept_and_shown_under_the_new_one(tmp_path):
  from openpilot.starpilot.efficiency.compare import load_tank_history, record_finished_tank, tank_headline
  _drive(tmp_path, '20261101-080000-1', 'driver', 10, 0.25)
  _drive(tmp_path, '20261110-080000-1', 'chill', 10, 0.20)
  ended = record_finished_tank(tmp_path, _tank(10.0, fill_drive='20261110-080000-1'), now=0)
  assert ended['ready'] and ended['comma_saving'] == pytest.approx(0.2)
  assert load_tank_history(tmp_path) == [ended]
  assert tank_headline(tmp_path, _tank(0.0, fill_drive='20261120-080000-1')) == \
         'this tank: collecting 0.0 of 5 mi\nlast tank: saved 20%'
  assert record_finished_tank(tmp_path, _tank(0.0, fill_drive='20261120-080000-1')) is None   # nothing driven: not kept


def test_the_recorder_starts_the_new_tank_at_the_drive_the_fill_up_was_seen_in(tmp_path):
  from openpilot.starpilot.efficiency.efficiencyd import Recorder
  from openpilot.starpilot.efficiency.hybrid import Cluster
  from openpilot.starpilot.efficiency.tank import load, save
  _drive(tmp_path, '20261101-080000-1', 'driver', 10, 0.25)
  _drive(tmp_path, '20261110-080000-1', 'chill', 10, 0.20)
  save(_tank(10.0, fill_drive='20261110-080000-1'), tmp_path / 'tank.json')
  rec = Recorder(1500.0, None, tmp_path)
  rec.observe_can(None, Cluster(avg_mpg=50.0, range_mi=520))   # range jumped from 380: a fill-up
  assert rec.tank.fills_seen == 2 and rec._fill_pending and rec.tank.fill_drive is None
  rec.save()
  assert rec.tank.fill_drive == rec.drive_id and load(tmp_path / 'tank.json').fill_drive == rec.drive_id
  from openpilot.starpilot.efficiency.compare import load_tank_history
  assert load_tank_history(tmp_path)[-1]['comma_saving'] == pytest.approx(0.2)
