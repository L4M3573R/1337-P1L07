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
