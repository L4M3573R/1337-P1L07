"""Context tags (drive mode, tires, map) per stretch, and the PC trip reports built from them."""
import pytest

from openpilot.starpilot.efficiency import trips
from openpilot.starpilot.efficiency.car_signals import CarSignals
from openpilot.starpilot.efficiency.ledger import Ledger, Sample
from openpilot.starpilot.efficiency.store import drive_document

MILE = 1609.344


def mdps(mode):
  return bytes([0, 0, 0, 0, 0, 0, 0, (mode & 3) << 3])


def test_car_signals_decode_drive_mode_tires_and_map():
  s = CarSignals()
  assert s.drive_mode is None
  s.update([(0x381, mdps(0), 0), (0x44E, bytes([0x01]), 0)])
  assert s.drive_mode == 'eco'
  s.update([(0x381, mdps(1), 0)])
  assert s.drive_mode == 'sport'
  s.update([(0x381, mdps(0), 0), (0x44E, bytes([0x81]), 0)])
  assert s.drive_mode == 'smart'
  s.update([(0x593, bytes.fromhex('000122232223'), 0)])  # as on the speed-test drive
  assert s.tires_psi == (34, 35, 34, 35) and s.tire_avg == 34.5 and s.tire_min == 34
  s.update([(0x593, bytes.fromhex('0000ffffffff'), 0)])  # parked: sensors asleep
  assert s.tires_psi is None
  s.update([(0x544, bytes.fromhex('4160413a00000000'), 0)])
  assert s.map_limit == 65 and s.road_code == 1
  s.update([(0x544, bytes.fromhex('ff77ffff00000000'), 0), (0x381, mdps(1), 2)])  # unknown map; other bus ignored
  assert s.map_limit is None and s.drive_mode == 'smart'


def drive(seconds=60.0, **tags):
  ledger, t = Ledger(), 0.0
  while t < seconds:
    ledger.add(Sample(t=t, speed=20.0, accel=0.0, pitch=0.0, mass=1500.0, long_active=True, obd_fuel_gal=0.0004 * t, **tags))
    t += 0.05
  return ledger.finish()


def test_stretch_records_carry_the_tags():
  stretches = drive(drive_mode='sport', tire_avg=34.5, tire_min=34, ambient_c=7.0, map_limit=65, road_code=1, fuel_level=48.0,
                    lat=40.71234, lon=-74.00123, bearing=90.0)
  rec = drive_document('d', stretches)['stretches'][0]
  assert rec['drive_mode'] == 'sport' and rec['limit'] == 65 and rec['road'] == 1
  assert rec['tire'] == 34.5 and rec['tire_low'] == 34 and rec['ambient_c'] == 7.0 and rec['level0'] == 48.0
  assert rec['lat'] == 40.71234 and rec['bearing'] == 90 and rec['mode'] == 'chill' and rec['obd_gal'] > 0


def rec(miles, gal, mode='driver', **kw):
  return {'m': miles * MILE, 'dur': miles * 80, 'obd_gal': gal, 'soc_d': 0.0, 'mode': mode, **kw}


def test_trip_summary_and_breakdowns():
  records = [rec(10, 0.25, drive_mode='eco', tire=35, ambient_c=10, limit=65),
             rec(10, 0.20, 'pulse-glide', drive_mode='eco', tire=35, ambient_c=10, limit=65),
             rec(5, 0.15, drive_mode='sport', tire=32, ambient_c=-2, limit=35)]
  summary = trips.drive_summary(0.0, records, gas_price=3.0)
  assert summary['miles'] == pytest.approx(25.0) and summary['mpg'] == pytest.approx(25 / 0.60, abs=0.1)
  assert summary['you']['mpg'] == pytest.approx(37.5) and summary['comma']['mpg'] == pytest.approx(50.0)
  assert summary['cost'] == pytest.approx(1.80) and summary['drive_mode_miles'] == {'eco': 20.0, 'sport': 5.0}
  b = trips.breakdowns(records)
  assert b['drive_mode']['eco']['mpg'] == pytest.approx(20 / 0.45, abs=0.1)
  assert b['road_type']['highway (60+)']['miles'] == pytest.approx(20.0)
  assert b['tire_psi_by_temp']['50-60F | 35 psi']['mpg'] == pytest.approx(20 / 0.45, abs=0.1)
  assert b['who_by_road']['highway (60+) | comma']['mpg'] == pytest.approx(50.0)


def test_fill_up_detection_and_tank_check():
  drives = [(1.0, [rec(10, 0.25, level0=60.0, level1=58.0)]), (2.0, [rec(10, 0.25, level0=57.5, level1=55.0)]),
            (3.0, [rec(1, 0.02, level0=95.0, level1=95.0)]),
            (4.0, [rec(30, 0.6, level0=95.0, level1=90.0)])]
  fills = trips.fill_ups(drives, tank_gal=12.0)
  assert len(fills) == 1 and fills[0]['from_pct'] == 55.0 and fills[0]['gallons_added'] == pytest.approx(4.8)
  check = trips.tank_check(drives, tank_gal=12.0)
  assert check['tank_gal'] == pytest.approx(0.6) and check['measured_gal'] == pytest.approx(0.62)
