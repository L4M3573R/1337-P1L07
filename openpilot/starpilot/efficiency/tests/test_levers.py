"""The fuel levers report: speed, heat, tire pressure and cold starts, from synthetic stretch records."""
import json
from datetime import date

import pytest

from openpilot.starpilot.efficiency import levers as lv

MILE = 1609.344


def st(t, mph, mpg, miles=1.0, coolant=90, climate='heat', ambient=0.0, tire=34, ev=0.2, pitch=0.0, soc=0.0):
  dur = miles / mph * 3600.0
  v = mph / lv.MPS_TO_MPH
  return {'t': t, 'dur': dur, 'm': miles * MILE, 'obd_gal': miles / mpg, 'soc_d': soc, 'pitch': pitch, 'v0': v, 'v1': v,
          'coolant': coolant, 'climate': climate, 'ambient_c': ambient, 'tire_low': tire, 'tire': tire, 'ev': ev,
          'mode': 'driver', 'ctx': ['60+ mph', 'flat', 'straight', 'open road']}


def doc(drive, stretches):
  return {'version': 1, 'drive': drive, 'speed_factor': 1.0223, 'stretches': stretches}


def test_battery_change_is_paid_back_in_fuel():
  s = st(0, 60, 50.0, soc=-10.0)
  assert lv.adjusted_gal(s) == pytest.approx(1 / 50.0 + 0.1 * lv.BATTERY_KWH * lv.GAL_PER_BATTERY_KWH)
  assert lv.adjusted_gal({'obd_gal': None}) is None


def test_speed_prices_the_miles_over_70_at_the_best_measured_speed():
  stretches = [st(i, 68, 52.0) for i in range(6)] + [st(10 + i, 69, 52.0) for i in range(6)]
  stretches += [st(20 + i, 70, 51.0) for i in range(6)] + [st(30 + i, 75, 42.0) for i in range(4)]
  stretches += [st(40 + i, 74, 44.0) for i in range(4)] + [st(50 + i, 76, 41.0) for i in range(4)]
  out = lv.speed_lever(stretches, set())
  assert out['status'] == 'ok' and out['fast_miles'] == pytest.approx(12)
  assert out['curve']['75'] == pytest.approx(12 / (4 / 42 + 4 / 44 + 4 / 41), rel=1e-3)
  assert out['saved_gal'] > 0 and out['minutes'] > 0
  assert 'over 70 mph' in out['line']


def test_speed_without_steady_driving_at_the_fast_speeds_is_collecting():
  stretches = [st(i, 75, 42.0, miles=0.3) for i in range(3)]
  assert lv.speed_lever(stretches, set())['status'] == 'collecting'


def test_heat_compared_inside_one_temperature_band():
  stretches = [st(i, 50, 40.0, climate='heat', ambient=2.0, ev=0.1) for i in range(6)]
  stretches += [st(10 + i, 50, 50.0, climate='off', ambient=3.0, ev=0.4) for i in range(6)]
  stretches += [st(20 + i, 50, 60.0, climate='off', ambient=20.0) for i in range(6)]   # other weather: not compared
  out = lv.heat_lever(stretches, set())
  assert out['status'] == 'ok' and len(out['bands']) == 1
  assert out['bands'][0]['heat_mpg'] == pytest.approx(40.0) and out['bands'][0]['off_mpg'] == pytest.approx(50.0)
  assert out['cost_share'] == pytest.approx(0.2)


def test_heat_with_no_heat_off_driving_is_collecting():
  out = lv.heat_lever([st(i, 50, 40.0) for i in range(10)], set())
  assert out['status'] == 'collecting' and out['heat_miles'] == pytest.approx(10)


def test_tires_flag_under_the_placard_and_compare_pressures():
  stretches = [st(i, 50, 44.0, tire=32) for i in range(6)] + [st(10 + i, 50, 50.0, tire=35) for i in range(6)]
  out = lv.tire_lever(stretches, set())
  assert out['under_placard'] and out['placard_assumed'] and out['lowest_psi'] == 32
  assert out['bands'][0]['low_mpg'] == pytest.approx(44.0) and out['bands'][0]['high_mpg'] == pytest.approx(50.0)
  assert 'add air' in out['line'] and out['placard_psi'] == 33.0


def test_cold_start_share_by_coolant():
  stretches = [st(i, 30, 25.0, coolant=40) for i in range(5)] + [st(10 + i, 50, 50.0) for i in range(20)]
  d = doc('20261125-0600', stretches)
  out = lv.cold_lever(stretches, lv._cold_ids([d]))
  assert out['status'] == 'ok' and out['cold_mpg'] == pytest.approx(25.0) and out['warm_mpg'] == pytest.approx(50.0)
  assert out['cold_share'] == pytest.approx((5 / 25) / (5 / 25 + 20 / 50), abs=1e-3)


def test_report_uses_the_last_week_and_falls_back_to_everything(tmp_path):
  (tmp_path / 'drives').mkdir()
  old = doc('20261101-0600', [st(i, 50, 50.0) for i in range(30)])
  new = doc('20261124-0600', [st(i, 50, 45.0) for i in range(30)])
  for d in (old, new):
    (tmp_path / 'drives' / f"{d['drive']}.json").write_text(json.dumps(d))
  r = lv.report(tmp_path, today=date(2026, 11, 25))
  assert r['span'] == 'last 7 days' and r['drives'] == 1
  r = lv.report(tmp_path, today=date(2026, 12, 25))
  assert r['span'] == 'all stored drives' and r['drives'] == 2
  lines = lv.report_lines(r)
  assert lines[0].startswith('Fuel levers') and len(lines) == 5
