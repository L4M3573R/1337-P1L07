"""Shadow efficiency learner: EV cut-off, Pulse and Glide vs steady cruise within the time budget, downhill carry."""
import json

import pytest

from openpilot.starpilot.efficiency import learner
from openpilot.starpilot.efficiency.learner import (MAX_UNDER_MPH, MIN_BAND_MPH, TIME_BUDGET_MIN_PER_HR, band_depth_mph,
                                                    band_time_cost, learn, learn_bands, learn_downhill, learn_ev_cutoff,
                                                    learn_stored, report_lines)


def test_band_depth_stays_inside_the_time_budget():
  assert band_depth_mph(65.0) == MAX_UNDER_MPH  # the budget would allow a little more; the 5 mph floor caps it
  depth40 = band_depth_mph(40.0)
  assert 3.0 < depth40 < 3.5
  assert band_time_cost(40.0, depth40) == pytest.approx(TIME_BUDGET_MIN_PER_HR)
  assert band_time_cost(65.0, band_depth_mph(65.0)) < TIME_BUDGET_MIN_PER_HR
  assert band_depth_mph(10.0) == MIN_BAND_MPH
  for set_mph in (20.0, 30.0, 45.0, 55.0, 70.0, 80.0):
    assert band_time_cost(set_mph, band_depth_mph(set_mph)) <= 3.0  # never past the driver's 3 min/hr maximum


def _flow(shares, samples=400, accel='-0.05'):
  """shares: {speed bin mph: EV share} -> a ledger power-flow histogram."""
  out = {}
  for mph, share in shares.items():
    out[f'{mph:g}|{accel}|glide'] = [samples, samples * share, 0, samples * (1 - share), 0.0]
  return out


def test_ev_cutoff_found_where_the_engine_starts():
  result = learn_ev_cutoff(_flow({56: 0.97, 58: 0.96, 60: 0.95, 62: 0.93, 64: 0.40, 66: 0.10}))
  assert result['learned_mph'] == 64 and not result['at_least']
  assert result['current_mph'] == pytest.approx(63.0)


def test_ev_cutoff_needs_data_and_reports_at_least_when_never_crossed():
  assert learn_ev_cutoff(_flow({60: 0.95}))['learned_mph'] is None          # one bin is not enough
  assert learn_ev_cutoff(_flow({60: 0.95, 62: 0.95}, samples=50))['learned_mph'] is None  # too few samples
  ok = learn_ev_cutoff(_flow({58: 0.95, 60: 0.95, 62: 0.92}))
  assert ok['learned_mph'] == 64 and ok['at_least']
  # harder acceleration or regen bins are not feather glides
  assert learn_ev_cutoff(_flow({58: 0.95, 60: 0.95, 62: 0.92}, accel='-0.30'))['samples'] == 0


def _stretch(mode, miles, mph, gal, ctx=('45-60 mph', 'flat', 'straight', 'open road'), soc_d=0.0, drive_mode='eco', **extra):
  meters = miles * learner.METERS_PER_MILE
  return {'mode': mode, 'm': meters, 'dur': meters / (mph * learner.MPH), 'obd_gal': gal, 'soc_d': soc_d,
          'ctx': list(ctx), 'drive_mode': drive_mode, **extra}


def test_pulse_glide_recommended_where_it_saves_with_the_budget_band():
  records = [_stretch('pulse-glide', 6.0, 53.0, 6.0 / 60), _stretch('chill', 6.0, 55.0, 6.0 / 50)]
  band = learn_bands(records)['45-60 mph']
  assert band['saving'] == pytest.approx(1 - 50 / 60, abs=1e-3)
  assert band['recommend'] == 'pulse-glide'
  assert band['depth_mph'] == pytest.approx(band_depth_mph(55.0), abs=0.05)  # steady cruise runs at the set speed
  assert band['planned_time_cost_min_per_hr'] <= TIME_BUDGET_MIN_PER_HR
  assert band['measured_time_cost_min_per_hr'] == pytest.approx(60 * (55 / 53 - 1), abs=0.01)


def test_steady_cruise_kept_where_pulse_glide_does_not_pay_and_contexts_must_match():
  same = [_stretch('pulse-glide', 6.0, 53.0, 6.0 / 50.5), _stretch('chill', 6.0, 55.0, 6.0 / 50)]
  assert learn_bands(same)['45-60 mph']['recommend'] == 'steady cruise'
  # Pulse and Glide downhill against steady cruise uphill is not a comparison
  mismatched = [_stretch('pulse-glide', 6.0, 53.0, 0.05, ctx=('45-60 mph', 'downhill', 'straight', 'open road')),
                _stretch('chill', 6.0, 55.0, 0.2, ctx=('45-60 mph', 'uphill', 'straight', 'open road'))]
  assert learn_bands(mismatched) == {}
  # Sport is only compared with Sport
  assert learn_bands([_stretch('pulse-glide', 6.0, 53.0, 0.1, drive_mode='sport'), _stretch('chill', 6.0, 55.0, 0.12)]) == {}


def test_bands_use_battery_adjusted_measured_fuel_only():
  # Pulse and Glide "saved" fuel only by draining the battery: once paid back it is no better
  drained = [_stretch('pulse-glide', 6.0, 53.0, 0.10, soc_d=-30.0), _stretch('chill', 6.0, 55.0, 0.1356)]
  assert learn_bands(drained)['45-60 mph']['recommend'] == 'steady cruise'
  # stretches without BlueDriver fuel are skipped, never filled in from the dash
  no_obd = [_stretch('pulse-glide', 6.0, 53.0, None, dash_gal=0.05), _stretch('chill', 6.0, 55.0, 0.12)]
  assert learn_bands(no_obd) == {}
  assert learn_bands([_stretch('pulse-glide', 2.0, 53.0, 0.03), _stretch('chill', 2.0, 55.0, 0.04)])['45-60 mph']['recommend'] == 'collecting'


def test_downhill_carry_counts_each_hill_once_and_only_open_road():
  hill = ('45-60 mph', 'downhill', 'straight', 'open road')
  flat = ('45-60 mph', 'flat', 'straight', 'open road')
  big = 5e6
  drive = [_stretch('chill', 0.2, 55, 0.003, ctx=flat),
           _stretch('chill', 0.2, 55, 0.0, ctx=hill, regen_j=big, fric_j=0.0),
           _stretch('chill', 0.2, 55, 0.0, ctx=hill, regen_j=big, fric_j=0.0),
           _stretch('chill', 0.2, 55, 0.003, ctx=flat),
           _stretch('chill', 0.2, 55, 0.0, ctx=('45-60 mph', 'downhill', 'straight', 'following'), regen_j=big),
           _stretch('driver', 0.2, 55, 0.0, ctx=hill, regen_j=big)]
  result = learn_downhill([drive], mass_kg=1450.0)
  assert result['hills'] == 1
  headroom = 1450.0 * 55 * learner.MPH * 2 * learner.MPH  # limited by the 2 mph of speed it may carry
  assert result['est_gal_saved'] == pytest.approx(headroom / learner.WHEEL_J_PER_GAL, abs=1e-4)
  assert result['time_won_s'] > 0
  small = learn_downhill([[_stretch('chill', 0.2, 55, 0.0, ctx=hill, regen_j=1e4, fric_j=2e3)]])
  assert small['est_gal_saved'] == pytest.approx((0.4 * 1e4 + 2e3) / learner.WHEEL_J_PER_GAL, abs=1e-6)


def test_learn_stored_reads_drive_files_and_reports(tmp_path):
  drives = tmp_path / 'drives'
  drives.mkdir()
  doc = {'version': 1, 'stretches': [_stretch('pulse-glide', 6.0, 53.0, 0.1), _stretch('chill', 6.0, 55.0, 0.12)],
         'flow': _flow({56: 0.97, 58: 0.96, 60: 0.95, 64: 0.3})}
  (drives / 'a.json').write_text(json.dumps(doc))
  (drives / 'old.json').write_text(json.dumps({'version': 0, 'stretches': [_stretch('chill', 50.0, 55.0, 0.1)]}))
  (drives / 'bad.json').write_text('{broken')
  result = learn_stored(tmp_path)
  assert result['mode'] == 'shadow'
  assert result['bands']['45-60 mph']['recommend'] == 'pulse-glide'
  assert result['ev_cutoff']['learned_mph'] == 64
  text = '\n'.join(report_lines(result))
  assert 'EV cut-off: 64 mph learned' in text and 'saves 17%' in text
  assert '\n'.join(report_lines(learn([], {}))).count('collecting') == 1
