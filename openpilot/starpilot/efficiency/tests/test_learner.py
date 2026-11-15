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
  assert depth40 == pytest.approx(MAX_UNDER_MPH)   # 4 min/hr allows the full 5 mph at 40 mph
  assert band_time_cost(40.0, depth40) == pytest.approx(TIME_BUDGET_MIN_PER_HR)
  assert band_time_cost(65.0, band_depth_mph(65.0)) < TIME_BUDGET_MIN_PER_HR
  assert band_depth_mph(5.0) == MIN_BAND_MPH
  for set_mph in (20.0, 30.0, 45.0, 55.0, 70.0, 80.0):
    assert band_time_cost(set_mph, band_depth_mph(set_mph)) <= learner.TIME_LIMIT_MIN_PER_HR + 1e-9  # never past the budget


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
  headroom = 1450.0 * 55 * learner.MPH * learner.DOWNHILL_OVER_MPH * learner.MPH  # limited by the speed it may carry
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


def _obd_flow(shares, samples=400, accel='-0.05', power_share=0.2):
  """shares: {speed bin mph: BlueDriver engine-off share} -> 7-number histogram entries (power flow says power_share)."""
  return {f'{mph:g}|{accel}|glide': [samples, samples * power_share, 0, samples * (1 - power_share), 0.0, samples, samples * share]
          for mph, share in shares.items()}


def test_ev_cutoff_prefers_recent_bluedriver_readings():
  recent = _obd_flow({56: 0.97, 58: 0.96, 60: 0.95, 62: 0.5})
  result = learn_ev_cutoff(_flow({60: 0.2, 62: 0.2}), recent, window_days=14, ambient_c=21.0)
  assert result['source'] == 'bluedriver' and result['learned_mph'] == 62 and result['status'] == 'learned'
  assert result['window_days'] == 14 and result['ambient_c'] == 21.0
  # recent drives without BlueDriver readings fall back to their power flow, then to all drives
  assert learn_ev_cutoff({}, _flow({56: 0.97, 58: 0.96, 60: 0.4}))['source'] == 'power_flow'
  fallback = learn_ev_cutoff(_flow({56: 0.97, 58: 0.96, 60: 0.4}), {})
  assert fallback['source'] == 'power_flow_all' and fallback['learned_mph'] == 60 and fallback['window_days'] is None


def test_ev_cutoff_reports_when_the_engine_runs_in_every_light_glide():
  result = learn_ev_cutoff({}, _obd_flow({40: 0.12, 44: 0.25, 48: 0.18}), window_days=14, ambient_c=9.0)
  assert result['status'] == 'engine_on' and result['learned_mph'] is None
  assert (result['best_mph'], result['best_share']) == (44, 0.25)
  text = '\n'.join(report_lines(learn([], {}, recent_flow=_obd_flow({40: 0.12, 44: 0.25}), window_days=14, ambient_c=9.0)))
  assert 'the engine runs in light glides from 40 mph (best 25% electric at 44 mph' in text
  assert 'BlueDriver engine rpm, last 14 days, ~9 C outside' in text


def test_learn_stored_relearns_the_cutoff_from_the_newest_drives(tmp_path):
  drives = tmp_path / 'drives'
  drives.mkdir()
  warm = {'version': 1, 'drive': '20261015-120000-1', 'stretches': [_stretch('chill', 6.0, 55.0, 0.1) | {'ambient_c': 20.0}],
          'flow': _obd_flow({56: 0.97, 58: 0.96, 60: 0.95, 62: 0.4})}
  cold = {'version': 1, 'drive': '20261115-120000-2', 'stretches': [_stretch('chill', 6.0, 55.0, 0.1) | {'ambient_c': 9.0}],
          'flow': _obd_flow({40: 0.1, 44: 0.2, 48: 0.15})}
  (drives / 'warm.json').write_text(json.dumps(warm))
  (drives / 'cold.json').write_text(json.dumps(cold))
  ev = learn_stored(tmp_path)['ev_cutoff']
  assert ev['source'] == 'bluedriver' and ev['status'] == 'engine_on' and ev['ambient_c'] == 9.0  # October is out of the window
  assert learn_stored(tmp_path, window_days=60)['ev_cutoff']['ambient_c'] == 14.5  # both drives inside a longer window
