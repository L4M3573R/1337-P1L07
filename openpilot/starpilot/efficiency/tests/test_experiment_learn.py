"""Self-learning fuel efficiency: judging experiment variants from measured fuel."""
import json
import math
import random

from openpilot.starpilot.efficiency import experiment_learn as el


def stretch(tag, gal, m=250.0, soc_d=0.0):
  return {'exp': tag, 'obd_gal': gal, 'm': m, 'soc_d': soc_d}


def steady_drive(rng, n, tag, gal_per_mile):
  return [stretch(tag, max(0.0, rng.gauss(gal_per_mile * 250 / 1609.344, gal_per_mile * 250 / 1609.344 * 0.15))) for _ in range(n)]


def test_a_real_saving_wins_once_measured_enough():
  rng = random.Random(3)
  drives = [steady_drive(rng, 30, 'uphill|30-45|hold', 0.030) + steady_drive(rng, 30, 'uphill|30-45|sag3', 0.027)]
  r = el.judge(el.samples(drives))['uphill|30-45']
  assert r['winner'] == 'sag3' and 'saves' in r['status'] and r['confidence'] >= el.CONFIDENCE


def test_not_enough_miles_keeps_collecting():
  rng = random.Random(4)
  drives = [steady_drive(rng, 5, 'cruise|45-60|set', 0.03) + steady_drive(rng, 30, 'cruise|45-60|under2', 0.02)]
  r = el.judge(el.samples(drives))['cruise|45-60']
  assert r['winner'] is None and r['status'].startswith('collecting')


def test_a_tiny_or_noisy_difference_is_not_a_winner():
  rng = random.Random(5)
  drives = [steady_drive(rng, 30, 'follow|30-45|normal', 0.030) + steady_drive(rng, 30, 'follow|30-45|smooth', 0.0297)]
  r = el.judge(el.samples(drives))['follow|30-45']
  assert r['winner'] in (None, 'normal')


def test_battery_charge_counts_as_fuel():
  assert el.adjusted_gal(stretch('x', 0.01, soc_d=-2.0)) > 0.01   # used charge: fuel spent earlier
  assert el.adjusted_gal(stretch('x', 0.01, soc_d=+2.0)) < 0.01   # banked charge
  assert el.adjusted_gal({'exp': 'x', 'obd_gal': None}) is None


def test_launch_is_scored_over_a_fixed_distance():
  # a gentle launch covers less road in its episode; the window adds the following stretch so both cover 400 m
  drive = []
  for _ in range(10):
    drive += [stretch('launch|launch|normal', 0.004, m=300), stretch(None, 0.002, m=250)]
    drive += [stretch('launch|launch|gentle', 0.002, m=150), stretch(None, 0.003, m=250)]
    drive += [stretch('launch|launch|moderate', 0.003, m=200), stretch(None, 0.0025, m=250)]
  data = el.samples([drive])
  assert len(data[('launch', 'launch', 'normal')]) == 10
  assert all(run.metres >= 400 for run in data[('launch', 'launch', 'gentle')])


def test_update_writes_winners(tmp_path):
  rng = random.Random(6)
  (tmp_path / 'drives').mkdir()
  drive = steady_drive(rng, 30, 'downhill|30-45|hold', 0.02) + steady_drive(rng, 30, 'downhill|30-45|carry3', 0.016)
  (tmp_path / 'drives' / 'd1.json').write_text(json.dumps({'stretches': drive}))
  doc = el.update(tmp_path)
  assert doc['winners'] == {'downhill|30-45': 'carry3'}
  assert json.loads((tmp_path / 'experiments.json').read_text())['winners'] == {'downhill|30-45': 'carry3'}
  assert any('downhill' in line for line in el.report_lines(doc))


def test_a_variant_that_happened_to_run_downhill_is_not_flattered():
  # both variants burn the same on level road; 'pg' was always measured on a 1 % downhill
  rng = random.Random(7)
  base = 0.030 * 250 / 1609.344
  drive = []
  for _ in range(40):
    climb = -0.01 * 250
    drive.append({'exp': 'pulse_glide|30-45|steady', 'obd_gal': max(0.0, rng.gauss(base, base * 0.05)), 'm': 250.0,
                  'pitch': 0.0, 'soc_d': 0.0})
    drive.append({'exp': 'pulse_glide|30-45|pg', 'obd_gal': max(0.0, rng.gauss(base + el.DEFAULT_CLIMB * climb, base * 0.05)),
                  'm': 250.0, 'pitch': math.asin(-0.01), 'soc_d': 0.0})
  raw = el.judge(el.samples([drive]))['pulse_glide|30-45']
  assert raw['winner'] == 'pg'   # uncorrected, the downhill makes it look better
  fixed = el.judge(el.samples([drive], {'climb': el.DEFAULT_CLIMB, 'speed': el.DEFAULT_SPEED}))['pulse_glide|30-45']
  assert fixed['winner'] != 'pg'


def test_speed_change_is_charged():
  c = {'climb': 0.0, 'speed': el.DEFAULT_SPEED}
  slower_end = {'obd_gal': 0.004, 'm': 400.0, 'v0': 0.0, 'v1': 10.0}
  faster_end = {'obd_gal': 0.004, 'm': 400.0, 'v0': 0.0, 'v1': 15.0}
  assert el.corrected_gal(faster_end, c) < el.corrected_gal(slower_end, c)   # it banked more speed for the same fuel


def test_fit_falls_back_to_physics_without_enough_data():
  assert el.fit_corrections([[{'obd_gal': 0.004, 'm': 250.0, 'pitch': 0.01}]])['source'] == 'physics'



def runs(rng, n, gal_per_mile, *, curve='straight', ambient=None, seconds_per_mile=90.0, spread=0.05):
  m = 400.0
  out = []
  for _ in range(n):
    g = max(0.0, rng.gauss(gal_per_mile * m / 1609.344, gal_per_mile * m / 1609.344 * spread))
    out.append({'obd_gal': g, 'm': m, 'soc_d': 0.0, 'dur': seconds_per_mile * m / 1609.344,
                'ctx': ['45-60 mph', 'flat', curve, 'open road'], 'ambient_c': ambient, 'mode': 'chill'})
  return out


def tagged(tag, stretches):
  return [s | {'exp': tag} for s in stretches]


def test_summer_runs_are_never_weighed_against_winter_ones(tmp_path):
  rng = random.Random(11)
  (tmp_path / 'drives').mkdir()
  # summer: holding flat paid off; winter (the newest drive): it does not, normal driving is better
  summer = tagged('hold|45-60|normal', runs(rng, 20, 0.030, ambient=28.0)) + tagged('hold|45-60|flat', runs(rng, 20, 0.024, ambient=28.0))
  winter = tagged('hold|45-60|normal', runs(rng, 20, 0.030, ambient=3.0)) + tagged('hold|45-60|flat', runs(rng, 20, 0.033, ambient=3.0))
  (tmp_path / 'drives' / 'a_summer.json').write_text(json.dumps({'stretches': summer}))
  (tmp_path / 'drives' / 'b_winter.json').write_text(json.dumps({'stretches': winter}))
  import os
  os.utime(tmp_path / 'drives' / 'a_summer.json', (1, 1))   # older
  doc = el.update(tmp_path)
  assert doc['season'] == 'cold' and doc['winners'].get('hold|45-60') == 'normal'
  assert doc['results']['hold|45-60']['measured']['flat'] == round(20 * 400 / 1609.344, 2)   # the winter miles only, not summer's too
  assert el.season_of(9.9) == 'cold' and el.season_of(15.0) == 'mild' and el.season_of(25.0) == 'warm' and el.season_of(None) is None


def test_a_variant_driven_on_straighter_roads_is_not_flattered():
  rng = random.Random(12)
  # both burn the same on each kind of road; 'under2' just happened to get mostly straight road, 'set' mostly curves
  set_runs = runs(rng, 6, 0.025, curve='straight') + runs(rng, 30, 0.035, curve='curvy')
  under_runs = runs(rng, 30, 0.025, curve='straight') + runs(rng, 6, 0.035, curve='curvy')
  data = el.samples([tagged('cruise|45-60|set', set_runs) + tagged('cruise|45-60|under2', under_runs) +
                     tagged('cruise|45-60|under4', set_runs)])
  assert el.gal_per_100mi(data[('cruise', '45-60', 'under2')]) < 0.9 * el.gal_per_100mi(data[('cruise', '45-60', 'set')])  # pooled
  r = el.judge(data)['cruise|45-60']
  assert r['winner'] in (None, 'set') and set(r['curves']) == {'straight', 'curvy'}


def test_time_free_savings_are_always_taken():
  rng = random.Random(13)
  drive = tagged('hold|60+|normal', runs(rng, 20, 0.040)) + tagged('hold|60+|flat', runs(rng, 20, 0.036))   # same time
  r = el.judge(el.samples([drive]), situation_miles_per_hour={'hold|60+': 30.0}, budget_min_per_hr=0.0)['hold|60+']
  assert r['winner'] == 'flat' and 'no extra time' in r['status']


def test_the_budget_goes_where_a_minute_saves_the_most_fuel():
  rng = random.Random(14)
  # 60+: 'under2' saves 0.4 gal/100 mi for 2 min/100 mi; 30-45 'under2' saves 0.15 for the same 2 min
  drive = (tagged('cruise|60+|set', runs(rng, 20, 0.040, seconds_per_mile=55.0)) +
           tagged('cruise|60+|under2', runs(rng, 20, 0.036, seconds_per_mile=56.2)) +
           tagged('cruise|60+|under4', runs(rng, 20, 0.0355, seconds_per_mile=60.0)) +
           tagged('cruise|30-45|set', runs(rng, 20, 0.030, seconds_per_mile=90.0)) +
           tagged('cruise|30-45|under2', runs(rng, 20, 0.0285, seconds_per_mile=91.2)) +
           tagged('cruise|30-45|under4', runs(rng, 20, 0.0300, seconds_per_mile=95.0)))
  share = {'cruise|60+': 40.0, 'cruise|30-45': 40.0}   # miles per engaged hour in each
  # room for one 2 min/100 mi step at 40 mi/hr (0.8 min/hr): the highway gets it
  r = el.judge(el.samples([drive]), situation_miles_per_hour=share, budget_min_per_hr=1.0)
  assert r['cruise|60+']['winner'] == 'under2' and 'pay less' in r['cruise|60+']['status']   # under4: little more, much dearer
  assert r['cruise|30-45']['winner'] == 'set' and 'saves more fuel elsewhere' in r['cruise|30-45']['status']
  assert r['_budget']['left_min_per_hr'] < 1.0
  # with the full budget both get their step
  r = el.judge(el.samples([drive]), situation_miles_per_hour=share, budget_min_per_hr=4.0)
  assert r['cruise|30-45']['winner'] == 'under2'


def test_a_step_below_the_minute_floor_is_skipped_even_with_budget_left():
  rng = random.Random(15)
  # 'under2' saves 0.15 gal/100 mi but costs 12 min/100 mi: 0.0125 gal per extra minute, under the floor
  drive = (tagged('cruise|45-60|set', runs(rng, 20, 0.030, seconds_per_mile=70.0)) +
           tagged('cruise|45-60|under2', runs(rng, 20, 0.0285, seconds_per_mile=77.2)) +
           tagged('cruise|45-60|under4', runs(rng, 20, 0.0300, seconds_per_mile=85.0)))
  r = el.judge(el.samples([drive]), situation_miles_per_hour={'cruise|45-60': 20.0}, budget_min_per_hr=40.0)
  entry = r['cruise|45-60']
  assert el.MIN_GAL_PER_MINUTE == 0.02
  assert entry['winner'] == 'set' and 'not worth the time' in entry['status']
  assert r['_budget']['left_min_per_hr'] == 40.0



def test_road_tests_count_from_the_judge_date_but_corrections_use_every_drive(tmp_path, monkeypatch):
  rng = random.Random(16)
  (tmp_path / 'drives').mkdir()
  old = tagged('uphill|30-45|hold', runs(rng, 20, 0.030)) + tagged('uphill|30-45|sag3', runs(rng, 20, 0.020))
  new = tagged('uphill|30-45|hold', runs(rng, 20, 0.030)) + tagged('uphill|30-45|sag3', runs(rng, 20, 0.031))
  (tmp_path / 'drives' / 'old.json').write_text(json.dumps({'drive': '20261114-120000-1', 'stretches': old}))
  (tmp_path / 'drives' / 'new.json').write_text(json.dumps({'drive': f'{el.JUDGE_FROM}-120000-2', 'stretches': new}))
  fitted = []
  monkeypatch.setattr(el, 'fit_corrections', lambda drives: fitted.append(len(drives)) or {'climb': 0.0, 'speed': 0.0})
  doc = el.update(tmp_path)
  assert fitted == [2]                                    # corrections: both drives
  assert doc['results']['uphill|30-45']['measured']['sag3'] == round(20 * 400 / 1609.344, 2)   # judging: from JUDGE_FROM only
  assert doc['winners'].get('uphill|30-45') != 'sag3'     # the old, redefined runs do not make it win
  assert len(el.load_drives(tmp_path, since=el.JUDGE_FROM)) == 1 and len(el.load_drives(tmp_path)) == 2


def test_a_variant_driven_with_the_heat_off_is_not_flattered():
  rng = random.Random(21)
  # the heat costs the same for both; 'under2' just happened to run mostly with it off
  def climate(band, rs):
    return [r | {'climate': band} for r in rs]
  set_runs = climate('off', runs(rng, 6, 0.025)) + climate('heat', runs(rng, 30, 0.035))
  under_runs = climate('off', runs(rng, 30, 0.025)) + climate('heat', runs(rng, 6, 0.035))
  data = el.samples([tagged('cruise|45-60|set', set_runs) + tagged('cruise|45-60|under2', under_runs) +
                     tagged('cruise|45-60|under4', set_runs)])
  assert el.gal_per_100mi(data[('cruise', '45-60', 'under2')]) < 0.9 * el.gal_per_100mi(data[('cruise', '45-60', 'set')])  # pooled
  r = el.judge(data)['cruise|45-60']
  assert r['winner'] in (None, 'set') and set(r['curves']) == {'straight/off', 'straight/heat'}


def test_stretches_from_before_the_climate_band_keep_their_curve_class():
  assert el._run_curve(el.Run(0.1, 400.0, 30.0, 'curvy')) == 'curvy'
  assert el._run_curve(el.Run(0.1, 400.0, 30.0, 'curvy', 'cold', 'heat')) == 'curvy/heat'


def test_fuel_from_outside_the_tests_speed_band_is_not_counted():
  # stored before tests ended on leaving their band: a "60+" follow stretch driven at 56 mph belongs to 45-60
  mph = 0.44704
  inside = dict(stretch('follow|60+|a0.90', 0.004), v0=66 * mph, v1=68 * mph)
  edge = dict(stretch('follow|60+|a0.90', 0.004), v0=59 * mph, v1=59 * mph)       # within the 1.5 mph margin
  slowed = dict(stretch('follow|60+|a0.90', 0.004), v0=57 * mph, v1=55 * mph)
  untimed = stretch('follow|60+|a0.90', 0.004)                                    # older stretches without speeds stay
  launch = dict(stretch('launch|launch|gentle', 0.004), v0=0.0, v1=9.0)          # not keyed by its own speed
  data = el.samples([[inside, edge, slowed, untimed]])
  assert len(data[('follow', '60+', 'a0.90')]) == 3
  assert not el._stretch_outside_band('launch', 'launch', launch)
