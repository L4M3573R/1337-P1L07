"""Self-learning fuel efficiency: judging experiment variants from measured fuel."""
import json
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
  assert all(m >= 400 for _, m in data[('launch', 'launch', 'gentle')])


def test_update_writes_winners(tmp_path):
  rng = random.Random(6)
  (tmp_path / 'drives').mkdir()
  drive = steady_drive(rng, 30, 'downhill|30-45|hold', 0.02) + steady_drive(rng, 30, 'downhill|30-45|carry2', 0.016)
  (tmp_path / 'drives' / 'd1.json').write_text(json.dumps({'stretches': drive}))
  doc = el.update(tmp_path)
  assert doc['winners'] == {'downhill|30-45': 'carry2'}
  assert json.loads((tmp_path / 'experiments.json').read_text())['winners'] == {'downhill|30-45': 'carry2'}
  assert any('downhill' in line for line in el.report_lines(doc))
