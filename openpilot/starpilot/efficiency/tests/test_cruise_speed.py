"""Cruising picks its speed from the car's measured MPG by speed (speed_curve.py), inside the driver's limits: at most
2 mph under the set speed, over it only as far as the driver allows, the time budget, and the driver's vetoes."""
import json
import random
from types import SimpleNamespace as NS

import pytest

from openpilot.starpilot.efficiency import dials as dl
from openpilot.starpilot.efficiency import experiment as ex
from openpilot.starpilot.efficiency import experiment_learn as el
from openpilot.starpilot.efficiency.experiment import DialExperimenter, Frame
from openpilot.starpilot.efficiency.experiment_host import ExperimentHost
from openpilot.starpilot.efficiency.tests.test_experiment_host import Clock, Owner, sm

MPH = dl.MPH
SET = 65 * MPH


def speeds(spec):
  return {str(mph): {'mi': mi, 'est': est, 'low': round(est * 0.98, 1), 'high': round(est * 1.02, 1), 'ok': mi >= 10}
          for mph, (mi, est) in spec.items()}


def experimenter(table, seed=1, **kw):
  doc = table if 'flat' in table else {'flat': [table]}   # one level: all driving
  exp = DialExperimenter(rng=random.Random(seed), speeds=doc, **kw)
  exp.frame = Frame(0.0, SET, SET, True)
  return exp


def draws(exp, n=400):
  out = []
  for _ in range(n):
    v = exp.draw('cruise', '60+')
    exp.tried[f'cruise|60+|{v}'] = exp.tried.get(f'cruise|60+|{v}', 0) + 1
    out.append(v)
  return out


def test_the_cruise_dial_reaches_over_the_set_speed_but_is_never_probed_there():
  assert dl.decode('cruise', 'u-2.00') == {'u': -2.0} and dl.decode('cruise', 'u-2.50') is None
  assert dl.effect('cruise', {'u': -2.0}, v_ego=SET, v_target=SET, normal=0.5, below_target_mps=0.0).target_offset_mps == pytest.approx(2 * MPH)
  assert not dl.costs_time('cruise', {'u': -2.0}) and dl.ongoing_time_s('cruise', {'u': -2.0}, SET, 1.0) == 0
  assert dl.faster_than_normal('cruise', {'u': -1.0}) and not dl.faster_than_normal('cruise', {'u': 0.0})
  assert all(dl.decode('cruise', v)['u'] >= 0 for v, _, _ in dl.new_situation('cruise').probes('cruise'))
  plain = DialExperimenter(rng=random.Random(2))      # no measured table: the plain dial, as before
  assert {plain.draw('cruise', '60+') for _ in range(300)} == {'u0.00', 'u1.00'}


def test_with_nothing_measured_it_drives_the_speeds_under_the_set_speed_to_learn_them():
  got = draws(experimenter({}))
  assert set(got) == {'u0.00', 'u1.00', 'u2.00'} and 0.4 < got.count('u0.00') / len(got) < 0.6
  assert abs(got.count('u1.00') - got.count('u2.00')) <= 1      # each in turn
  got = draws(experimenter({}, cruise_over_mph=2.0))
  assert set(got) == {'u0.00', 'u1.00', 'u2.00', 'u-1.00', 'u-2.00'}


def test_once_the_window_is_measured_and_flat_it_just_drives_the_set_speed():
  flat = speeds({63: (20, 50.0), 64: (20, 50.4), 65: (20, 50.0), 66: (20, 49.6), 67: (20, 49.0)})
  assert set(draws(experimenter(flat, cruise_over_mph=2.0))) == {'u0.00'}


def test_a_measured_better_speed_over_the_set_speed_is_driven_only_when_the_driver_allows_it():
  table = speeds({63: (20, 50.0), 64: (20, 50.0), 65: (20, 50.0), 66: (20, 53.0), 67: (20, 56.0)})
  assert set(draws(experimenter(table))) == {'u0.00'}
  exp = experimenter(table, cruise_over_mph=2.0)
  assert set(draws(exp, 50)) == {'u-2.00'}
  exp.cruise_turn = 1   # the cruise-speed episode's turn
  plans = [exp.sample(Frame(t * 0.05, SET, SET, True)) for t in range(400)]
  assert plans[-1].tag == 'cruise|60+|u-2.00' and plans[-1].speed_ceiling_mps is None
  assert plans[-1].speed_bonus_mps == pytest.approx(2 * MPH, abs=0.05) and exp.spent_s == 0
  # the gas pedal does not veto a faster speed; turning the set speed down does, and for everything as fast
  exp.sample(Frame(20.05, SET, SET, True, gas=True))
  assert 'cruise|60+|u-2.00' not in exp.vetoes
  exp.episode = ex.Episode('cruise', '60+', 'u-2.00', 20.1, exp.distance_m, 99)
  exp.ep_set_mps = SET
  plan = exp.sample(Frame(20.1, SET, SET - 1 * MPH, True))
  assert exp.vetoes['cruise|60+|u-2.00'] == ex.VETO_LIMIT and plan.tag is None
  exp.frame = Frame(0.0, SET, SET, True)
  assert set(draws(exp, 50)) == {'u-1.00'}        # 66 is still better than 65 and was not refused
  exp.vetoes['cruise|60+|u-1.00'] = ex.VETO_LIMIT
  assert set(draws(exp, 50)) == {'u0.00'}


def test_a_measured_better_slower_speed_needs_minutes_and_no_veto():
  table = speeds({63: (20, 56.0), 64: (20, 53.0), 65: (20, 50.0)})
  assert set(draws(experimenter(table))) == {'u2.00'}
  broke = experimenter(table)
  broke.spent_s = 1e6
  assert set(draws(broke)) == {'u0.00'}
  refused = experimenter(table, vetoes={'cruise|60+|u1.00': ex.VETO_LIMIT})    # 1 under was too slow: so is 2
  assert set(draws(refused)) == {'u0.00'}
  only_two = experimenter(table, vetoes={'cruise|60+|u2.00': ex.VETO_LIMIT})
  assert set(draws(only_two)) == {'u1.00'}


def test_the_host_hands_the_table_and_the_drivers_limit_to_the_experimenter(tmp_path):
  table = speeds({65: (20, 50.0), 67: (20, 56.0)})
  (tmp_path / 'experiment.json').write_text(json.dumps({'mode': 'on', 'engine': 'dials', 'cruise_over_mph': 5}))
  (tmp_path / 'experiments.json').write_text(json.dumps({'speeds': {'flat': [table]}}))
  h = ExperimentHost(root=tmp_path, status_path=tmp_path / 'status.json', clock=Clock())
  h.sample(sm(), NS(owner=Owner()), drive=1, force_stop_forcing=False)
  assert h.exp.speeds == {'flat': [table]} and h.exp.cruise_over_mph == 2.0      # never more than 2 over, whatever the file says
  (tmp_path / 'experiment.json').write_text(json.dumps({'mode': 'on', 'engine': 'dials'}))
  (tmp_path / 'experiments.json').write_text(json.dumps({'speeds': {}}))
  h.sample(sm(), NS(owner=Owner()), drive=2, force_stop_forcing=False)
  assert h.exp.speeds == {} and h.exp.cruise_over_mph == 0.0         # not set: the set speed is the top
  (tmp_path / 'experiment.json').write_text(json.dumps({'mode': 'on', 'engine': 'dials', 'cruise_over_mph': True}))
  h.sample(sm(), NS(owner=Owner()), drive=3, force_stop_forcing=False)
  assert h.exp.cruise_over_mph == 0.0


def test_the_judge_stores_the_cars_speed_table_and_leaves_the_cruise_dial_at_normal(tmp_path):
  from openpilot.starpilot.efficiency.tests.test_speed_curve import drives
  (tmp_path / 'drives').mkdir()
  for n, d in enumerate(drives({63: 56.0, 65: 50.0})):
    (tmp_path / 'drives' / f'd{n}.json').write_text(json.dumps({'drive': f'2026111{8 + n % 2}-0{n}0000-1', 'stretches': d}))
  (tmp_path / 'experiment.json').write_text(json.dumps({'mode': 'on', 'engine': 'dials'}))
  (tmp_path / 'experiments.json').write_text(json.dumps({'dials': dl.state_doc({'cruise|60+': dl.Situation({'u': 1.0}, {'u': 0.5})})}))
  doc = el.update(tmp_path)
  flat = doc['speeds']['flat'][-1]                       # the last level: all driving
  assert flat['63']['ok'] and flat['65']['est'] == pytest.approx(50.0, abs=1.0)
  assert doc['speeds']['uphill'][-1] == {} and doc['speeds']['levels'] == ['all driving']   # no temperature recorded
  assert doc['dials']['cruise|60+']['center'] == {'u': 0.0}
  saved = json.loads((tmp_path / 'experiments.json').read_text())
  # 63 is measured clearly better: it drives 2 under, and 64 (never driven) the other half of the time to learn it
  assert set(draws(experimenter(saved['speeds']))) == {'u2.00', 'u1.00'}


def hill_draws(exp, category, n=300):
  out = []
  for _ in range(n):
    v = exp.draw(category, '60+')
    exp.tried[f'{category}|60+|{v}'] = exp.tried.get(f'{category}|60+|{v}', 0) + 1
    out.append(v)
  return set(out)


def test_hills_pick_from_their_own_grades_table_inside_the_drivers_limits():
  up = speeds({63: (20, 56.0), 64: (20, 53.0), 65: (20, 50.0), 66: (20, 60.0)})
  down = speeds({65: (20, 50.0), 66: (20, 50.5), 67: (20, 56.0), 68: (20, 57.0), 69: (20, 70.0), 63: (20, 70.0)})
  flat = speeds({63: (20, 50.0), 64: (20, 50.0), 65: (20, 50.0)})
  exp = experimenter({'flat': [flat], 'uphill': [up], 'downhill': [down]}, cruise_over_mph=2.0)
  assert hill_draws(exp, 'uphill') == {'s2.00'}            # never over the set speed uphill, whatever 66 measured
  assert hill_draws(exp, 'downhill') == {'c3.00'}          # at most 3 over downhill, never under
  # the hills' tables say nothing about the level road: nothing better there, and 66-67 (allowed here) still to learn
  assert hill_draws(exp, 'cruise') == {'u0.00', 'u-1.00', 'u-2.00'}
  exp.spent_s = 1e6
  assert hill_draws(exp, 'uphill') == {'s0.00'} and hill_draws(exp, 'downhill') == {'c3.00'}   # carrying costs no time
  unknown = experimenter({'flat': [flat], 'uphill': [{}], 'downhill': [{}]})
  assert hill_draws(unknown, 'uphill') == {'s0.00', 's1.00', 's2.00'}
  assert hill_draws(unknown, 'downhill') == {'c0.00', 'c1.00', 'c2.00', 'c3.00'}


def test_a_decision_is_made_inside_one_condition_and_each_condition_is_driven():
  # all driving says 63 is better; in the cold with the heater on (the current condition) the two measure the same
  everything = speeds({63: (40, 56.0), 64: (40, 53.0), 65: (40, 50.0)})
  cold_heat = speeds({63: (20, 50.0), 64: (20, 50.0), 65: (20, 50.0)})
  assert set(draws(experimenter({'flat': [cold_heat, everything]}))) == {'u0.00'}
  # this condition has not measured 63 and 64 yet: the pooled table decides, and this condition gets driven
  new = speeds({65: (20, 50.0)})
  assert set(draws(experimenter({'flat': [new, everything]}))) == {'u2.00', 'u1.00'}
  # nothing measured for it at all: still the pooled answer, and learning
  assert set(draws(experimenter({'flat': [{}, everything]}))) == {'u2.00', 'u1.00'}
