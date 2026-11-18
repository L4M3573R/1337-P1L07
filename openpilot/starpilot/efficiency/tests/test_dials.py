"""Self-tuning fuel dials: settings that move on their own toward what saves fuel, inside the driver's limits."""
import random

import pytest

from openpilot.starpilot.efficiency import dials as dl
from openpilot.starpilot.efficiency import experiment_learn as el
from openpilot.starpilot.efficiency.experiment import DialExperimenter, Frame, normal_accel
from openpilot.starpilot.efficiency.tests.test_experiment_learn import runs, tagged

MPH = dl.MPH


# ---------- settings ----------
def test_every_situation_starts_at_normal_driving_and_round_trips():
  for category in dl.DIALS:
    start = dl.start_values(category)
    v = dl.encode(category, start)
    assert dl.decode(category, v) == start
    assert not dl.costs_time(category, start)   # normal driving costs no time
    assert dl.effect(category, start, v_ego=25.0, v_target=25.0, normal=0.8, below_target_mps=0.0) in (
      dl.Effect(), dl.Effect(pulse_glide_off=True))   # and changes nothing (Pulse and Glide simply stays off)


def test_old_named_variants_and_out_of_range_values_are_not_dial_settings():
  assert dl.decode('cruise', 'under4') is None
  assert dl.decode('cruise', 'u4.00') is None          # the driver's limit: 2 mph under at most
  assert dl.decode('hold', 'd1.00') is None            # every dial must be written
  assert dl.decode('pulse_glide', 'b6.0p1.00g0.00') is None   # Pulse and Glide band never over 5 mph


def test_the_driver_limits_are_the_dial_ranges():
  assert dl.DIALS['cruise'][0].high == 2.0 and dl.DIALS['uphill'][0].high == 2.0
  assert {d.key: d.high for d in dl.DIALS['hold']}['d'] == 2.0
  assert dl.DIALS['downhill'][0].high == 3.0
  assert {d.key: d.high for d in dl.DIALS['pulse_glide']}['b'] == 5.0


def test_probes_move_one_dial_one_step_and_stay_in_range():
  sit = dl.new_situation('hold')
  probes = {v: (k, d) for v, k, d in sit.probes('hold')}
  assert probes == {'d1.00k1.00': ('d', 1), 'd0.00k0.75': ('k', -1)}   # d cannot go under 0, k not over 1


# ---------- what each setting does ----------
def test_steady_driving_lets_the_speed_dip_then_climbs_back_gently():
  v = {'d': 1.0, 'k': 0.5}
  within = dl.effect('hold', v, v_ego=0.0, v_target=0.0, normal=0.8, below_target_mps=0.5 * MPH)
  assert within.accel_max == 0.0   # off the throttle until the dip allowance is used
  past = dl.effect('hold', v, v_ego=0.0, v_target=0.0, normal=0.8, below_target_mps=1.5 * MPH)
  assert past.accel_max == pytest.approx(0.4)   # then back up at half the normal rate


def test_acceleration_eases_off_before_the_set_speed():
  v = {'a': 1.0, 'e': 4.0}
  far = dl.effect('accelerate', v, v_ego=20, v_target=25, normal=0.8, below_target_mps=10 * MPH)
  near = dl.effect('accelerate', v, v_ego=24, v_target=25, normal=0.8, below_target_mps=1 * MPH)
  assert far.accel_max is None and near.accel_max < 0.8 * 0.6


def test_pulse_and_glide_band_pulse_and_glide_regen_are_each_a_dial():
  off = dl.effect('pulse_glide', {'b': 0.0, 'p': 1.0, 'g': 0.0}, v_ego=20, v_target=20, normal=0.6, below_target_mps=0)
  assert off.pulse_glide_off and off.pulse_glide is None
  on = dl.effect('pulse_glide', {'b': 3.0, 'p': 1.2, 'g': 0.2}, v_ego=20, v_target=20, normal=0.6, below_target_mps=0)
  assert on.pulse_glide == pytest.approx((3.0 * MPH, 1.2, 0.2))


def test_the_pulse_and_glide_owner_uses_the_learned_band_pulse_and_regen():
  from openpilot.starpilot.controllers.coast import CoastOwner, PulseGlideTuning, glide_accel, pulse_accel
  def first(tuning, speed):
    owner = CoastOwner()
    owner.pulse_glide, owner.tuning = True, tuning
    return owner.sample(admitted=True, active=True, gas=False, brake=False, speed_mps=speed, target_mps=20.0,
                        delta_mps=2.0, pitch=0.0, lead_relevant=False, stop_context=False)
  plain = first(None, 18.5)
  strong = first(PulseGlideTuning(2.0, 1.3, 0.0), 18.5)
  assert plain.pulsing and plain.accel_target == pytest.approx(pulse_accel(18.5))
  assert strong.accel_target == pytest.approx(min(1.3 * pulse_accel(18.5), 1.5 * 1.5))
  regen = first(PulseGlideTuning(2.0, 1.0, 0.3), 19.95)   # at the top of the band: gliding
  assert regen.pulse_coasting and regen.accel_target == pytest.approx(min(glide_accel(19.95, 0.0) - 0.3, 0.05))


def test_time_counts_for_slower_settings_only():
  slow = dl.episode_time_s('launch', {'a': 0.5}, v_ego=0.0, v_target=8.0, normal_at=normal_accel)
  assert slow > 0 and dl.episode_time_s('launch', {'a': 1.0}, v_ego=0.0, v_target=8.0, normal_at=normal_accel) == 0
  assert dl.ongoing_time_s('cruise', {'u': 2.0}, 27.0, 1.0) > 0
  assert dl.ongoing_time_s('cruise', {'u': 0.0}, 27.0, 1.0) == 0


# ---------- the experimenter ----------
def test_half_the_episodes_drive_the_center_and_half_probe_it():
  exp = DialExperimenter(rng=random.Random(3))
  draws = [exp.draw('cruise', '60+') for _ in range(2000)]
  assert 0.45 < draws.count('u0.00') / len(draws) < 0.55
  assert set(draws) == {'u0.00', 'u1.00'}


def test_out_of_minutes_it_only_draws_settings_that_cost_none():
  exp = DialExperimenter(rng=random.Random(4), state={'cruise|60+': dl.Situation({'u': 1.0}, {'u': 1.0}, 0.5)})
  exp.spent_s = 1e6
  draws = {exp.draw('cruise', '60+') for _ in range(200)}
  assert draws == {'u0.00'}   # the 1 mph center and the 2 mph probe both cost time: normal driving instead


def test_a_slower_setting_the_driver_pressed_through_is_counted_and_then_left_out():
  exp = DialExperimenter(rng=random.Random(5))
  exp.rng.random = lambda: 0.99   # always probe
  exp.cruise_turn = 1             # the cruise-speed experiment (Pulse and Glide comes first in the rotation)
  frames = [Frame(t * 0.1, 27.0, 27.0, True) for t in range(30)]
  for f in frames:
    exp.sample(f)
  assert exp.episode is not None and exp.episode.variant == 'u1.00'
  exp.sample(Frame(3.1, 27.0, 27.0, True, gas=True))
  assert exp.vetoes['cruise|60+|u1.00'] == el.VETO_LIMIT   # one press: too slow there, never offered again


def test_one_gas_press_on_the_center_steps_it_faster_for_the_rest_of_the_drive():
  # The driver, 2026-11-17: a gas press on a too-gentle setting means try a slightly faster one.
  exp = DialExperimenter(rng=random.Random(5), state={'cruise|60+': dl.Situation({'u': 1.0}, {'u': 0.5})})
  exp.rng.random = lambda: 0.0    # always drive the center
  exp.cruise_turn = 1
  for t in range(30):
    exp.sample(Frame(t * 0.1, 27.0, 27.0, True))
  assert exp.episode is not None and exp.episode.variant == 'u1.00'
  exp.sample(Frame(3.1, 27.0, 27.0, True, gas=True))
  assert exp.state['cruise|60+'].center == {'u': 0.5}
  assert exp.draw('cruise', '60+') == 'u0.50'


def test_a_dial_episode_shapes_the_plan_and_tags_the_ledger():
  exp = DialExperimenter(rng=random.Random(6), state={'cruise|60+': dl.Situation({'u': 2.0}, {'u': 1.0})})
  exp.rng.random = lambda: 0.0   # the center
  exp.cruise_turn = 1
  plans = [exp.sample(Frame(t * 0.05, 27.0, 27.0, True)) for t in range(400)]
  assert plans[-1].tag == 'cruise|60+|u2.00'
  assert plans[-1].speed_ceiling_mps == pytest.approx(27.0 - 2.0 * MPH, abs=0.05)   # ramped in, 2 mph under
  assert exp.spent_s > 0


# ---------- the judge ----------
def _data(rng, center, probes, *, category='cruise', context='60+', n=20, base=0.040, spm=55.0):
  drive = tagged(f'{category}|{context}|{center}', runs(rng, n, base, seconds_per_mile=spm))
  for variant, (gal, sec) in probes.items():
    drive += tagged(f'{category}|{context}|{variant}', runs(rng, n, gal, seconds_per_mile=sec))
  return el.samples([drive])


def test_a_probe_that_saves_fuel_becomes_the_new_center():
  data = _data(random.Random(10), 'u0.00', {'u1.00': (0.036, 55.6)})
  state, report = el.judge_dials(data, {}, {}, situation_miles_per_hour={'cruise|60+': 40.0})
  assert state['cruise|60+'].center == {'u': 1.0} and state['cruise|60+'].moves == 1
  assert state['cruise|60+'].cost_min_per_100mi > 0 and 'moved to u1.00' in report['cruise|60+']['status']


def test_both_sides_losing_halves_the_step_so_it_closes_in():
  sit = dl.Situation({'u': 1.0}, {'u': 1.0})
  data = _data(random.Random(11), 'u1.00', {'u2.00': (0.0405, 56.0), 'u0.00': (0.0402, 55.0)})
  state, _ = el.judge_dials(data, {'cruise|60+': sit}, {})
  assert state['cruise|60+'].center == {'u': 1.0} and state['cruise|60+'].step['u'] == 0.5


def test_a_move_that_costs_time_waits_when_the_budget_is_used():
  data = _data(random.Random(12), 'u0.00', {'u1.00': (0.036, 57.0)})
  busy = {'uphill|60+': dl.Situation({'s': 2.0}, {'s': 1.0}, cost_min_per_100mi=10.0)}
  state, report = el.judge_dials(data, busy, {}, situation_miles_per_hour={'cruise|60+': 40.0, 'uphill|60+': 40.0},
                                 budget_min_per_hr=4.0)
  assert state.get('cruise|60+') is None or state['cruise|60+'].center == {'u': 0.0}
  assert 'budget is used' in report['cruise|60+']['status']


def test_a_free_saving_is_always_taken():
  data = _data(random.Random(13), 'b0.0p1.00g0.00', {'b2.0p1.00g0.00': (0.035, 55.0)}, category='pulse_glide',
               context='45-60')
  state, _ = el.judge_dials(data, {}, {}, budget_min_per_hr=0.0)
  assert state['pulse_glide|45-60'].center['b'] == 2.0


def test_a_center_the_driver_keeps_pressing_through_steps_back_toward_normal():
  sit = dl.Situation({'u': 2.0}, {'u': 1.0})
  state, report = el.judge_dials({}, {'cruise|60+': sit}, {'cruise|60+|u2.00': 2})
  assert state['cruise|60+'].center == {'u': 1.0} and 'vetoed' in report['cruise|60+']['status']


def test_the_state_survives_the_file_and_a_new_epoch_starts_over(tmp_path):
  path = tmp_path / 'experiments.json'
  import json
  sit = dl.Situation({'d': 1.0, 'k': 0.75}, {'d': 0.5, 'k': 0.25}, 0.4, 2)
  path.write_text(json.dumps({'dials': dl.state_doc({'hold|60+': sit})}))
  loaded = dl.load_state(path)
  assert loaded['hold|60+'].center == sit.center and loaded['hold|60+'].step == sit.step and loaded['hold|60+'].moves == 2
  path.write_text(json.dumps({'dials': {'_epoch': 0, 'hold|60+': sit.as_doc()}}))
  assert dl.load_state(path) == {}


# ---------- engines ----------
def test_the_dials_start_in_shadow_and_only_drive_when_switched_on(tmp_path):
  import json
  assert el.engine(tmp_path) == 'shadow'
  (tmp_path / 'experiment.json').write_text(json.dumps({'mode': 'on', 'engine': 'dials'}))
  assert el.engine(tmp_path) == 'dials'


def test_in_shadow_the_fixed_variants_drive_and_the_dials_never_touch_the_plan(tmp_path):
  from openpilot.starpilot.efficiency.experiment_host import ExperimentHost
  host = ExperimentHost(root=tmp_path, status_path=tmp_path / 'status.json')
  host._new_drive(1)
  assert type(host.exp).__name__ == 'Experimenter' and isinstance(host.shadow, DialExperimenter)
  import json
  (tmp_path / 'experiment.json').write_text(json.dumps({'engine': 'dials'}))
  host._new_drive(2)
  assert isinstance(host.exp, DialExperimenter) and host.shadow is None


def test_the_dials_also_find_out_whether_pulse_and_glide_pays_on_the_highway():
  exp = DialExperimenter(rng=random.Random(7))
  exp.cruise_turn = 0
  assert exp._enter(Frame(0.0, 30.0, 30.0, True, grade=0.0)) == ('pulse_glide', '60+')
  from openpilot.starpilot.efficiency.experiment import Experimenter
  assert Experimenter()._enter(Frame(0.0, 30.0, 30.0, True, grade=0.0)) != ('pulse_glide', '60+')


def test_with_pulse_and_glide_off_only_its_band_is_probed():
  off = {v for v, _, _ in dl.new_situation('pulse_glide').probes('pulse_glide')}
  assert off == {'b2.0p1.00g0.00'}
  on = {k for _, k, _ in dl.Situation({'b': 2.0, 'p': 1.0, 'g': 0.0}, {'b': 2.0, 'p': 0.2, 'g': 0.2}).probes('pulse_glide')}
  assert on == {'b', 'p', 'g'}


def test_a_saving_below_the_fuel_per_minute_floor_is_not_worth_the_time():
  # 2 mph under saves 4 % but costs 6 min/100 mi: 0.016 gal/100 mi over 6 min/100 mi is far under 0.02 gal per minute
  data = _data(random.Random(14), 'u0.00', {'u1.00': (0.0384, 61.0)})
  state, report = el.judge_dials(data, {}, {}, situation_miles_per_hour={'cruise|60+': 40.0})
  assert state['cruise|60+'].center == {'u': 0.0}
  assert 'not worth the time' in report['cruise|60+']['probes']['u1.00']


def test_with_two_winners_the_better_fuel_per_minute_is_taken():
  # hold: a deeper dip saves a bit more fuel but costs much more time; the gentler recovery saves less, for no time
  data = _data(random.Random(15), 'd0.00k1.00', {'d1.00k1.00': (0.0360, 56.5), 'd0.00k0.75': (0.0372, 55.0)},
               category='hold')
  state, report = el.judge_dials(data, {}, {}, situation_miles_per_hour={'hold|60+': 40.0})
  assert state['hold|60+'].center == {'d': 0.0, 'k': 0.75} and 'no extra time' in report['hold|60+']['status']
