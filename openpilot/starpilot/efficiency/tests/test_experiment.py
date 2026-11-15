"""Self-learning fuel efficiency: the experimenter's situations, variants, limits and time budget."""
import math
import random

import pytest

from openpilot.starpilot.efficiency import experiment as ex
from openpilot.starpilot.efficiency.experiment import Experimenter, Frame

MPH = ex.MPH


def run(exp, frames):
  return [exp.sample(f) for f in frames]


def cruise(t, v=40 * MPH, v_set=40 * MPH, **kw):
  return Frame(t, v, v_set, True, **kw)


def test_every_variant_is_tried_before_any_winner():
  exp = Experimenter(rng=random.Random(1))
  seen = []
  for _ in range(3):
    v = exp.draw('cruise', '30-45')
    exp.tried[f'cruise|30-45|{v}'] = exp.tried.get(f'cruise|30-45|{v}', 0) + 1
    seen.append(v)
  assert sorted(seen) == sorted(ex.CATEGORIES['cruise'])


def test_learned_winner_is_used_mostly_but_others_still_tried():
  exp = Experimenter(winners={'uphill|30-45': 'sag3'}, rng=random.Random(2))
  draws = [exp.draw('uphill', '30-45') for _ in range(2000)]
  assert 0.75 < draws.count('sag3') / len(draws) < 0.85


def test_uphill_sag_lowers_the_target_gently_and_counts_time():
  exp = Experimenter(winners={'uphill|30-45': 'sag3'}, rng=random.Random(0))
  exp.rng.random = lambda: 0.99  # no exploring
  plans = run(exp, [cruise(i * 0.05, grade=0.04) for i in range(200)])
  assert all(p.tag == 'uphill|30-45|sag3' for p in plans)
  ceilings = [p.speed_ceiling_mps for p in plans if p.speed_ceiling_mps is not None]
  assert ceilings[0] > 40 * MPH - 0.05 and ceilings[-1] == pytest.approx(37 * MPH, abs=0.01)   # ramped, 3 mph under
  assert max(a - b for a, b in zip(ceilings, ceilings[1:], strict=False)) <= ex.RAMP_MPS2 * 0.05 + 1e-9
  assert exp.spent_s > 0


def test_downhill_carry_is_the_only_raise_and_stays_within_the_allowance():
  assert ex.DOWNHILL_CARRY_MPH == 3.0   # the driver's allowance (2026-11-15)
  exp = Experimenter(winners={'downhill|30-45': 'carry3'})
  exp.rng.random = lambda: 0.99
  plans = run(exp, [cruise(i * 0.05, grade=-0.04) for i in range(300)])
  assert plans[-1].speed_bonus_mps == pytest.approx(3 * MPH) and plans[-1].speed_ceiling_mps is None
  assert all(p.speed_bonus_mps <= 3 * MPH + 1e-9 for p in plans)


def test_pedals_end_the_episode_at_once():
  exp = Experimenter(winners={'uphill|30-45': 'sag3'})
  exp.rng.random = lambda: 0.99
  run(exp, [cruise(i * 0.05, grade=0.04) for i in range(100)])
  p = exp.sample(cruise(5.0, grade=0.04, brake=True))
  assert p.tag is None and p.speed_ceiling_mps is None and exp.offset_mps == 0.0
  p = exp.sample(cruise(5.05, grade=0.04, gas=True))
  assert p.tag is None


def test_launch_rates_and_end_at_20_mph():
  exp = Experimenter(winners={'launch|launch': 'gentle'})
  exp.rng.random = lambda: 0.99
  exp.sample(Frame(0.0, 0.0, 30 * MPH, True, standstill=True))
  p = exp.sample(Frame(0.05, 0.3, 30 * MPH, True))
  assert p.tag == 'launch|launch|gentle' and p.accel_max == pytest.approx(0.75 * ex.normal_accel(0.3))
  p = exp.sample(Frame(0.1, 21 * MPH, 30 * MPH, True))
  assert not (p.tag or '').startswith('launch')


def test_speeding_up_to_the_set_speed_is_learned_per_target_band():
  exp = Experimenter(winners={'accelerate|45-60': 'moderate'})
  exp.rng.random = lambda: 0.99
  p = exp.sample(cruise(0.0, v=30 * MPH, v_set=50 * MPH, a_ego=0.5))
  assert p.tag == 'accelerate|45-60|moderate' and p.accel_max == pytest.approx(0.9 * ex.normal_accel(30 * MPH))
  assert exp.spent_s > 0


def test_time_budget_stops_costly_variants():
  exp = Experimenter(winners={'uphill|30-45': 'sag3', 'cruise|30-45': 'under4'})
  exp.spent_s = 1000.0  # far over budget
  for _ in range(50):
    assert exp.draw('uphill', '30-45') == 'hold'
    assert exp.draw('cruise', '30-45') == 'set'
  assert exp.draw('downhill', '30-45') in ex.CATEGORIES['downhill']   # saves time: always allowed


def test_driver_pulse_glide_choice_is_respected():
  exp = Experimenter()
  exp.cruise_turn = 0
  p = exp.sample(cruise(0.0, v=50 * MPH, v_set=50 * MPH, pulse_glide_by_driver=True, grade=0.0))
  assert p.pulse_glide is None and not (p.tag or '').startswith('pulse_glide')


def test_following_and_stops_win_over_experiments():
  exp = Experimenter()
  p = exp.sample(cruise(0.0, grade=0.04, lead_gap_s=1.5))
  assert (p.tag or '').startswith('follow')
  p = exp.sample(cruise(0.1, grade=0.04, stop_ahead=True))
  assert p.tag is None and p.speed_ceiling_mps is None


def test_bad_input_never_raises():
  exp = Experimenter()
  assert exp.sample(Frame(float('nan'), None, 10.0, True)) == ex.Plan()   # type: ignore[arg-type]


def test_winners_file(tmp_path):
  path = tmp_path / 'experiments.json'
  assert ex.load_winners(path) == {}
  path.write_text('{"winners": {"uphill|30-45": "sag3", "bogus|x": "y", "cruise|45-60": "fast"}}')
  assert ex.load_winners(path) == {'uphill|30-45': 'sag3'}


def test_bands():
  assert ex.band(28 * MPH) == '25-30' and ex.band(50 * MPH) == '45-60' and ex.band(70 * MPH) == '60+'
  assert math.isinf(ex.SPEED_BANDS[-1][0])


def test_episodes_last_long_enough_to_measure():
  # a grade wobbling around the hill threshold must not flip the experiment every few frames
  exp = Experimenter()
  tags = []
  for i in range(400):   # 20 s
    grade = 0.03 if (i // 20) % 2 == 0 else 0.018   # in and out of 2.5 % every second, never below the 1.5 % exit
    tags.append(exp.sample(cruise(i * 0.05, grade=grade)).tag)
  assert len({t for t in tags if t}) == 1 and exp.episodes == 1
  # but a pedal ends it at once, dwell or not
  assert exp.sample(cruise(20.05, grade=0.03, brake=True)).tag is None


def test_a_slowdown_an_experiment_made_is_not_a_speed_up_to_test():
  exp = Experimenter(winners={'cruise|30-45': 'under4', 'pulse_glide|30-45': 'steady'})
  exp.rng.random = lambda: 0.99
  exp.cruise_turn = 1   # the cruise-speed experiment's turn
  t = 0.0
  for _ in range(400):   # 20 s at 4 under: the car has slowed to the lower target
    t += 0.05
    exp.sample(cruise(t, v=44 * MPH + min(exp.offset_mps, 0.0), v_set=44 * MPH, grade=0.0))
  assert exp.episode.tag() == 'cruise|30-45|under4'
  # the episode ends without a pedal (a brief stop signal): the offset ramps back, the car follows it
  assert exp.sample(cruise(t + 0.05, v=40 * MPH, v_set=44 * MPH, stop_ahead=True)).tag is None
  p = exp.sample(cruise(t + 0.1, v=40 * MPH, v_set=44 * MPH))
  assert not (p.tag or '').startswith('accelerate')
  # after a pedal the planner goes straight back to the set speed: that speed-up is real and may be tested
  exp.sample(cruise(t + 0.15, v=40 * MPH, v_set=44 * MPH, brake=True))
  p = exp.sample(cruise(t + 0.2, v=40 * MPH, v_set=44 * MPH, a_ego=0.5))
  assert (p.tag or '').startswith('accelerate')



def test_slower_variants_are_shares_of_normal_acceleration():
  assert ex.normal_accel(0.0) == pytest.approx(1.6) and ex.normal_accel(5.0) == pytest.approx(1.4)
  assert ex.normal_accel(60.0) == pytest.approx(0.6)
  # at 11 mph gentle is about 1 m/s2 now, not the 0.5 that was far too slow
  assert 1.0 < 0.75 * ex.normal_accel(11 * MPH) < 1.1


def test_pressing_on_during_a_slower_variant_vetoes_it_after_two_times():
  exp = Experimenter(winners={'accelerate|30-45': 'gentle'})
  exp.rng.random = lambda: 0.99
  for drive in range(2):
    exp.episode = None
    p = exp.sample(cruise(drive * 100.0, v=20 * MPH, v_set=40 * MPH, a_ego=0.5))
    assert p.tag == 'accelerate|30-45|gentle'
    exp.sample(cruise(drive * 100.0 + 0.05, v=20 * MPH, v_set=40 * MPH, gas=True))
    exp.sample(cruise(drive * 100.0 + 0.1, v=20 * MPH, v_set=40 * MPH, gas=True))   # one veto per episode
  assert exp.vetoes == {'accelerate|30-45|gentle': 2}
  assert all(exp.draw('accelerate', '30-45') != 'gentle' for _ in range(200))   # never offered there again


def test_vetoes_file(tmp_path):
  path = tmp_path / 'vetoes.json'
  assert ex.load_vetoes(path) == {}
  ex.save_vetoes(path, {'accelerate|30-45|gentle': 2})
  assert ex.load_vetoes(path) == {'accelerate|30-45|gentle': 2}



def test_no_speed_up_test_while_still_slowing():
  exp = Experimenter()
  p = exp.sample(cruise(0.0, v=15 * MPH, v_set=40 * MPH, a_ego=-1.0))   # slowing behind a car or into a turn
  assert not (p.tag or '').startswith('accelerate')
  p = exp.sample(cruise(0.05, v=15 * MPH, v_set=40 * MPH, a_ego=0.6))
  assert (p.tag or '').startswith('accelerate')



def test_no_experiment_with_a_turn_signal_on():
  exp = Experimenter()
  assert exp.sample(cruise(0.0, grade=0.04)).tag
  p = exp.sample(cruise(0.05, grade=0.04, blinker=True))
  assert p.tag is None and p.speed_ceiling_mps is None


def _hold_episode(variant):
  """An experimenter whose next level-cruising episode is 'hold' with the given variant."""
  for turn in range(6):
    exp = Experimenter()
    exp.cruise_turn = turn
    exp.draw = lambda category, context: variant if category == 'hold' else ex.CATEGORIES[category][0]
    p = exp.sample(cruise(0.0, grade=0.0))
    if (p.tag or '').startswith('hold|'):
      return exp, p
  raise AssertionError('level cruising never rotated to the hold experiment')


def test_hold_flat_holds_the_throttle_until_a_mile_an_hour_under():
  exp, p = _hold_episode('flat')
  assert p.tag == 'hold|30-45|flat' and p.accel_max == 0.0 and p.speed_ceiling_mps is None
  spent = exp.spent_s
  p = exp.sample(cruise(0.05, v=40 * MPH - 0.5 * MPH, grade=0.0))
  assert p.accel_max == 0.0 and exp.spent_s > spent   # sagging inside the band: still flat, and it costs time
  p = exp.sample(cruise(0.08, v=40 * MPH - 1.5 * MPH, grade=0.0))
  assert p.accel_max == 0.0                           # still inside the 2 mph band
  p = exp.sample(cruise(0.1, v=40 * MPH - 2.5 * MPH, grade=0.0))
  assert p.accel_max is None                          # past the band: the planner wins the speed back
  _, normal = _hold_episode('normal')
  assert normal.tag == 'hold|30-45|normal' and normal.accel_max is None


def test_hold_flat_is_a_slower_variant_inside_the_time_budget():
  assert ('hold', 'flat') in ex.COSTS_TIME and 'flat' in ex.SLOWER_VARIANTS
  exp = Experimenter(winners={'hold|30-45': 'flat'})
  exp.spent_s = 1000.0
  assert all(exp.draw('hold', '30-45') == 'normal' for _ in range(50))


def test_hold_flat_runs_at_highway_speed_where_pulse_and_glide_is_off():
  for turn in range(6):
    exp = Experimenter()
    exp.cruise_turn = turn
    exp.draw = lambda category, context: 'flat' if category == 'hold' else ex.CATEGORIES[category][0]
    p = exp.sample(cruise(0.0, v=65 * MPH, v_set=65 * MPH, grade=0.0))
    assert not (p.tag or '').startswith('pulse_glide')   # above 60 mph Pulse and Glide is never offered
    if p.tag == 'hold|60+|flat':
      assert p.accel_max == 0.0
      return
  raise AssertionError('65 mph cruising never rotated to holding flat')
