"""The fuel learner rebuild (2026-11-25): a gas press only vetoes a setting that was holding the car back, the stop,
speed-up, following and hold situations are learned once for every speed, stops start early enough to coast into, a
new 'approach' situation lifts off early behind a slower car, and stops and approaches are judged on the measured
energy instead of fuel per mile."""
import json
import math

import pytest

from openpilot.starpilot.efficiency import dials as dl
from openpilot.starpilot.efficiency import experiment as ex
from openpilot.starpilot.efficiency import experiment_learn as el
from openpilot.starpilot.efficiency.experiment import DialExperimenter, Experimenter, Frame

MPH = ex.MPH


def cruise(t, v, v_set=None, **kw):
  return Frame(t, v, v if v_set is None else v_set, True, **({'grade': 0.0} | kw))


def _stopping(v, distance):
  exp = Experimenter()
  exp.draw = lambda category, context: 'regen' if category == 'stop' else ex.CATEGORIES[category][0]
  p = exp.sample(cruise(0.0, v, stop_distance_m=distance))
  return exp, p


# ---------- vetoes ----------
def test_gas_during_a_phantom_stop_the_setting_was_not_braking_for_is_no_veto():
  # 2026-11-23 17:31: a "stopped car" 107 m out at 31 mph that was not there; the planner braked, the driver pressed on
  exp, p = _stopping(13.9, 101.0)
  assert p.tag == 'stop|all|regen' and exp.limiting_t is None   # its steady slowdown has not started yet
  exp.sample(cruise(0.05, 13.5, stop_distance_m=99.0, gas=True))
  assert exp.vetoes == {}


def test_gas_while_the_stop_setting_is_slowing_the_car_is_a_veto():
  exp, _ = _stopping(15.0, 150.0)
  exp.sample(cruise(0.05, 15.0, stop_distance_m=80.0))   # sqrt(2 * 1.0 * 80) = 12.6 m/s < 15: it is slowing the car
  assert exp.limiting_t == pytest.approx(0.05)
  exp.sample(cruise(0.10, 14.9, stop_distance_m=79.0, gas=True))
  assert exp.vetoes == {'stop|all|regen': 1}


def test_a_press_long_after_the_setting_let_go_is_no_veto():
  exp, _ = _stopping(15.0, 150.0)
  exp.sample(cruise(0.05, 15.0, stop_distance_m=80.0))
  exp.sample(cruise(0.05 + ex.VETO_WINDOW_S + 0.5, 12.0, stop_distance_m=60.0, gas=True))
  assert exp.vetoes == {}


def test_a_lower_target_holds_the_car_back_only_near_that_target():
  exp = Experimenter()
  exp.episode = ex.Episode('cruise', '45-60', 'under2', 0.0, 0.0, 1)
  set_mps = 50 * MPH
  exp._note_limiting(cruise(0.0, 40 * MPH, set_mps), set_mps - 2 * MPH, None, None)   # far under it: not held by it
  assert exp.limiting_t is None
  exp._note_limiting(cruise(1.0, 48 * MPH, set_mps), set_mps - 2 * MPH, None, None)   # cruising at it: held
  assert exp.limiting_t == 1.0


def test_epoch_2_vetoes_are_dropped_except_set_speed_turned_down(tmp_path):
  path = tmp_path / 'vetoes.json'
  path.write_text(json.dumps({'version': 1, 'epoch': 2, 'vetoes': {
    'stop|25-30|r1.50': 2, 'follow|60+|a0.80': 2, 'accelerate|30-45|a0.85e0.0': 2, 'cruise|45-60|u-1.00': 2}}))
  assert ex.load_vetoes(path) == {'cruise|45-60|u-1.00': 2}   # the one that never depended on a gas press
  ex.save_vetoes(path, {'stop|all|r1.50': 2})
  assert ex.load_vetoes(path) == {'stop|all|r1.50': 2}


# ---------- pooled situations ----------
def test_per_band_states_are_dropped_for_pooled_situations(tmp_path):
  path = tmp_path / 'experiments.json'
  old = {'_epoch': dl.DIALS_EPOCH, 'stop|25-30': {'center': {'r': 2.0}, 'step': {'r': 0.1}},
         'pulse_glide|60+': {'center': {'b': 0.0, 'p': 1.0, 'g': 0.0}, 'step': {'b': 0.5, 'p': 0.2, 'g': 0.2}}}
  path.write_text(json.dumps({'dials': old}))
  state = dl.load_state(path)
  assert 'stop|25-30' not in state and 'pulse_glide|60+' in state   # Pulse and Glide still learns per speed band
  assert dl.new_situation('stop').step == {'r': 0.5}


def test_old_per_band_runs_join_the_pooled_situation():
  rows = el.samples([[{'exp': 'hold|60+|d0.00k1.00', 'obd_gal': 0.004, 'm': 250.0},
                      {'exp': 'hold|30-45|d0.00k1.00', 'obd_gal': 0.003, 'm': 250.0}]])
  assert len(rows[('hold', 'all', 'd0.00k1.00')]) == 2


def test_stops_start_early_enough_to_coast_into():
  exp = Experimenter()
  assert exp._stop_entry(cruise(0.0, 13.4, stop_distance_m=250.0)) == ('stop', 'all')   # needs 0.36 m/s2
  assert dl.DIALS['stop'][0].low <= 0.3


# ---------- approach: lift off early behind a slower car ----------
def test_closing_on_a_slower_car_is_an_approach_and_lift_coasts():
  exp = Experimenter()
  exp.draw = lambda category, context: 'lift' if category == 'approach' else ex.CATEGORIES[category][0]
  far = cruise(0.0, 25.0, lead_d_m=140.0, lead_v=18.0, lead_gap_s=140.0 / 25.0)
  p = exp.sample(far)
  assert p.tag == 'approach|all|lift'
  # (140 - 1.8 * 18 - 6) / 7 = 14.5 s to catch up: not yet
  assert p.accel_max is None
  near = cruise(0.05, 25.0, lead_d_m=90.0, lead_v=18.0, lead_gap_s=90.0 / 25.0)   # 7.3 s
  p = exp.sample(near)
  assert p.accel_max == pytest.approx(ex.coast_accel(25.0, 0.0)) and p.accel_max < 0
  assert exp.spent_s > 0


def test_no_approach_behind_a_stopped_or_close_or_matching_car():
  exp = Experimenter()
  assert exp._enter(cruise(0.0, 25.0, lead_d_m=100.0, lead_v=0.0, lead_gap_s=4.0)) != ('approach', 'all')
  assert exp._enter(cruise(0.0, 25.0, lead_d_m=50.0, lead_v=18.0, lead_gap_s=2.0)) == ('follow', 'all')
  assert exp._enter(cruise(0.0, 25.0, lead_d_m=100.0, lead_v=24.0, lead_gap_s=4.0)) != ('approach', 'all')


def test_the_approach_dial_lifts_off_inside_its_horizon():
  assert dl.effect('approach', {'h': 0.0}, v_ego=25.0, v_target=25.0, normal=0.6, below_target_mps=0.0,
                   closing_s=3.0, coast=-0.27).accel_max is None   # normal: hold speed, brake late
  assert dl.effect('approach', {'h': 6.0}, v_ego=25.0, v_target=25.0, normal=0.6, below_target_mps=0.0,
                   closing_s=5.0, coast=-0.27).accel_max == -0.27
  assert dl.effect('approach', {'h': 6.0}, v_ego=25.0, v_target=25.0, normal=0.6, below_target_mps=0.0,
                   closing_s=9.0, coast=-0.27).accel_max is None
  assert dl.costs_time('approach', {'h': 6.0})


def test_the_dial_engine_runs_approaches_too():
  exp = DialExperimenter()
  exp.draw = lambda category, context: 'h8.0' if category == 'approach' else dl.encode(category, dl.start_values(category))
  p = exp.sample(cruise(0.0, 25.0, lead_d_m=90.0, lead_v=18.0, lead_gap_s=3.6))
  assert p.tag == 'approach|all|h8.0' and p.accel_max is not None and p.accel_max < 0


# ---------- judged on measured energy ----------
def _stop(tag, v0, prop_kj, regen_kj, metres=60.0, v1=0.0):
  return {'exp': tag, 'v0': v0, 'v1': v1, 'prop_j': prop_kj * 1e3, 'regen_j': regen_kj * 1e3, 'fric_j': 0, 'm': metres,
          'dur': 8.0, 'pitch': 0.0, 'obd_gal': 0.0, 'soc_d': 0.0, 'ctx': ['15-30 mph', 'flat', 'straight', 'open road'],
          'ambient_c': 0.0}


def test_each_stop_is_one_run_scored_by_the_energy_lost():
  late = [_stop('stop|30-45|r2.00', 15.0, 40, 60), _stop('stop|30-45|r2.00', 8.0, 0, 50)]   # cruised on, braked hard
  gentle = [_stop('stop|all|r1.50', 15.0, 0, 90)]                                         # lifted early, regen'd
  rows = el.samples([late + [{'exp': None, 'm': 100.0}] + gentle])
  hard, soft = rows[('stop', 'all', 'r2.00')], rows[('stop', 'all', 'r1.50')]
  assert len(hard) == 1 and len(soft) == 1 and hard[0].metres == pytest.approx(120.0)
  kinetic = 0.5 * el.MASS_KG * 15.0 ** 2
  assert hard[0].gal == pytest.approx((kinetic + 40e3 - el.REGEN_ROUND_TRIP * 110e3) / el.WHEEL_J_PER_GAL)
  assert soft[0].gal < hard[0].gal
  assert el.enough('stop', soft * el.MIN_EPISODES)   # counted in stops, not miles


def test_an_approach_is_scored_over_a_fixed_stretch_of_road():
  held = [_stop('approach|all|h0.0', 25.0, 120, 0, 300, 25.0), _stop('approach|all|h0.0', 25.0, 0, 80, 250, 18.0)]
  lifted = [_stop('approach|all|h6.0', 25.0, 0, 20, 300, 20.0), _stop(None, 20.0, 30, 0, 250, 18.0)]
  rows = el.samples([held + [{'exp': None, 'm': 50.0}] + lifted])
  a, b = rows[('approach', 'all', 'h0.0')][0], rows[('approach', 'all', 'h6.0')][0]
  assert a.metres >= el.APPROACH_WINDOW_M and b.metres >= el.APPROACH_WINDOW_M
  ke = 0.5 * el.MASS_KG * (25.0 ** 2 - 18.0 ** 2)
  assert a.gal == pytest.approx((ke + 120e3 - el.REGEN_ROUND_TRIP * 80e3) / el.WHEEL_J_PER_GAL)
  assert b.gal < a.gal
  assert math.isfinite(b.seconds)


def test_the_evidence_bar_is_lower():
  assert (el.DIAL_MIN_MILES, el.DIAL_MIN_EPISODES, el.DIAL_MIN_DRIVES) == (4.0, 12, 4)
  assert el.DIAL_CONFIDENCE == 0.95
