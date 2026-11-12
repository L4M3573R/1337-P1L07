import math
from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import messaging
from openpilot.common.params import Params
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.starpilot.controllers.coast import (CoastOwner, CoastRuntime, FEATHER_ACCEL, FEATHER_MAX_SPEED_MPS, GLIDE_FLOOR_MARGIN,
                                                  TRANSITION_JERK, DT, cluster_ratio, glide_accel, pulse_accel, regen_glide_accel)
from openpilot.starpilot.controllers.wheel_actions import WheelPublisher
from openpilot.starpilot.controllers.tests.test_wheel_actions import Publisher, DRIVE, NOW
from openpilot.starpilot.longitudinal.tests.test_cruise_ceiling import messages


def sample(owner, **changes):
  values = {'admitted': True, 'active': True, 'gas': False, 'brake': False, 'speed_mps': 20.0,
                'target_mps': 20.0, 'delta_mps': 2.0, 'pitch': 0.0, 'lead_relevant': False, 'stop_context': False}
  values.update(changes)
  return owner.sample(**values)


def test_pulse_glide_hysteresis_grade_pause_and_band():
  owner = CoastOwner()
  assert owner.toggle(14, long_active=True)
  # At the set speed it glides; the band runs the full set speed down to set - delta (18 m/s).
  assert sample(owner).pulse_coasting
  assert sample(owner, speed_mps=18.01).pulse_coasting
  assert not sample(owner, speed_mps=18.0).pulse_coasting    # glide runs all the way to the bottom of the band
  assert sample(owner, speed_mps=19.89).pulsing             # pulse until the set speed is reached
  assert sample(owner, speed_mps=19.9).pulse_coasting
  assert not sample(owner, pitch=math.radians(3)).pulse_coasting
  assert not sample(owner, pitch=None).pulse_coasting
  assert not sample(owner, pitch=math.radians(2.51)).pulse_coasting
  assert sample(owner, pitch=math.radians(2.5)).pulse_coasting
  assert sample(owner, target_mps=15.0, speed_mps=15.0).speed_ceiling_mps is None


@pytest.mark.parametrize('speed', [12.0, 20.0, 27.0, 29.0, 35.0])
def test_glide_feathers_in_ev_below_the_cutoff_and_keeps_the_regen_glide_above(speed):
  owner = CoastOwner()
  owner.toggle(14, long_active=True)
  plan = sample(owner, speed_mps=speed, target_mps=speed, delta_mps=2.2)
  assert plan.pulse_coasting and not plan.pulsing
  assert plan.accel_target == pytest.approx(glide_accel(speed, 0.0))
  assert plan.brake_floor == pytest.approx(plan.accel_target - GLIDE_FLOOR_MARGIN)
  if speed < FEATHER_MAX_SPEED_MPS:
    # Just enough request to cancel regen: the Elantra stays electric with the battery about neutral.
    assert plan.accel_target == pytest.approx(FEATHER_ACCEL)
  else:
    # Above ~63 mph a lighter request starts the engine, so the natural-coast glide is kept.
    assert plan.accel_target == pytest.approx(regen_glide_accel(speed)) and plan.accel_target < -0.25


def test_glide_follows_grade_but_never_exceeds_the_set_speed():
  owner = CoastOwner()
  owner.toggle(14, long_active=True)
  assert sample(owner).pulse_coasting          # enter the glide at the set speed
  flat = sample(owner, speed_mps=19.5).accel_target
  uphill = sample(owner, speed_mps=19.5, pitch=math.radians(2.0)).accel_target
  assert uphill < flat
  # Downhill, natural coasting would speed the car up; at the set speed it holds instead.
  downhill = sample(owner, speed_mps=20.0, pitch=math.radians(-2.0)).accel_target
  assert downhill <= 0.0
  assert sample(owner, speed_mps=19.5, pitch=math.radians(-2.0)).accel_target <= 0.5


def test_pulse_is_steady_and_eases_only_just_below_the_set_speed():
  owner = CoastOwner()
  owner.toggle(14, long_active=True)
  sample(owner, speed_mps=17.95)           # below the bottom of the band: pulsing
  low = sample(owner, speed_mps=18.3)
  mid = sample(owner, speed_mps=19.0)
  assert low.pulsing and mid.pulsing and not low.pulse_coasting
  assert low.accel_target == pytest.approx(pulse_accel(18.3))
  assert mid.accel_target == pytest.approx(pulse_accel(19.0))
  near = sample(owner, speed_mps=19.6)
  assert 0.0 < near.accel_target < pulse_accel(19.6)
  assert near.brake_floor is None
  assert pulse_accel(12.0) > pulse_accel(30.0) > 0.3


def test_far_below_the_band_ordinary_cruise_returns():
  owner = CoastOwner()
  owner.toggle(14, long_active=True)
  plan = sample(owner, speed_mps=15.0)     # band bottom is 18 m/s
  assert plan.accel_target is None and not plan.pulsing and not plan.pulse_coasting


@pytest.mark.parametrize('restriction', [{'lead_relevant': True}, {'stop_context': True}, {'target_mps': 5.0},
                                         {'target_mps': 4.0}, {'delta_mps': 0.0}])
def test_pulse_cannot_coast_into_lead_stop_or_low_target(restriction):
  owner = CoastOwner()
  owner.toggle(14, long_active=True)
  assert not sample(owner, **restriction).pulse_coasting


def test_force_coast_driver_reset_inactive_freeze_and_source_loss():
  owner = CoastOwner()
  owner.toggle(2, long_active=True)
  owner.toggle(14, long_active=True)
  assert sample(owner).force_decel
  assert sample(owner, active=False).brake_floor is None
  assert owner.force_coast and owner.pulse_glide
  assert not sample(owner, gas=True).force_decel
  assert owner.pulse_glide
  sample(owner, admitted=False)
  assert not owner.pulse_glide and not owner.force_coast


def run_planner(params, cp, *, action=0, lead=False, v_ego=None, v_cluster=None):
  class Services(dict):
    pass
  runtime, publisher = CoastRuntime(), WheelPublisher()
  sender = Publisher()
  planner = LongitudinalPlanner(cp, init_v=20.0)
  for frame in range(65):
    now = NOW + frame * 50_000_000
    data, _ = messages(lead=lead)
    sm = Services(data)
    sm['deviceState'] = NS(started=True, startedMonoTime=DRIVE)
    sm['carState'].canValid = True
    sm['carState'].vCruise = 100.0
    if v_ego is not None:
      sm['carState'].vEgo = v_ego
    if v_cluster is not None:
      sm['carState'].vEgoCluster = v_cluster
    sm['carControl'].enabled = sm['carControl'].longActive = True
    sm.logMonoTime = dict.fromkeys(sm, now)
    sm.recv_time = dict.fromkeys(sm, now / 1e9)
    sm.seen = sm.valid = sm.alive = dict.fromkeys(sm, True)
    if frame == 0 and action:
      params.put('DistanceButtonControl', action, block=True)
      command = (('DistanceButtonControl', action),)
    else:
      command = (('', 0),)
    publisher.publish(command, cp, sender, now_ns=now, drive_id=DRIVE, source_car_ns=now, source_control_ns=now)
    runtime.receive(messaging.log_from_bytes(sender.events[-1][1]), params, cp, sm, now_ns=now)
    planner.update(sm, now_ns=now, drive_id=DRIVE, wheel_coast=(runtime, params))
  return planner, runtime


def test_actual_wheel_receipt_changes_cruise_but_preserves_native_mpc_lead_braking(tmp_path):
  params = Params(str(tmp_path))
  cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  ordinary, _ = run_planner(params, cp)
  coast, owner = run_planner(params, cp, action=2)
  assert owner.owner.force_coast
  assert coast.a_cruise < ordinary.a_cruise
  assert coast.mpc.params[0, 4] == ordinary.mpc.params[0, 4]
  lead, _ = run_planner(params, cp, action=14, lead=True)
  native_lead, _ = run_planner(params, cp, lead=True)
  assert lead.output_a_target == pytest.approx(native_lead.output_a_target)
  assert lead.output_a_target < lead.a_cruise


def test_planner_glides_at_natural_coast_at_highway_speed(tmp_path):
  # Above the EV cut-off a lighter request starts the engine, so the cruise targets natural coast.
  params = Params(str(tmp_path))
  params.put('PulseGlideSpeedDelta', 5.0, block=True)
  params.put('IsMetric', False, block=True)
  cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  planner, runtime = run_planner(params, cp, action=14, v_ego=100 / 3.6)  # 62 mph: EV feather
  assert runtime.owner.pulse_glide and runtime.owner.coasting
  assert planner.a_cruise == pytest.approx(FEATHER_ACCEL, abs=0.02)


def test_band_follows_the_dash_speedometer(tmp_path):
  # Hyundai's dash reads ~2.7% above wheel speed. At a wheel speed whose dash reading has reached the
  # 100 km/h set speed, the pulse is done and the glide starts; without a dash speed it would still pulse.
  params = Params(str(tmp_path))
  params.put('PulseGlideSpeedDelta', 5.0, block=True)
  cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  wheel = 27.2  # m/s; the 100 km/h set speed is 27.78 m/s
  _, with_dash = run_planner(params, cp, action=14, v_ego=wheel, v_cluster=wheel * 1.027)
  _, wheel_only = run_planner(params, cp, action=14, v_ego=wheel)
  assert with_dash.owner.coasting
  assert not wheel_only.owner.coasting


@pytest.mark.parametrize('v,dash,expected', [(28.0, 28.0 * 1.027, 1.027), (28.0, 0.0, 1.0), (3.0, 3.1, 1.0),
                                             (28.0, 28.0 * 1.3, 1.0), (float('nan'), 28.0, 1.0)])
def test_cluster_ratio_accepts_only_plausible_dash_readings(v, dash, expected):
  assert cluster_ratio(NS(vEgo=v, vEgoCluster=dash)) == pytest.approx(expected)


def test_log_fields_report_phase_band_and_target():
  from openpilot.starpilot.controllers.coast import plan_log_fields
  owner = CoastOwner()
  owner.toggle(14, long_active=True)
  runtime = NS(owner=owner, cluster_ratio=1.027)
  glide = plan_log_fields(sample(owner), runtime)
  assert glide['pulseGlideEnabled'] and glide['pulseGlidePhase'] == 'glide'
  assert glide['pulseGlideBandTop'] == 20.0 and glide['pulseGlideBandBottom'] == 18.0
  assert glide['pulseGlideAccelTarget'] < 0 and glide['pulseGlideClusterRatio'] == pytest.approx(1.027)
  fresh = CoastOwner()
  fresh.toggle(14, long_active=True)
  pulse = plan_log_fields(sample(fresh, speed_mps=17.95), runtime)  # pulse targets are eased after a glide
  assert pulse['pulseGlidePhase'] == 'pulse' and pulse['pulseGlideAccelTarget'] > 0
  assert plan_log_fields(sample(owner, lead_relevant=True), runtime)['pulseGlidePhase'] == 'off'
  assert plan_log_fields(None, None) == {'pulseGlideEnabled': False, 'pulseGlidePhase': 'off'}


def test_the_published_plan_carries_the_pulse_glide_fields(tmp_path):
  from openpilot.cereal import messaging as msg
  params = Params(str(tmp_path))
  params.put('PulseGlideSpeedDelta', 5.0, block=True)
  cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  planner, _ = run_planner(params, cp, action=14, v_ego=100 / 3.6)

  class PublishSM(dict):
    def all_checks(self, names):
      return True
  data, _ = messages()
  sm = PublishSM(data)
  sm.logMonoTime = dict.fromkeys(sm, NOW)
  sent = {}
  planner.publish(sm, NS(send=lambda name, message: sent.__setitem__(name, message)))
  plan = msg.log_from_bytes(sent['starpilotLongitudinalPlan'].to_bytes()).starpilotLongitudinalPlan
  assert plan.pulseGlideEnabled and str(plan.pulseGlidePhase) == 'glide'
  assert plan.pulseGlideBandTop == pytest.approx(100 / 3.6, abs=0.01)
  assert plan.pulseGlideBandBottom == pytest.approx(100 / 3.6 - 5 * 0.44704, abs=0.01)
  assert plan.pulseGlideAccelTarget == pytest.approx(FEATHER_ACCEL, abs=0.02)  # 62 mph: EV feather


def test_glide_entry_and_pulse_entry_are_eased_not_stepped():
  owner = CoastOwner()
  owner.toggle(14, long_active=True)
  pulse = sample(owner, speed_mps=17.9)            # below the band: pulsing hard
  assert pulse.pulsing and pulse.accel_target > 0.5
  first = sample(owner, speed_mps=19.95)           # reaches the set speed: glide starts
  assert first.pulse_coasting
  assert first.accel_target == pytest.approx(pulse.accel_target - TRANSITION_JERK * DT)  # one eased step, no jump
  steps = [sample(owner, speed_mps=19.9).accel_target for _ in range(40)]
  assert all(b <= a + 1e-9 and a - b <= TRANSITION_JERK * DT + 1e-9 for a, b in zip([first.accel_target] + steps[:-1], steps, strict=True))
  assert steps[-1] == pytest.approx(glide_accel(19.9, 0.0))  # settles on the feather target


def test_the_ease_restarts_fresh_after_a_pause():
  owner = CoastOwner()
  owner.toggle(14, long_active=True)
  sample(owner, speed_mps=17.9)
  sample(owner, active=False)
  plan = sample(owner, speed_mps=20.0)             # resumes straight into a glide
  assert plan.accel_target == pytest.approx(glide_accel(20.0, 0.0))


def test_regen_glide_above_the_cutoff_blends_into_the_feather_below_it():
  owner = CoastOwner()
  owner.toggle(14, long_active=True)
  top = FEATHER_MAX_SPEED_MPS + 1.0
  above = sample(owner, speed_mps=top, target_mps=top, delta_mps=4.0)
  assert above.accel_target == pytest.approx(regen_glide_accel(top))
  below = sample(owner, speed_mps=FEATHER_MAX_SPEED_MPS - 0.1, target_mps=top, delta_mps=4.0)
  assert below.accel_target == pytest.approx(above.accel_target + TRANSITION_JERK * DT)  # eased toward the feather
