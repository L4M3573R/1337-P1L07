import pytest

from opendbc.car.car_helpers import interfaces
from opendbc.car.structs import CarParams
from openpilot.starpilot.conditional_mode.stop_release import owner_for


BRANDS = sorted({interface.__module__.split('.')[-2] for interface in interfaces.values()})


@pytest.mark.parametrize('brand', BRANDS)
def test_cem_departure_owner_is_shared_across_active_longitudinal_configs(brand):
  for identity, interface in interfaces.items():
    if interface.__module__.split('.')[-2] != brand:
      continue
    cp = interface.get_non_essential_params(identity)
    cp.openpilotLongitudinalControl = True
    assert (owner_for(cp) is not None) == (not (cp.passive or cp.dashcamOnly or cp.notCar)), identity


@pytest.mark.parametrize('flag', ['passive', 'dashcamOnly', 'notCar'])
def test_inactive_profiles_never_get_departure_owner(flag):
  cp = CarParams(openpilotLongitudinalControl=True)
  setattr(cp, flag, True)
  assert owner_for(cp) is None


def test_stock_longitudinal_never_gets_departure_owner():
  assert owner_for(CarParams(openpilotLongitudinalControl=False)) is None


from types import SimpleNamespace
from functools import lru_cache
from openpilot.cereal import messaging
from openpilot.selfdrive.controls.radard import get_RadarState_from_vision
from openpilot.starpilot.conditional_mode.stop_release import (StoppedLeadRelease, LEAD_EXTENT_MARGIN_M, LEAD_MAX_DISTANCE_M,
                                                              MIN_MODEL_ACCEL)
from openpilot.starpilot.longitudinal.tests import test_cem_green_release as green
from openpilot.starpilot.tests.test_ford_recorded_departure import _motion_payloads, _samples


@lru_cache(maxsize=1)
def recorded_motion():
  return next(sample for sample in _samples() if 334.44 < sample['routeSeconds'] < 334.48)


def scene(now, vision=False):
  motion = recorded_motion()
  payloads = _motion_payloads(green.serialized_scene(stamp=green.BOOT + now - green.MONO - 2_000_000,
    speed=0., standstill=True), motion)
  for service in ('carControl', 'controlsState'):
    event = messaging.new_message(service)
    setattr(event, service, payloads[service])
    payloads[service] = getattr(event, service)
  if vision:
    model = payloads['modelV2']
    for slot, lead in enumerate((payloads['radarState'].leadOne, payloads['radarState'].leadTwo)):
      if lead.present:
        raw = model.leadsV3[slot]
        lead.from_dict(get_RadarState_from_vision(raw, payloads['carState'].vEgo, model.velocity.x[0], raw.prob))
  return green.CallerFrame(payloads, now - 2_000_000)


@pytest.mark.parametrize('brand', BRANDS)
@pytest.mark.parametrize('vision', [False, True])
def test_shared_release_preserves_native_target_with_radar_or_correlated_vision(brand, vision):
  cp = next(interface.get_non_essential_params(identity) for identity, interface in interfaces.items()
            if interface.__module__.split('.')[-2] == brand)
  cp.openpilotLongitudinalControl = True
  owner = owner_for(cp)
  if cp.passive or cp.dashcamOnly or cp.notCar:
    assert owner is None
    return
  assert owner is not None
  detector = SimpleNamespace(committed=True, standstill_committed=True, standstill_reason='light', commit_distance_m=3.)
  planner = SimpleNamespace(mpc=SimpleNamespace(solution_status=0), output_should_stop=False, output_a_target=.5,
                            force_stop_plan=SimpleNamespace(forcing=False, should_stop=False))
  results = []
  for tick in range(13):
    now = green.MONO + 1_000_000_000 + tick * 50_000_000
    results.append(owner.sample(scene(now, vision), cp, planner, detector, now_ns=now, drive_id=green.MONO-1_000_000_000))
    assert planner.output_a_target == .5 and planner.output_should_stop is False
  assert not any(results[:10]) and all(results[10:])


@pytest.mark.parametrize('defect', ['mismatched-model', 'weak-model-lead', 'native-stop', 'native-braking', 'force-stop',
                                  'second-stopped', 'model-stop', 'stale', 'sign', 'config-change', 'source-change'])
def test_vision_release_requires_continuous_corroboration_and_native_go(defect):
  cp = CarParams(openpilotLongitudinalControl=True, carFingerprint='KIA_NIRO_EV')
  owner = StoppedLeadRelease()
  detector = SimpleNamespace(committed=True, standstill_committed=True, standstill_reason='light', commit_distance_m=3.)
  planner = SimpleNamespace(mpc=SimpleNamespace(solution_status=0), output_should_stop=False, output_a_target=.5,
                            force_stop_plan=SimpleNamespace(forcing=False, should_stop=False))
  results = []
  for tick in range(13):
    now = green.MONO + 1_000_000_000 + tick * 50_000_000
    sm = scene(now, True)
    if defect == 'mismatched-model':
      sm['radarState'].leadOne.dRel += 1.
    elif defect == 'weak-model-lead':
      sm['modelV2'].leadsV3[0].prob = .5
    elif defect == 'native-stop':
      planner.output_should_stop = True
    elif defect == 'native-braking':
      planner.output_a_target = -.1
    elif defect == 'force-stop':
      planner.force_stop_plan.forcing = True
    elif defect == 'second-stopped':
      sm['radarState'].leadTwo.vLead = 0.
    elif defect == 'model-stop':
      sm['modelV2'].action.shouldStop = True
    elif defect == 'stale':
      sm.logMonoTime['radarState'] = now - 150_000_001
    elif defect == 'sign':
      detector.standstill_reason = 'sign'
    elif defect == 'config-change':
      cp.carFingerprint = str(tick)
    elif defect == 'source-change' and tick % 2:
      sm = scene(now, False)
    results.append(owner.sample(sm, cp, planner, detector, now_ns=now, drive_id=green.MONO-1_000_000_000))
  assert not any(results)


@pytest.mark.parametrize('vision', [False, True])
def test_niro_actual_cem_hold_releases_only_after_native_go(monkeypatch, vision):
  from opendbc.car.hyundai.values import CAR
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
  case = green.CemGreenCallerTests(methodName='runTest')
  case.setUp()
  case.cp = interfaces[CAR.KIA_NIRO_EV].get_non_essential_params(CAR.KIA_NIRO_EV)
  case.cp.openpilotLongitudinalControl = True
  case.cp.pcmCruise = False
  case.planner = LongitudinalPlanner(case.cp, init_v=4.27)
  try:
    case.prime()
    original = green.serialized_scene
    samples = [sample for sample in _samples() if 334.29 <= sample['routeSeconds'] <= 335.3]
    motion = samples[0]
    def moving_lead(**kwargs):
      payloads = _motion_payloads(original(**kwargs), motion)
      if vision:
        model = payloads['modelV2']
        for slot, lead in enumerate((payloads['radarState'].leadOne, payloads['radarState'].leadTwo)):
          if lead.present:
            raw = model.leadsV3[slot]
            lead.from_dict(get_RadarState_from_vision(raw, payloads['carState'].vEgo, model.velocity.x[0], raw.prob))
      return payloads
    monkeypatch.setattr(green, 'serialized_scene', moving_lead)
    observations = []
    original_sample = StoppedLeadRelease.sample
    def observed(owner, sm, cp, planner, detector, **kwargs):
      before = float(planner.output_a_target), bool(planner.output_should_stop)
      released = original_sample(owner, sm, cp, planner, detector, **kwargs)
      observations.append((released, before, float(planner.output_a_target), bool(planner.output_should_stop)))
      return released
    monkeypatch.setattr(StoppedLeadRelease, 'sample', observed)
    results = []
    for motion in samples:
      _, _, held, plan = case.cycle(stopped=True, position=motion['model']['position']['x'],
        velocity=motion['model']['velocity']['x'], should_stop=False,
        desired_accel=motion['model']['action']['desiredAcceleration'], lead=True)
      results.append((held, plan.shouldStop, plan.aTarget))
    assert any(released for released, _, _, _ in observations), (observations, results)
    assert all(before == (target, stop) for _, before, target, stop in observations)
    assert any(not held and not stop and target > .1 for held, stop, target in results)
    assert not case.owner.mode.projector.stop_detector.committed
  finally:
    case.doCleanups()


def _release_results(mutate, *, vision=False):
  cp = CarParams(openpilotLongitudinalControl=True, carFingerprint='HYUNDAI_ELANTRA_HEV_2024')
  owner = StoppedLeadRelease()
  detector = SimpleNamespace(committed=True, standstill_committed=True, standstill_reason='light', commit_distance_m=0.)
  planner = SimpleNamespace(mpc=SimpleNamespace(solution_status=0), output_should_stop=False, output_a_target=.5,
                            force_stop_plan=SimpleNamespace(forcing=False, should_stop=False))
  results = []
  for tick in range(13):
    now = green.MONO + 1_000_000_000 + tick * 50_000_000
    sm = scene(now, vision)
    mutate(sm)
    results.append(owner.sample(sm, cp, planner, detector, now_ns=now, drive_id=green.MONO-1_000_000_000))
  return results


def _scale_path(sm, extent):
  model = sm['modelV2']
  positions = list(model.position.x)
  model.position.x = [x * extent / max(positions) for x in positions]


@pytest.mark.parametrize('vision', [False, True])
def test_departing_lead_releases_before_road_is_clear_for_the_full_extent(vision):
  # Recorded Elantra Hybrid stops: behind a departing lead the model plans only 22-52 m,
  # below the 54 m clear-road extent, yet it has already chosen to go.
  results = _release_results(lambda sm: _scale_path(sm, 30.), vision=vision)
  assert not any(results[:10]) and all(results[10:])


@pytest.mark.parametrize('vision', [False, True])
def test_model_path_must_reach_past_the_departing_lead(vision):
  def short_of_lead(sm):
    nearest = min(lead.dRel for lead in (sm['radarState'].leadOne, sm['radarState'].leadTwo) if lead.present)
    _scale_path(sm, nearest + LEAD_EXTENT_MARGIN_M - .5)
  assert not any(_release_results(short_of_lead, vision=vision))


@pytest.mark.parametrize('accel,released', [(.2, True), (MIN_MODEL_ACCEL, True), (.1, False), (-.1, False)])
def test_gentle_model_launch_is_enough(accel, released):
  def set_accel(sm):
    sm['modelV2'].action.desiredAcceleration = accel
  results = _release_results(set_accel)
  assert (not any(results[:10]) and all(results[10:])) if released else not any(results)


@pytest.mark.parametrize('distance,released', [(25., True), (LEAD_MAX_DISTANCE_M, True), (LEAD_MAX_DISTANCE_M + .5, False)])
def test_departing_lead_window_extends_beyond_eighteen_metres(distance, released):
  def move_leads(sm):
    for lead in (sm['radarState'].leadOne, sm['radarState'].leadTwo):
      if lead.present:
        lead.dRel = distance
  results = _release_results(move_leads)
  assert (not any(results[:10]) and all(results[10:])) if released else not any(results)


@pytest.mark.parametrize('extra,released', [(.5, True), (-.5, False)])
def test_far_departing_lead_still_needs_path_past_it(extra, released):
  def far_lead(sm):
    for lead in (sm['radarState'].leadOne, sm['radarState'].leadTwo):
      if lead.present:
        lead.dRel = LEAD_MAX_DISTANCE_M
    _scale_path(sm, LEAD_MAX_DISTANCE_M + LEAD_EXTENT_MARGIN_M + extra)
  results = _release_results(far_lead)
  assert (not any(results[:10]) and all(results[10:])) if released else not any(results)
