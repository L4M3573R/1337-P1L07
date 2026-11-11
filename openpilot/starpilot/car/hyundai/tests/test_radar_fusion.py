"""Tandem camera + radar leadOne fusion for Hyundai front radars."""
from types import SimpleNamespace

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.radard import KalmanParams, RadarD, Track
from openpilot.starpilot.car.hyundai import radar_fusion as rf

STRAIGHT = SimpleNamespace(position=SimpleNamespace(x=[0.0, 50.0, 200.0], y=[0.0, 0.0, 0.0]))
KP = KalmanParams(DT_MDL)


def track(identifier, d, y=0.0, v_lead=20.0, v_ego=20.0):
  t = Track(identifier, v_lead, KP)
  t.update(d, y, v_lead - v_ego, v_lead)
  return t


def camera_lead(t, prob=0.9):
  return t.get_RadarState(prob)


def vision_lead(d, prob=0.9):
  return {'present': True, 'radar': False, 'radarTrackId': -1, 'dRel': d, 'modelProb': prob}


NO_LEAD = {'present': False}


def run(policy, frames, base, tracks, prob=0.2, model=STRAIGHT):
  lead = None
  for _ in range(frames):
    lead = policy.lead_one(base, tracks, model, prob)
  return lead


def test_supported_only_for_elantra_hybrid_with_radar():
  assert rf.supported("HYUNDAI_ELANTRA_HEV_2024", False)
  assert not rf.supported("HYUNDAI_ELANTRA_HEV_2024", True)
  assert not rf.supported("HYUNDAI_IONIQ_6", False)


def test_camera_confirmed_lead_passes_through():
  car = track(1, 40.0)
  base = camera_lead(car)
  assert rf.TandemLeadPolicy().lead_one(base, {1: car}, STRAIGHT, 0.9) is base


def test_confirmed_stopped_car_is_held_through_camera_dropout_then_released():
  policy, car = rf.TandemLeadPolicy(), track(1, 40.0, v_lead=0.0)
  policy.lead_one(camera_lead(car), {1: car}, STRAIGHT, 0.9)
  held = run(policy, rf.HOLD_MAX_FRAMES, NO_LEAD, {1: car})
  assert held['present'] and held['radar'] and held['radarTrackId'] == 1
  assert held['modelProb'] == 0.2  # carries the camera's real probability
  assert not policy.lead_one(NO_LEAD, {1: car}, STRAIGHT, 0.2)['present']


def test_confirmed_moving_car_stays_the_lead_past_the_hold_through_pickup():
  policy, car = rf.TandemLeadPolicy(), track(1, 40.0)
  policy.lead_one(camera_lead(car), {1: car}, STRAIGHT, 0.9)
  lead = run(policy, rf.HOLD_MAX_FRAMES * 2, NO_LEAD, {1: car})
  assert lead['present'] and lead['radarTrackId'] == 1


def test_hold_ends_when_the_car_leaves_the_path_or_disappears():
  policy, car = rf.TandemLeadPolicy(), track(1, 40.0)
  policy.lead_one(camera_lead(car), {1: car}, STRAIGHT, 0.9)
  car.update(40.0, rf.HOLD_PATH_HALF_WIDTH + 0.5, 0.0, 20.0)
  assert not policy.lead_one(NO_LEAD, {1: car}, STRAIGHT, 0.2)['present']
  policy, car = rf.TandemLeadPolicy(), track(1, 40.0)
  policy.lead_one(camera_lead(car), {1: car}, STRAIGHT, 0.9)
  assert not policy.lead_one(NO_LEAD, {}, STRAIGHT, 0.2)['present']


def test_hold_yields_to_a_confident_camera_lead_on_a_different_car():
  policy, old = rf.TandemLeadPolicy(), track(1, 30.0)
  policy.lead_one(camera_lead(old), {1: old}, STRAIGHT, 0.9)
  new_ahead = vision_lead(60.0, prob=0.95)  # lane change: camera now follows a farther car
  assert policy.lead_one(new_ahead, {1: old}, STRAIGHT, 0.95) is new_ahead


def test_radar_keeps_measuring_the_same_car_when_radard_loses_the_match():
  policy, car = rf.TandemLeadPolicy(), track(1, 50.0)
  policy.lead_one(camera_lead(car), {1: car}, STRAIGHT, 0.9)
  lead = policy.lead_one(vision_lead(54.0, prob=0.9), {1: car}, STRAIGHT, 0.9)
  assert lead['radar'] and lead['radarTrackId'] == 1


def test_hold_yields_to_a_much_closer_camera_lead():
  policy, car = rf.TandemLeadPolicy(), track(1, 60.0)
  policy.lead_one(camera_lead(car), {1: car}, STRAIGHT, 0.9)
  closer = vision_lead(30.0)
  assert policy.lead_one(closer, {1: car}, STRAIGHT, 0.9) is closer


def test_moving_in_path_car_is_picked_up_after_persisting():
  policy, car = rf.TandemLeadPolicy(), track(7, 50.0, v_lead=15.0)
  assert not run(policy, rf.PICKUP_MIN_FRAMES - 1, NO_LEAD, {7: car})['present']
  lead = policy.lead_one(NO_LEAD, {7: car}, STRAIGHT, 0.2)
  assert lead['present'] and lead['radarTrackId'] == 7 and lead['modelProb'] == 0.2


def test_stationary_or_off_path_returns_need_the_camera():
  for car in (track(3, 50.0, v_lead=0.0), track(4, 50.0, y=rf.PATH_HALF_WIDTH + 0.3),
              track(5, rf.PICKUP_MAX_DISTANCE + 10.0)):
    assert not run(rf.TandemLeadPolicy(), rf.PICKUP_MIN_FRAMES * 3, NO_LEAD, {car.identifier: car})['present']


def test_path_follows_the_model_curve():
  curve = SimpleNamespace(position=SimpleNamespace(x=[0.0, 50.0, 100.0], y=[0.0, 3.0, 12.0]))
  in_lane_on_curve = track(8, 50.0, y=-3.0)  # radar yRel is the negated model y
  straight_ahead = track(9, 50.0, y=0.0)
  lead = run(rf.TandemLeadPolicy(), rf.PICKUP_MIN_FRAMES, NO_LEAD, {8: in_lane_on_curve, 9: straight_ahead}, model=curve)
  assert lead['radarTrackId'] == 8


def test_confident_camera_lead_is_never_overridden_by_a_closer_radar_track():
  near, far = track(1, 30.0, v_lead=18.0), track(2, 60.0)
  camera_far = camera_lead(far)
  lead = run(rf.TandemLeadPolicy(), rf.PICKUP_MIN_FRAMES * 3, camera_far, {1: near, 2: far}, prob=0.9)
  assert lead is camera_far


def test_pickup_replaces_a_low_confidence_lead_only_when_clearly_closer():
  near, far = track(1, 30.0, v_lead=18.0), track(2, 60.0)
  weak_far = vision_lead(60.0, prob=0.3)
  lead = run(rf.TandemLeadPolicy(), rf.PICKUP_MIN_FRAMES, weak_far, {1: near, 2: far}, prob=0.3)
  assert lead['radarTrackId'] == 1
  similar = track(3, 57.0)
  lead = run(rf.TandemLeadPolicy(), rf.PICKUP_MIN_FRAMES, weak_far, {3: similar}, prob=0.3)
  assert lead is weak_far


def test_radard_enables_tandem_only_when_asked_and_not_with_vision_only_acc():
  assert RadarD(radar_available=True, tandem_fusion=True).tandem is not None
  assert RadarD(radar_available=True).tandem is None
  assert RadarD(radar_available=True, tandem_fusion=True, use_voacc=True).tandem is None


def test_decision_log_records_camera_hold_and_pickup():
  policy, car = rf.TandemLeadPolicy(), track(1, 40.0, v_lead=0.0)
  policy.lead_one(camera_lead(car), {1: car}, STRAIGHT, 0.9)
  assert policy.last_action == 'camera'
  policy.lead_one(NO_LEAD, {1: car}, STRAIGHT, 0.2)
  assert policy.last_action == 'hold' and policy.held.track_id == 1 and policy.last_lead_prob == 0.2
  picker, mover = rf.TandemLeadPolicy(), track(7, 50.0, v_lead=15.0)
  run(picker, rf.PICKUP_MIN_FRAMES, NO_LEAD, {7: mover})
  assert picker.last_action == 'pickup' and picker.last_pickup_id == 7
