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


# A left turn: the predicted path is 3 m to the left (model y is right-positive) by 25 m.
LEFT_TURN = SimpleNamespace(position=SimpleNamespace(x=[0.0, 10.0, 25.0, 50.0], y=[0.0, -0.5, -3.0, -10.0]))


def override_lead(t):
  """radard's low-speed override: the closest stationary return straight ahead, with no camera confirmation."""
  return t.get_RadarState(0.0)


def policy_with(turn=None):
  return rf.TandemLeadPolicy(turn_status=lambda: turn)


def test_stationary_override_lead_off_the_turning_path_is_dropped():
  # 2026-11-15 bookmark: 23 m straight ahead across the intersection while the car turned left at 3 m/s
  wall = track(1, 23.0, v_lead=0.0, v_ego=3.0)
  policy = policy_with()
  lead = policy.lead_one(override_lead(wall), {1: wall}, LEFT_TURN, 0.0, v_ego=3.0)
  assert not lead['present'] and policy.last_action == 'veto_off_path'
  # the same return on a straight path is in the car's way and stays
  kept = policy_with().lead_one(override_lead(wall), {1: wall}, STRAIGHT, 0.0, v_ego=3.0)
  assert kept['present'] and kept['radarTrackId'] == 1


def test_stationary_override_lead_past_a_signalled_turn_is_dropped():
  wall = track(1, 23.1, v_lead=0.0, v_ego=3.0)
  pending = {'side': 'left', 'turning': False, 'distance_m': 22.5}
  policy = policy_with(pending)
  assert not policy.lead_one(override_lead(wall), {1: wall}, STRAIGHT, 0.0, v_ego=3.0)['present']
  assert policy.last_action == 'veto_past_turn'
  near = track(2, 15.0, v_lead=0.0, v_ego=3.0)  # before the side road: still in the way
  assert policy_with(pending).lead_one(override_lead(near), {2: near}, STRAIGHT, 0.0, v_ego=3.0)['present']
  # once turning the map distance is gone; only the path decides
  turning = policy_with({'side': 'left', 'turning': True, 'distance_m': None})
  assert turning.lead_one(override_lead(wall), {1: wall}, STRAIGHT, 0.0, v_ego=3.0)['present']


def test_turn_veto_never_drops_camera_moving_held_or_faster_leads():
  wall = track(1, 23.0, v_lead=0.0, v_ego=3.0)
  pending = {'side': 'left', 'turning': False, 'distance_m': 10.0}
  # the camera sees it: a stopped car in the intersection stays the lead
  assert policy_with(pending).lead_one(camera_lead(wall), {1: wall}, LEFT_TURN, 0.9, v_ego=3.0)['present']
  # moving, or the car above radard's low-speed range, or no speed given (old callers): untouched
  mover = track(2, 23.0, v_lead=3.0, v_ego=3.0)
  assert policy_with(pending).lead_one(override_lead(mover), {2: mover}, LEFT_TURN, 0.0, v_ego=3.0)['present']
  assert policy_with(pending).lead_one(override_lead(wall), {1: wall}, LEFT_TURN, 0.0, v_ego=5.0)['present']
  assert policy_with(pending).lead_one(override_lead(wall), {1: wall}, LEFT_TURN, 0.0)['present']
  # a car the camera confirmed, then lost, is held through the turn
  policy = policy_with(pending)
  policy.lead_one(camera_lead(wall), {1: wall}, STRAIGHT, 0.9, v_ego=3.0)
  held = policy.lead_one(NO_LEAD, {1: wall}, STRAIGHT, 0.2, v_ego=3.0)
  assert held['present'] and policy.last_action == 'hold'


def test_a_picked_up_car_stays_the_lead_as_it_slows_to_a_stop():
  # A car the camera never rates above 0.5 (trailer, motorcycle, glare) brakes for a light ahead of us.
  policy, car = rf.TandemLeadPolicy(), track(7, 50.0, v_lead=15.0)
  assert run(policy, rf.PICKUP_MIN_FRAMES, NO_LEAD, {7: car})['present']
  d, lead = 50.0, None
  for v_lead in (10.0, 6.0, 2.9, 1.0, 0.0, 0.0):
    d -= 2.0
    car.update(d, 0.0, v_lead - 10.0, v_lead)
    lead = policy.lead_one(NO_LEAD, {7: car}, STRAIGHT, 0.2)
    assert lead['present'] and lead['radarTrackId'] == 7, v_lead
  assert policy.last_action == 'pickup'


def test_a_stopped_object_taking_the_picked_up_slot_is_not_followed():
  # The followed car leaves and the radar reuses its slot for a sign 30 m nearer: that is not the same car.
  policy, car = rf.TandemLeadPolicy(), track(7, 60.0, v_lead=15.0)
  assert run(policy, rf.PICKUP_MIN_FRAMES, NO_LEAD, {7: car})['present']
  car.update(30.0, 0.0, -20.0, 0.0)
  assert not policy.lead_one(NO_LEAD, {7: car}, STRAIGHT, 0.2)['present']
  assert not run(policy, 5, NO_LEAD, {7: car})['present']        # and it is not taken up again while stopped


def test_a_car_that_was_never_picked_up_still_needs_speed():
  policy, car = rf.TandemLeadPolicy(), track(3, 40.0, v_lead=2.0)
  assert not run(policy, rf.PICKUP_MIN_FRAMES * 3, NO_LEAD, {3: car})['present']
