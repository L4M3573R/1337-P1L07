import math

from openpilot.starpilot.stop_line.fusion import (MAX_AGE_S, MAX_SIGMA_M, MISS_LIMIT, Inference, StopPointFuser, choose,
                                                  decode)


def test_decode_maps_the_model_row():
  inf = decode((0.0, 0.25, math.log(0.01)))
  assert inf.probability == 0.5
  assert math.isclose(inf.distance_m, 30.0)
  assert math.isclose(inf.sigma_m, math.sqrt(2) * 1.2)


def test_frames_fuse_into_one_point_that_holds_between_inferences():
  f = StopPointFuser()
  f.update(Inference(0.9, 50.0, 6.0), odometer_m=100.0, now_s=0.0)
  f.update(Inference(0.9, 40.0, 6.0), odometer_m=112.0, now_s=0.25)  # says 152 m, first said 150 m
  left, sigma = f.distance(112.0)
  assert 38.0 < left < 40.0 and sigma < 6.0
  # driving on without a new inference just counts the metres down
  assert math.isclose(f.distance(120.0)[0], left - 8.0)


def test_a_far_disagreeing_measurement_restarts_the_estimate():
  f = StopPointFuser()
  for i in range(5):
    f.update(Inference(0.9, 30.0 - i, 2.0), odometer_m=float(i), now_s=i * 0.25)
  f.update(Inference(0.9, 90.0, 2.0), odometer_m=5.0, now_s=1.25)
  assert math.isclose(f.distance(5.0)[0], 90.0) and f.samples == 1


def test_no_stop_for_a_few_inferences_drops_it_and_passing_it_drops_it():
  f = StopPointFuser()
  f.update(Inference(0.9, 30.0, 3.0), 0.0, 0.0)
  for i in range(MISS_LIMIT):
    f.update(Inference(0.1, 0.0, 3.0), 0.0, 0.25 * (i + 1))
  assert f.distance(0.0) is None
  f.update(Inference(0.9, 30.0, 3.0), 0.0, 2.0)
  f.passed(34.0, speed_mps=8.0)
  assert f.distance(34.0) is None


def test_choose_uses_chestnut_when_its_plan_stops_and_ours_before_that():
  f = StopPointFuser()
  f.update(Inference(0.9, 40.0, 3.0), 0.0, 0.0)
  c = choose(f, 0.0, 0.1, model_stop_m=55.0, model_big=True, model_should_stop=True, ours_available=True)
  assert (c.source, c.distance_m, round(c.ours_m)) == ('chestnut', 55.0, 40)
  far = choose(f, 0.0, 0.1, model_stop_m=None, model_big=True, model_should_stop=False, ours_available=True)
  assert (far.source, round(far.distance_m), far.reason) == ('ours', 40, 'plan_clear')
  small = choose(f, 0.0, 0.1, model_stop_m=30.0, model_big=False, model_should_stop=True, ours_available=False)
  assert small.source == 'driving_model'
  stale = choose(f, 0.0, MAX_AGE_S + 0.5, model_stop_m=None, model_big=True, model_should_stop=False, ours_available=True)
  assert (stale.source, stale.reason) == ('none', 'stale')
  unsure = StopPointFuser()
  unsure.update(Inference(0.9, 40.0, MAX_SIGMA_M * 2), 0.0, 0.0)
  c = choose(unsure, 0.0, 0.1, model_stop_m=None, model_big=True, model_should_stop=False, ours_available=True)
  assert (c.source, c.reason, round(c.ours_m)) == ('none', 'unsure', 40)
