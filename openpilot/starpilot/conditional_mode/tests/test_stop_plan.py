"""A stop is a plan that reaches standstill; a passed or pulled-away stop releases (2026-11-17 drives b0 and b2)."""
from dataclasses import replace

from openpilot.starpilot.conditional_mode.stop import StopLead, StopLightDetector
from openpilot.starpilot.conditional_mode.tests.test_stop import frame


def plan(sample, min_speed, accel=0.0):
  return replace(sample, model_min_speed_mps=min_speed, model_desired_accel=accel)


def committed_short_stop():
  detector = StopLightDetector()
  for tick in range(30):
    detector.step(plan(frame(tick), 0.3))
  assert detector.committed
  return detector


def test_a_plan_that_only_slows_is_not_a_stop():
  # b0 seg 10: 26 mph, the model planned down to ~8 mph (path 83 m, under speed x 9 s) and the stop held 6.5 s.
  detector = StopLightDetector()
  for tick in range(40):
    result = detector.step(plan(frame(tick, speed=11.6, horizon=83.), 3.7))
    assert not detector.committed and not result.light_detected
  for tick in range(40, 80):
    detector.step(plan(frame(tick, speed=11.6, horizon=83.), 0.4))
  assert detector.committed


def test_unknown_plan_keeps_the_old_behaviour():
  detector = StopLightDetector()
  for tick in range(30):
    detector.step(frame(tick))
  assert detector.committed


def test_driving_past_the_stop_point_releases_it():
  # b2 seg 24: committed 11 m ahead while crawling behind an ambulance, then held for 70 s of 10 mph traffic
  # because a path behind a lead never reaches the 54 m clear distance.
  detector = committed_short_stop()
  start = detector.commit_start_m
  tick = 30
  while detector.committed and tick < 400:
    detector.step(plan(frame(tick, speed=4.5, horizon=45.), 4.0))
    tick += 1
  assert not detector.committed
  assert start + 8.0 <= detector.commit_travel_m <= start + 8.0 + 4.5 * 1.0   # past the point, then the clear dwell


def test_a_stop_still_planned_past_the_point_is_kept():
  detector = committed_short_stop()
  for tick in range(30, 400):
    detector.step(plan(frame(tick, speed=4.5, horizon=45.), 0.3))
  assert detector.committed


def stopped_behind(detector, lead_speed, *, accel, should_stop=False, start=30, ticks=40):
  result = None
  for tick in range(start, start + ticks):
    lead = StopLead(True, 6.0 + 0.05 * (tick - start) * lead_speed, lead_speed, True, 0.99, True)
    result = detector.step(plan(frame(tick, speed=0., standstill=True, horizon=15., lead=lead,
                                      model_should_stop=should_stop), 0.0, accel))
  return result


def test_lead_pulling_away_releases_a_standstill_stop():
  # b2 seg 25: stopped behind the ambulance, the model asked +0.5 m/s2 with shouldStop off while it pulled away
  # at 0.7-2.5 m/s; the car sat 5.5 s until the driver pressed the gas.
  detector = committed_short_stop()
  stopped_behind(detector, 0.0, accel=-0.3, ticks=10)
  assert detector.committed
  result = stopped_behind(detector, 1.2, accel=0.5, start=40)
  assert not detector.committed and not result.standstill_hold


def test_far_lead_pulling_away_keeps_the_hold():
  detector = committed_short_stop()
  for tick in range(30, 70):
    lead = StopLead(True, 30.0, 12.0, True, 0.99, True)
    result = detector.step(plan(frame(tick, speed=0., standstill=True, horizon=15., lead=lead), 0.0, 0.5))
  assert detector.committed and result.standstill_hold


def test_lead_that_does_not_move_or_a_model_stop_keeps_the_hold():
  for lead_speed, accel, should_stop in ((0.0, 0.5, False), (1.2, 0.0, False), (1.2, 0.5, True)):
    detector = committed_short_stop()
    result = stopped_behind(detector, lead_speed, accel=accel, should_stop=should_stop)
    assert detector.committed and result.standstill_hold, (lead_speed, accel, should_stop)
