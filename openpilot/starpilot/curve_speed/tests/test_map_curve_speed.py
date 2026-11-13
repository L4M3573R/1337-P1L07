import math

import numpy as np
import pytest

from openpilot.starpilot.curve_speed import map_curves, map_target
from openpilot.starpilot.curve_speed.curve_memory import CurveMemory, CurvePass, PassTracker, Sample, verdict_for
from openpilot.starpilot.curve_speed.curvemapd import Shadow, mode

MPH = 0.44704


def road_with_curve(radius, straight=200.0, arc_deg=90.0, step=2.0):
  """Straight north, then a left arc of the given radius, then straight again (metres east/north)."""
  pts = [(0.0, y) for y in np.arange(0.0, straight, step)]
  cx, cy = -radius, straight
  for a in np.arange(0.0, math.radians(arc_deg), step / radius):
    pts.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
  end = np.array(pts[-1])
  heading = np.array([-math.sin(math.radians(arc_deg)), math.cos(math.radians(arc_deg))])
  pts += [tuple(end + heading * d) for d in np.arange(step, straight, step)]
  return np.array(pts)


@pytest.mark.parametrize('radius', [80.0, 200.0, 400.0])
def test_curvature_of_an_arc(radius):
  distances, k, pts = map_curves.curvature_profile(road_with_curve(radius))
  curves = map_curves.find_curves(distances, k, pts)
  assert len(curves) == 1
  c = curves[0]
  assert c.direction == 1
  assert c.curvature == pytest.approx(1 / radius, rel=0.12)
  assert 160 <= c.start_m <= 230  # the bend starts 200 m ahead (the 60 m chord blurs the edge either way)
  assert c.start_m < c.apex_m < c.end_m
  arc_mid = 200.0 + radius * math.radians(45.0)
  assert abs(c.apex_m - arc_mid) < 0.25 * radius * math.radians(90.0)


def test_straight_road_and_gentle_bends_are_not_curves():
  straight = np.array([(0.0, y) for y in np.arange(0.0, 800.0, 5.0)])
  d, k, p = map_curves.curvature_profile(straight)
  assert map_curves.find_curves(d, k, p) == []
  d, k, p = map_curves.curvature_profile(road_with_curve(900.0))  # 900 m radius: under the 500 m threshold
  assert map_curves.find_curves(d, k, p) == []


def test_polyline_corners_do_not_read_as_sharp_curves():
  # a gentle 400 m bend drawn with sparse vertices every 40 m (typical map data)
  dense = road_with_curve(400.0, step=1.0)
  sparse = dense[::40]
  d, k, p = map_curves.curvature_profile(sparse)
  curves = map_curves.find_curves(d, k, p)
  assert curves and max(c.curvature for c in curves) < 1 / 250


def test_highway_curves_are_limited_by_comfort_tight_ones_by_steering():
  highway = map_target.apex_speed(1 / 700, comfort=1.8)
  assert highway.limit == 'comfort' and highway.apex_speed == pytest.approx(math.sqrt(1.8 * 700))
  tight = map_target.apex_speed(1 / 40, comfort=2.6)   # a ramp loop
  assert tight.limit == 'steering'
  assert tight.apex_speed ** 2 / 40 <= map_target.torque_cap(tight.apex_speed) + 1e-3
  struggled = map_target.apex_speed(1 / 40, comfort=2.6, margin=map_target.STRUGGLE_MARGIN)
  assert struggled.apex_speed < tight.apex_speed


def profile(curvature_at, length=1000.0, step=10.0):
  d = np.arange(0.0, length + 1e-6, step)
  return d, np.array([curvature_at(x) for x in d], dtype=float)


COMFORT = lambda k: 2.0  # noqa: E731
EXACT = None  # adjust=None applies the unknown-map margin everywhere


def test_far_curves_are_met_by_easing_off_not_braking():
  bend = lambda lo, hi: (lambda x: 1 / 300 if lo <= x <= hi else 0.0)  # noqa: E731
  far = map_target.plan(*profile(bend(800, 900)), COMFORT)
  near = map_target.plan(*profile(bend(100, 200)), COMFORT)
  in_curve = float(far.limit[far.binding])
  assert far.now > near.now > in_curve
  assert map_target.plan(*profile(lambda x: 0.0), COMFORT).binding is None  # a straight road limits nothing
  assert math.isinf(map_target.plan(*profile(lambda x: 0.0), COMFORT).now)


def test_tightening_curve_is_eased_into_progressively():
  # wide at the start (600 m radius), tightening steadily to 120 m at the end
  tighten = lambda x: 0.0 if x < 300 else 1 / 600 + (1 / 120 - 1 / 600) * (x - 300) / 400 if x <= 700 else 0.0  # noqa: E731
  d, k = profile(tighten)
  p = map_target.plan(d, k, COMFORT)
  inside = (d >= 320) & (d <= 690)  # interior: the synthetic edges are abrupt steps (correctly jerk-limited)
  assert np.all(np.diff(p.speed[inside]) <= 1e-6)  # keeps easing off as it tightens
  tight_end = int(np.argmax(np.abs(k)))
  assert p.speed[tight_end] <= map_target.apex_speed(abs(k[tight_end]) * map_target.UNKNOWN_MAP_MARGIN, 2.0).apex_speed + 1e-6
  # one tightness number for the whole curve (80th percentile) would have planned the tight end too fast
  one_number = map_target.apex_speed(np.percentile(np.abs(k[(d >= 300) & (d <= 700)]), 80) * map_target.UNKNOWN_MAP_MARGIN, 2.0)
  assert one_number.apex_speed > p.speed[tight_end] + 0.5


def test_opening_curve_lets_the_car_speed_back_up():
  # tight at the start (120 m radius), opening to 600 m
  opening = lambda x: 0.0 if x < 300 else 1 / 120 - (1 / 120 - 1 / 600) * min((x - 300) / 300, 1.0) if x <= 700 else 0.0  # noqa: E731
  d, k = profile(opening)
  p = map_target.plan(d, k, COMFORT)
  assert p.limit[(d >= 600) & (d <= 690)].min() > p.limit[(d >= 300) & (d <= 320)].max()  # faster where it has opened


def test_s_bend_transition_is_limited_by_jerk():
  # left 150 m radius, then straight into right 150 m radius: the switch is a quick change in tightness
  s_bend = lambda x: (1 / 150 if 300 <= x < 400 else -1 / 150 if 400 <= x <= 500 else 0.0)  # noqa: E731
  d, k = profile(s_bend)
  p = map_target.plan(d, k, COMFORT)
  switch = (d >= 390) & (d <= 410)
  assert 'jerk' in {p.reason[i] for i in np.flatnonzero(switch)}
  i = int(np.flatnonzero(switch)[0])
  dkds = abs(np.gradient(k * map_target.UNKNOWN_MAP_MARGIN, d))[i]
  assert p.limit[i] == pytest.approx(max((map_target.JERK_MAX / dkds) ** (1 / 3), map_target.MIN_SPEED), rel=1e-6)


def test_remembered_curves_use_their_own_tightness_and_comfort():
  d, k = profile(lambda x: 1 / 300 if 50 <= x <= 120 else 0.0, length=400.0)
  curve = map_curves.MapCurve(50.0, 80.0, 120.0, 1 / 300, 1)
  unknown = map_target.plan(d, k, COMFORT, map_target.adjustments(d, [curve]))
  known = map_target.plan(d, k, COMFORT, map_target.adjustments(d, [curve], lambda c: (1 / 400, 2.4, map_target.TORQUE_MARGIN)))
  tighter = map_target.plan(d, k, COMFORT, map_target.adjustments(d, [curve], lambda c: (1 / 200, 2.0, map_target.TORQUE_MARGIN)))
  assert known.now > unknown.now > tighter.now  # measured looser + own comfort; measured tighter than the map


def sample(t, v=25.0, k=0.0, lat=True, long=True, brake=False, gas=False, steer=False, sat=False, cruise=True, torque=0.0,
           command=0.0, lead_gap_s=math.inf, lead_speed=math.nan):
  # torque: the driver's (signed like curvature); command: the comma's steering command
  return Sample(t, v, k, lat, long, brake, gas, steer, sat, cruise, 34.0, -117.0, 0.0, torque, command, lead_gap_s, lead_speed)


# this car reports driver torque opposite to curvature, so steering INTO a left (positive) curve reads negative
INTO_CURVE = -200.0


def drive(events_at=None, k=1 / 300, manual=False, tracker=None):
  """Approach straight for 6 s, curve for 4 s, straight again; events_at maps time -> overrides."""
  tracker = tracker or PassTracker()
  result = None
  for i in range(260):
    t = i * 0.05
    curving = 6.0 <= t < 10.0
    kw = {'k': k if curving else 0.0, 'long': not manual, 'cruise': not manual, 'lat': not manual}
    kw.update((events_at or {}).get(round(t, 2), {}))
    out = tracker.add(sample(t, **kw))
    result = out or result
  return result


def test_comma_drove_it_without_takeover_is_confirmed():
  p = drive()
  assert p is not None and p.verdict == 'confirmed' and p.comma_speed and p.comma_steering
  assert p.lateral == pytest.approx(25.0 ** 2 / 300, rel=0.05)


def test_takeovers_are_recognized():
  brake = {round(5.0 + i * 0.05, 2): {'brake': True, 'long': i == 0} for i in range(20)}
  assert drive(brake).verdict == 'too fast'
  cancel = {round(7.0 + i * 0.05, 2): {'cruise': False, 'long': False} for i in range(60)}
  assert drive(cancel).verdict == 'too fast'
  gas = {round(7.0 + i * 0.05, 2): {'gas': True} for i in range(10)}
  assert drive(gas).verdict == 'too slow'
  steer = {round(8.0 + i * 0.05, 2): {'steer': True, 'torque': INTO_CURVE} for i in range(12)}
  assert drive(steer).verdict == 'struggled'
  saturated = {round(8.0 + i * 0.05, 2): {'sat': True} for i in range(10)}
  assert drive(saturated).verdict == 'struggled'


def test_hands_on_the_wheel_is_not_a_takeover():
  resting = {round(6.0 + i * 0.05, 2): {'steer': True, 'torque': 0.0} for i in range(80)}
  assert drive(resting).verdict == 'confirmed'
  pushing_wide = {round(8.0 + i * 0.05, 2): {'steer': True, 'torque': -INTO_CURVE} for i in range(30)}
  assert drive(pushing_wide).verdict == 'confirmed'  # about the line, not the speed
  brief = {round(8.0 + i * 0.05, 2): {'steer': True, 'torque': INTO_CURVE} for i in range(5)}
  assert drive(brief).verdict == 'confirmed'  # under half a second


def test_torque_direction_is_learned_from_hand_steering():
  tracker = PassTracker()
  assert tracker.torque_sign == -1  # this car, until told otherwise
  # a car whose torque matches curvature: steer by hand through curves a while
  for i in range(400):
    tracker.add(sample(i * 0.05, k=1 / 100, lat=False, long=False, cruise=False, steer=True, torque=200.0))
  assert tracker.torque_sign == 1
  steer_same_sign = {round(8.0 + i * 0.05, 2): {'steer': True, 'torque': 200.0} for i in range(12)}
  assert drive(steer_same_sign, tracker=tracker).verdict == 'struggled'


def test_driving_yourself_teaches_your_comfort_and_braking_is_not_a_complaint():
  assert drive(manual=True).verdict == 'manual'
  brake = {round(5.0 + i * 0.05, 2): {'brake': True} for i in range(20)}
  assert drive(brake, manual=True).verdict == 'manual'
  assert verdict_for({'brake', 'gas'}, comma_speed=True) == 'too fast'  # safety wins over a gas press


def test_memory_learns_each_takeover(tmp_path):
  memory = CurveMemory(tmp_path / 'curves.json')
  base = {'start_t': 0.0, 'duration': 4.0, 'curvature': 1 / 300, 'apex_speed': 25.0, 'latitude': 34.0, 'longitude': -117.0, 'bearing': 90.0,
              'comma_speed': True, 'comma_steering': True}
  record = memory.learn(CurvePass(lateral=2.1, verdict='confirmed', **base))
  start = record.comfort
  # a gentle pass nobody slowed for never lowers comfort; one where the driver slowed sets it
  memory.learn(CurvePass(lateral=0.8, verdict='manual', **base))
  assert record.comfort == start
  memory.learn(CurvePass(lateral=1.9, verdict='manual', slowed=True, **base))
  assert record.comfort < start
  start = record.comfort
  # braking in it: learns the speed you braked down to at the tightest point
  memory.learn(CurvePass(lateral=2.1, verdict='too fast', chosen_lateral=1.5, **base))
  assert record.comfort == pytest.approx(1.5) and record.too_fast == 1
  # pressing on through it: learns the faster speed you actually took it at
  memory.learn(CurvePass(lateral=2.3, verdict='too slow', chosen_lateral=2.2, **base))
  assert record.comfort == pytest.approx(2.3) and record.too_slow == 1
  assert record.margin == map_target.TORQUE_MARGIN
  memory.learn(CurvePass(lateral=2.0, verdict='struggled', **base))
  assert record.margin == map_target.STRUGGLE_MARGIN
  # the same curve from the other direction is a different curve
  assert memory.find(34.0, -117.0, 270.0) is None
  assert memory.find(34.0001, -117.0001, 95.0) is record
  memory.save()
  again = CurveMemory.load(tmp_path / 'curves.json')
  assert again.find(34.0, -117.0, 90.0) == record


def test_a_car_ahead_never_teaches_curve_comfort(tmp_path):
  memory = CurveMemory(tmp_path / 'curves.json')
  base = {'start_t': 0.0, 'duration': 4.0, 'curvature': 1 / 300, 'apex_speed': 20.0, 'latitude': 34.0, 'longitude': -117.0, 'bearing': 90.0,
          'comma_speed': True, 'comma_steering': True}
  record = memory.learn(CurvePass(lateral=2.0, verdict='manual', slowed=True, **base))
  start, general = record.comfort, memory.comfort.comfort(1 / 300)
  memory.learn(CurvePass(lateral=1.2, verdict='following', slowed=True, **base))  # crawled through behind someone
  assert record.comfort == start and memory.comfort.comfort(1 / 300) == pytest.approx(general)
  memory.learn(CurvePass(lateral=2.6, verdict='following', **base))  # but it does show you are fine at least that fast
  assert record.comfort > start


def test_braking_or_gas_behind_a_car_is_about_the_car_not_the_curve():
  behind = {round(t * 0.05, 2): {'lead_gap_s': 1.5, 'lead_speed': 22.0} for t in range(260)}
  brake = {**behind, **{round(5.0 + i * 0.05, 2): {'brake': True, 'long': i == 0, 'lead_gap_s': 1.5, 'lead_speed': 22.0} for i in range(20)}}
  p = drive(brake)
  assert p.following and p.verdict == 'following' and 'brake' not in p.events
  gas = {**behind, **{round(7.0 + i * 0.05, 2): {'gas': True, 'lead_gap_s': 1.5, 'lead_speed': 22.0} for i in range(10)}}
  assert drive(gas).verdict == 'following'
  far_ahead = {round(t * 0.05, 2): {'lead_gap_s': 6.0, 'lead_speed': 30.0} for t in range(260)}
  assert drive({**far_ahead, **{round(5.0 + i * 0.05, 2): {'brake': True, 'long': i == 0, 'lead_gap_s': 6.0, 'lead_speed': 30.0}
                                for i in range(20)}}).verdict == 'too fast'  # a car far ahead did not set the speed


def test_corrupt_memory_starts_fresh(tmp_path):
  path = tmp_path / 'curves.json'
  path.write_text('{"version": 1, "comfort": null, "curves": {"a": {"latitude": 1}}}')
  assert CurveMemory.load(path).records == {}
  path.write_text('not json')
  assert CurveMemory.load(path).records == {}


def test_shadow_links_what_it_planned_to_the_curve_driven(tmp_path):
  shadow = Shadow(CurveMemory(tmp_path / 'c.json'))
  d, k, pts = map_curves.curvature_profile(road_with_curve(200.0, straight=300.0))  # a real (smoothed) map profile
  curves = map_curves.find_curves(d, k, pts)
  plan = shadow.plan(1.0, 34.0, -117.0, map_curves.RoadProfile(d, k, pts, curves), speed=31.0)
  assert plan is not None and plan['would_slow'] and not plan['known'] and plan['limit'] == 'comfort'
  assert curves[0].start_m <= plan['distance_m'] <= curves[0].end_m and len(plan['plan_every_50m']) == 11
  line = shadow.finish(CurvePass(5.0, 4.0, 1 / 200, 2.4, 22.0, plan['apex_lat'], plan['apex_lon'], 0.0, True, True, 'confirmed'), False)
  assert line['map_seen'] and line['would_have_slowed'] and line['planned_apex_speed'] == plan['apex_speed']


def test_mode_defaults_to_shadow(tmp_path):
  assert mode(tmp_path) == 'shadow'
  (tmp_path / 'curve_map.json').write_text('{"mode": "off"}')
  assert mode(tmp_path) == 'off'
  (tmp_path / 'curve_map.json').write_text('{"mode": "drive"}')  # anything else stays in shadow
  assert mode(tmp_path) == 'shadow'


def test_steering_capability_learns_from_the_comma_steering():
  from openpilot.starpilot.curve_speed.steering_capability import SteeringCapability
  cap = SteeringCapability()
  v = 30 * MPH
  start = cap.cap(v)
  for _ in range(3000):  # this road and season: the comma gets 20% less cornering per unit torque than the table
    assert cap.observe(v, 0.8 * start * 0.5, 0.5)
  assert cap.cap(v) < start and cap.cap(v) == pytest.approx(0.8 * start, rel=0.05)
  assert not cap.observe(v, 2.0, 0.05)    # too little torque to mean anything
  assert not cap.observe(1.0, 2.0, 0.5)   # parking speed
  again = SteeringCapability.load(cap.document())
  assert again.cap(v) == pytest.approx(cap.cap(v), abs=1e-3)  # saved to 4 decimals
  assert SteeringCapability.load({'version': 1, 'gain': [99.0] * 7, 'count': [1] * 7}).cap(v) == pytest.approx(start)


def test_plan_uses_the_learned_steering_capability():
  d, k = profile(lambda x: 1 / 40 if 200 <= x <= 260 else 0.0, length=400.0)  # a ramp loop: steering-limited
  table = map_target.plan(d, k, lambda kk: 2.6)
  weaker = map_target.plan(d, k, lambda kk: 2.6, cap_of=lambda v: 0.7 * float(np.interp(v, map_target.TORQUE_CAP_V, map_target.TORQUE_CAP_A)))
  assert 'steering' in table.reason and float(weaker.limit.min()) < float(table.limit.min())


def live_drive(shadow, measured_k, command=0.3, seconds=6.0, v=20.0):
  """Drive through the planned curve (it starts at the car): straight approach is skipped, curve samples at 20 Hz."""
  line = None
  for i in range(int(seconds * 20) + 40):
    t = 2.0 + i * 0.05
    in_curve = i < seconds * 20
    out = shadow.add(sample(t, v=v, k=measured_k if in_curve else 0.0, command=command), False)
    line = out or line
  return line


def test_live_check_lowers_the_plan_when_the_curve_is_tighter_than_predicted(tmp_path):
  d, k = profile(lambda x: 1 / 300, length=600.0)  # predicted: 300 m radius all the way
  shadow = Shadow(CurveMemory(tmp_path / 'c.json'))
  shadow.live_plan, shadow.live_t, shadow.traveled = map_target.plan(d, k, COMFORT), 2.0, 0.0
  tight = live_drive(shadow, 1 / 150)   # measures 150 m: twice as tight
  assert tight['would_lower_mid_curve'] and tight['live']['why'] == 'tighter than predicted'
  assert tight['live']['lowered_to'] < 20.0 and tight['live']['max_tighter'] > 1.8
  shadow2 = Shadow(CurveMemory(tmp_path / 'd.json'))
  shadow2.live_plan, shadow2.live_t, shadow2.traveled = map_target.plan(d, k, COMFORT), 2.0, 0.0
  as_predicted = live_drive(shadow2, 1 / 300, v=15.0)
  assert not as_predicted['would_lower_mid_curve'] and 0.9 < as_predicted['live']['max_tighter'] < 1.1


def drive_shaped_curve(k_at, comfort=2.0, approach=30.0, length=400.0, cap_speed=None):
  """Approach at a steady speed, then drive a curve whose tightness follows k_at(fraction), holding a steady sideways
  feel (the speed follows the tightness), like a driver easing off as it tightens and picking up as it opens."""
  tracker, t, out = PassTracker(), 0.0, None
  for _ in range(120):  # 6 s approach
    tracker.add(sample(t, v=approach, k=0.0, long=False, lat=False, cruise=False))
    t += 0.05
  s = 0.0
  while s < length:
    k = k_at(s / length)
    v = min(math.sqrt(comfort / k), cap_speed or approach)
    out = tracker.add(sample(t, v=v, k=k, long=False, lat=False, cruise=False)) or out
    s += v * 0.05
    t += 0.05
  for _ in range(40):
    out = tracker.add(sample(t, v=approach, k=0.0, long=False, lat=False, cruise=False)) or out
    t += 0.05
  return out


def test_tightening_curve_is_learned_point_by_point(tmp_path):
  tighten = lambda f: 1 / 600 + (1 / 120 - 1 / 600) * f  # noqa: E731
  p = drive_shaped_curve(tighten)
  assert p is not None and p.verdict == 'manual' and len(p.profile) == 11
  ks = [q[1] for q in p.profile]
  assert all(b >= a - 1e-9 for a, b in zip(ks, ks[1:], strict=False))  # it really got tighter
  assert all(q[2] == pytest.approx(2.0, rel=0.1) for q in p.profile if q[3])  # the speed you chose held 2.0 sideways
  memory = CurveMemory(tmp_path / 'c.json')
  before = [memory.comfort.comfort(1 / r) for r in (500, 200, 130)]
  record = memory.learn(p)
  after = [memory.comfort.comfort(1 / r) for r in (500, 200, 130)]
  assert sum(abs(a - b) > 1e-6 for a, b in zip(before, after, strict=True)) >= 2  # one curve taught several tightnesses
  # next time, the plan for this curve follows how you drove it: wide start fast, tight end slow
  d, k = profile(lambda x: tighten(min(max((x - 200) / 400, 0.0), 1.0)) if 200 <= x <= 600 else 0.0)
  curve = map_curves.MapCurve(200.0, 580.0, 600.0, float(np.percentile(k[(d >= 200) & (d <= 600)], 80)), 1)
  remembered = (record.curvature, record.comfort, record.margin, record.profile)
  plan = map_target.plan(d, k, memory.comfort.comfort, map_target.adjustments(d, [curve], lambda c: remembered, k))
  start_i, end_i = int(np.searchsorted(d, 230)), int(np.searchsorted(d, 590))
  assert plan.limit[start_i] > plan.limit[end_i]
  assert plan.limit[end_i] == pytest.approx(math.sqrt(2.0 / tighten(0.975)), rel=0.15)


def test_opening_curve_speeding_up_only_raises_comfort(tmp_path):
  opening = lambda f: 1 / 120 + (1 / 600 - 1 / 120) * f  # noqa: E731
  p = drive_shaped_curve(opening, approach=24.0)
  chosen = [q[3] for q in p.profile]
  assert chosen[0] == 1.0 and chosen[-1] == 0.0  # slowed at the tight start; back at speed as it opened
  memory = CurveMemory(tmp_path / 'c.json')
  wide = memory.comfort.comfort(1 / 600)
  memory.learn(p)
  assert memory.comfort.comfort(1 / 600) >= wide - 1e-9  # the opened-up part never lowers comfort


def _ramp_right(start_xy, length=400.0, radius=250.0, step=5.0):
  """A ramp leaving northbound traffic and bending away to the right (east)."""
  x0, y0 = start_xy
  pts = [(x0 + radius - radius * math.cos(a), y0 + radius * math.sin(a)) for a in np.arange(0.0, length / radius, step / radius)]
  return np.array(pts)


def _match_north(lines):
  from openpilot.starpilot.navigation import side_road
  return side_road._match(lines, np.array([0.0, 1.0]))


def test_split_follows_the_highway_unless_the_blinker_picks_the_exit():
  from openpilot.starpilot.navigation import side_road
  from openpilot.starpilot.navigation.road_tiles import LINK, MOTORWAY
  highway_in = np.array([(0.0, y) for y in np.arange(-50.0, 300.0 + 1e-6, 5.0)])
  highway_on = np.array([(0.0, y) for y in np.arange(300.0, 1000.0 + 1e-6, 5.0)])
  ramp = _ramp_right((0.0, 300.0))
  lines = [(MOTORWAY, highway_in), (LINK, ramp), (MOTORWAY, highway_on)]  # the ramp listed first, as map data may
  start = _match_north(lines)
  straight, _ = side_road._path(lines, start, np.array([0.0, 1.0]), 900.0)
  assert abs(straight[-1][0]) < 1.0 and straight[-1][1] > 900.0  # stayed on the highway (the old code took the ramp)
  exit_path, _ = side_road._path(lines, start, np.array([0.0, 1.0]), 900.0, prefer='right')
  assert exit_path[-1][0] > 50.0  # right blinker: followed the ramp off to the east
  wrong_side, _ = side_road._path(lines, start, np.array([0.0, 1.0]), 900.0, prefer='left')
  assert abs(wrong_side[-1][0]) < 1.0  # a left blinker does not take a right exit


def test_blinker_takes_a_ramp_that_leaves_the_middle_of_the_road_line():
  from openpilot.starpilot.navigation import side_road
  from openpilot.starpilot.navigation.road_tiles import LINK, MOTORWAY
  highway = np.array([(0.0, y) for y in np.arange(-50.0, 1200.0 + 1e-6, 5.0)])  # one long line through the exit
  ramp = _ramp_right((0.0, 400.0))
  lines = [(MOTORWAY, highway), (LINK, ramp)]
  start = _match_north(lines)
  straight, _ = side_road._path(lines, start, np.array([0.0, 1.0]), 900.0)
  assert abs(np.array(straight)[:, 0]).max() < 1.0
  exit_path, _ = side_road._path(lines, start, np.array([0.0, 1.0]), 900.0, prefer='right')
  assert np.array(exit_path)[-1][0] > 50.0
  far = [(MOTORWAY, highway), (LINK, _ramp_right((0.0, 900.0)))]  # an exit beyond 600 m is not taken yet
  later, _ = side_road._path(far, _match_north(far), np.array([0.0, 1.0]), 1100.0, prefer='right')
  assert abs(np.array(later)[:, 0]).max() < 1.0


class _Line:
  def __init__(self, y):
    self.y = y


class _Xs:
  def __init__(self, values):
    self.x, self.z = values, values


class _Model:
  """A modelV2-like object: a path along x with a constant yaw rate, and lane edges/lines."""

  def __init__(self, curvature=0.0, v=25.0, right_edge=1.8, left_edge=5.4, probs=(0.9, 0.95, 0.95, 0.1)):
    xs = list(np.linspace(0.0, 190.0, 33))
    self.position = type('P', (), {'x': xs})()
    self.velocity = type('V', (), {'x': [v] * 33})()
    self.orientationRate = type('R', (), {'z': [curvature * v] * 33})()
    self.roadEdges = [_Line([-left_edge]), _Line([right_edge])]
    self.laneLineProbs = list(probs)


def test_camera_reads_curvature_and_lane_position():
  from openpilot.starpilot.curve_speed import camera
  d, k = camera.camera_curvature(_Model(curvature=1 / 200))
  assert camera.near_curvature(d, k) == pytest.approx(1 / 200)
  lane = camera.lane_position(_Model())
  assert lane.rightmost and not lane.leftmost and lane.right_edge_m == pytest.approx(1.8)
  middle = camera.lane_position(_Model(right_edge=5.4, probs=(0.9, 0.95, 0.95, 0.9)))
  assert not middle.rightmost
  assert camera.which_branch(-1 / 250, {'straight': 0.0, 'right': -1 / 250})[0] == 'right'
  assert camera.map_agrees(1 / 300, 1 / 320) and not camera.map_agrees(0.0, 1 / 150)


def test_sign_learner_needs_real_curves_and_finds_the_convention():
  from openpilot.starpilot.curve_speed import camera
  s = camera.SignLearner()
  for _ in range(60):
    s.observe(0.0005, 0.01)  # gentle: ignored
  assert s.factor is None
  for _ in range(60):
    s.observe(0.01, -0.008)  # the two signals point opposite ways
  assert s.factor == -1.0


def test_branches_and_camera_check_at_an_exit():
  from openpilot.starpilot.curve_speed import camera
  from openpilot.starpilot.curve_speed.curvemapd import CameraCheck
  from openpilot.starpilot.navigation.road_tiles import LINK, MOTORWAY
  highway = np.array([(0.0, y) for y in np.arange(-50.0, 1200.0 + 1e-6, 5.0)])
  lines = [(MOTORWAY, highway), (LINK, _ramp_right((0.0, 60.0), radius=150.0))]
  branches = map_curves.branches_from_lines(lines, _match_north(lines), np.array([0.0, 1.0]))
  assert set(branches) == {'straight', 'right'} and branches['straight'].road_class == MOTORWAY
  check = CameraCheck()
  for _ in range(60):  # learned conventions: here camera, measured and map all share left-positive
    check.observe(-0.005, -0.005, -0.005)
  near_right = camera.near_curvature(branches['right'].distances, branches['right'].curvature)
  line = check.check(1.0, 34.0, -117.0, 28.0, 'right', branches, camera.camera_curvature(_Model(curvature=near_right)),
                     camera.lane_position(_Model()))
  assert line['camera_says'] == 'right' and line['lane']['rightmost'] and line['blinker'] == 'right'
  stay = check.check(2.0, 34.0, -117.0, 28.0, None, branches, camera.camera_curvature(_Model(curvature=0.0)),
                     camera.lane_position(_Model(right_edge=5.4, probs=(0.9, 0.95, 0.95, 0.9))))
  assert stay['camera_says'] == 'straight' and not stay['lane']['rightmost']
  plain = {'straight': branches['straight']}
  assert check.check(3.0, 34.0, -117.0, 28.0, None, plain, camera.camera_curvature(_Model(curvature=0.0)),
                     camera.lane_position(_Model())) is None  # nothing to check on a plain straight road
