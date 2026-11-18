"""The car's MPG by speed: hills taken out, thin speeds marked, a speed chosen only when it is clearly better."""
import random

from openpilot.starpilot.efficiency import experiment_learn as el
from openpilot.starpilot.efficiency import speed_curve as sc


def stretch(mph, mpg, pitch=0.0, dur=10.0, dv=0.0):
  v = mph / sc.MPS_TO_MPH
  m = v * dur
  gal = m / el.METERS_PER_MILE / mpg + el.DEFAULT_CLIMB * m * pitch   # a hill costs what the correction takes back out
  return {'dur': dur, 'm': m, 'obd_gal': gal, 'soc_d': 0.0, 'pitch': pitch, 'v0': v, 'v1': v + dv, 'ev': 0.25}


def drives(spec, n_drives=4, per_drive=120):
  rnd = random.Random(3)
  return [[stretch(mph, mpg * rnd.uniform(0.97, 1.03)) for mph, mpg in spec.items() for _ in range(per_drive)]
          for _ in range(n_drives)]


def row(curve, mph):
  return next(r for r in curve['rows'] if r['mph'] == mph)


def test_each_speed_is_measured_and_a_held_speed_is_one_bin():
  curve = sc.speed_curve(drives({61: 50.0, 65: 50.0, 69: 56.0}))
  assert [r['mph'] for r in curve['rows']] == [61, 65, 69] and all(r['enough'] for r in curve['rows'])
  assert abs(row(curve, 65)['est'] - 50.0) < 1.0 and abs(row(curve, 69)['est'] - 56.0) < 1.0 and row(curve, 65)['ev_share'] == 0.25
  held = sc.speed_curve([[stretch(64.6, 50.0), stretch(65.4, 50.0)] * 80 for _ in range(4)])
  assert [r['mph'] for r in held['rows']] == [65]


def test_hills_do_not_decide_the_speed():
  # 61 mph driven only uphill, 65 only downhill, same car: the level MPG is the same
  up = [[stretch(61, 50.0, pitch=0.02) for _ in range(150)] for _ in range(4)]
  down = [[stretch(65, 50.0, pitch=-0.02) for _ in range(150)] for _ in range(4)]
  curve = sc.speed_curve([a + b for a, b in zip(up, down, strict=True)])
  assert abs(row(curve, 61)['est'] - row(curve, 65)['est']) < 1.5


def test_a_faster_speed_that_clearly_saves_is_chosen_only_when_over_is_allowed():
  speeds = sc.table(sc.speed_curve(drives({63: 50.0, 65: 50.0, 67: 56.0})))
  assert sc.choose_u(speeds, 65.2) == 0.0                    # the driver's set speed is the top unless they say so
  assert sc.choose_u(speeds, 65.2, over_mph=2) == -2.0       # 67 measured 12 % better: 2 over
  assert sc.choose_u(speeds, 65.2, over_mph=2, allowed=lambda u: u >= 0) == 0.0
  assert sc.choose_u(speeds, 67) == 2.0 - 2.0                # at 67 nothing in its window is better
  assert sc.choose_u(speeds, 60) == 0.0 and sc.choose_u({}, 65) == 0.0 and sc.choose_u(None, 65) == 0.0


def test_a_slower_speed_must_clearly_save_and_be_worth_its_minutes():
  clear = sc.table(sc.speed_curve(drives({63: 56.0, 65: 50.0})))
  assert sc.choose_u(clear, 65) == 2.0
  close = sc.table(sc.speed_curve(drives({63: 50.8, 65: 50.0})))      # 1.6 %: under the 3 % bar
  assert sc.choose_u(close, 65) == 0.0
  noisy = sc.table(sc.speed_curve([[stretch(63, 70.0)] * 120 + [stretch(65, 50.0)] * 120] +
                                  [[stretch(63, 47.0)] * 120 + [stretch(65, 50.0)] * 120 for _ in range(3)]))
  assert sc.choose_u(noisy, 65) == 0.0                                 # one great drive is not the car


def test_thin_speeds_are_listed_for_driving_least_driven_first_and_never_judged():
  d = drives({65: 50.0})
  d[0] += [stretch(63, 80.0) for _ in range(20)]   # 0.4 mile on one drive
  curve = sc.speed_curve(d)
  speeds = sc.table(curve)
  assert not row(curve, 63)['enough'] and sc.choose_u(speeds, 65) == 0.0
  assert sc.thin_us(speeds, 65) == [1.0, 2.0]                 # 64 has nothing, 63 a little
  assert sc.thin_us(speeds, 65, over_mph=2) == [-1.0, 1.0, -2.0, 2.0]
  assert sc.thin_us(sc.table(sc.speed_curve(drives({63: 50, 64: 50, 65: 50}))), 65) == []
  assert any('63 mph*' in line for line in sc.report_lines(curve))


def test_speeding_up_and_unmeasured_stretches_are_left_out():
  d = drives({64: 50.0})
  d[0] += [stretch(70, 20.0, dv=3.0) for _ in range(200)] + [{'dur': 10.0, 'm': 300.0, 'obd_gal': None}] * 50
  assert [r['mph'] for r in sc.speed_curve(d)['rows']] == [64]
  assert sc.speed_curve([])['rows'] == [] and 'no speed has enough' in sc.report_lines(sc.speed_curve([]))[-1]


def conditioned(mph, mpg, ambient, climate, pitch=0.0):
  return {**stretch(mph, mpg, pitch=pitch), 'ambient_c': ambient, 'climate': climate}


def test_tables_are_kept_per_grade_and_condition_most_specific_first():
  cold = [[conditioned(m, g, 5.0, 'heat') for m, g in ((63, 50.0), (65, 50.0)) for _ in range(120)] for _ in range(4)]
  warm = [[conditioned(m, g, 25.0, 'cool') for m, g in ((63, 60.0), (65, 50.0)) for _ in range(120)] for _ in range(4)]
  hill = [[conditioned(65, 50.0, 5.0, 'heat', pitch=0.04) for _ in range(120)] for _ in range(4)]
  physics = {'climb': el.DEFAULT_CLIMB, 'speed': el.DEFAULT_SPEED, 'pitch_zero': 0.0}   # these roads are not driven both ways
  doc = sc.condition_tables(warm + hill + cold, physics, 'cold', 'heat')
  assert doc['levels'] == ['cold, climate heat', 'cold', 'all driving'] and len(doc['flat']) == 3
  assert sc.current_climate(warm + hill + cold) == 'heat' and sc.current_climate([[stretch(60, 50)]]) is None
  first, _, everything = doc['flat']
  assert abs(first['63']['est'] - 50.0) < 1.0 and everything['63']['est'] > 53.0     # the warm days are only in "all"
  assert '65' in doc['uphill'][0] and '63' not in doc['uphill'][0] and doc['downhill'][0] == {}
  assert first['65']['mi'] < everything['65']['mi']                                   # the hill miles are not on the flat table
  # in the cold nothing is gained at 63, though all driving together says there is
  assert sc.choose(doc['flat'], 65) == 0.0 and sc.choose_u(everything, 65) == 2.0
  assert sc.judged_level(doc['flat'], 65) is first and sc.judged_level([{}, everything], 65) is everything
  assert sc.judged_level([{}], 65) is None and sc.choose([], 65) == 0.0 and sc.thin([], 65) == [1.0, 2.0]
  assert sc.thin(doc['uphill'], 65) == [1.0, 2.0] and sc.thin(doc['downhill'], 65, over_mph=3, under_mph=0) == [-1.0, -2.0, -3.0]
  assert sc.grade_of({'pitch': 0.036}, 0.0164) == 'flat' and sc.grade_of({'pitch': 0.04}, 0.0164) == 'uphill'
  assert sc.grade_of({'pitch': -0.01}, 0.0164) == 'downhill' and sc.grade_of({}) == 'flat'
  assert any('uphill' in line for line in sc.condition_lines(doc))


def test_the_cars_drive_mode_is_a_condition_of_its_own():
  physics = {'climb': el.DEFAULT_CLIMB, 'speed': el.DEFAULT_SPEED, 'pitch_zero': 0.0}
  eco = [[{**conditioned(m, g, 5.0, 'heat'), 'drive_mode': 'eco'} for m, g in ((63, 50.0), (65, 50.0)) for _ in range(120)] for _ in range(4)]
  sport = [[{**conditioned(m, g, 5.0, 'heat'), 'drive_mode': 'sport'} for m, g in ((63, 58.0), (65, 46.0)) for _ in range(120)] for _ in range(4)]
  assert sc.current_mode(eco + sport) == 'sport' and sc.current_mode([[stretch(60, 50)]]) is None
  assert sc.current_mode(sport + [[{**stretch(60, 50), 'drive_mode': 'unknown'}]]) == 'sport'
  doc = sc.condition_tables(eco + sport, physics, 'cold', 'heat', mode='sport')
  assert doc['levels'] == ['sport, cold, climate heat', 'sport, cold', 'sport', 'all driving'] and doc['mode'] == 'sport'
  assert sc.choose(doc['flat'], 65) == 2.0                    # in Sport 63 is clearly cheaper
  in_eco = sc.condition_tables(eco + sport, physics, 'cold', 'heat', mode='eco')
  assert sc.choose(in_eco['flat'], 65) == 0.0                 # in Eco it is not, whatever Sport measured
  assert sc.condition_tables(eco, physics, 'cold', None, mode='eco')['levels'] == ['eco, cold', 'eco', 'all driving']
  assert sc.condition_tables(eco, physics, None, None)['levels'] == ['all driving']
