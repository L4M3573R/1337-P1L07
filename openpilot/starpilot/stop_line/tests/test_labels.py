import math

import numpy as np

from openpilot.starpilot.stop_line.labels import (LABEL_RANGE_M, find_stops, label_frames, model_stop_distance, travel)

DT = 0.05


def drive(segments):
  """Speeds from (seconds, speed or (start, end)) pieces at 20 Hz."""
  v = []
  for seconds, speed in segments:
    n = int(round(seconds / DT))
    v.extend(np.linspace(speed[0], speed[1], n) if isinstance(speed, tuple) else [speed] * n)
  v = np.asarray(v, dtype=np.float64)
  t = np.arange(v.size) * DT
  return t, v, travel(t, v)


def stops_of(t, v, s, lead=None, brake=None, model_stop=None, sign=None):
  n = t.size
  lead = np.full(n, np.nan) if lead is None else lead
  brake = np.zeros(n, dtype=bool) if brake is None else brake
  model_stop = np.full(n, np.nan) if model_stop is None else model_stop
  sign = np.zeros(n, dtype=bool) if sign is None else sign
  return find_stops(t, v, s, lead, brake, model_stop, sign)


def test_model_stop_distance_interpolates_the_zero_speed_point():
  x = [0., 10., 20., 30.]
  assert math.isclose(model_stop_distance(x, [10., 5., 0., 0.]), 10. + 10. * (5. - .5) / 5.)
  assert math.isnan(model_stop_distance(x, [10., 10., 10., 10.]))
  assert model_stop_distance(x, [0., 0., 0., 0.]) == 0.


def test_front_stop_labels_the_approach_with_distance_to_rest():
  t, v, s = drive([(10, 15.), (8, (15., 0.)), (5, 0.), (10, (0., 15.)), (20, 15.)])
  stops = stops_of(t, v, s)
  assert len(stops) == 1 and stops[0].front and not stops[0].last
  dist, target, _ = label_frames(t, v, s, stops)
  rest = stops[0].rest_index
  labeled = np.flatnonzero(target == 1)
  assert labeled.size and labeled.max() < rest
  assert np.allclose(dist[labeled], s[rest] - s[labeled])
  assert np.nanmax(dist) <= LABEL_RANGE_M
  # far from any stop at speed is a 'no stop' frame; standing still is ignored
  assert target[0] == 0 and target[rest + 10] == -1


def test_queue_stop_and_parking_are_not_labels():
  t, v, s = drive([(10, 15.), (8, (15., 0.)), (5, 0.), (10, (0., 15.)), (10, 15.), (8, (15., 0.)), (5, 0.)])
  lead = np.full(t.size, np.nan)
  lead[:int(20 / DT)] = 6.0  # a car just ahead at the first stop
  stops = stops_of(t, v, s, lead=lead)
  assert [(st.front, st.last) for st in stops] == [(False, False), (True, True)]
  _, target, _ = label_frames(t, v, s, stops)
  assert not np.any(target == 1)


def test_creep_up_after_a_stop_is_not_an_approach():
  t, v, s = drive([(10, 15.), (8, (15., 0.)), (4, 0.), (2, 1.5), (4, 0.), (10, (0., 15.)), (20, 15.)])
  stops = stops_of(t, v, s)
  assert len(stops) == 1


def test_rolling_stop_is_found_and_its_approach_is_not_a_no_stop_frame():
  from openpilot.starpilot.stop_line.labels import find_rolls
  t, v, s = drive([(10, 15.), (6, (15., 1.5)), (1, 1.5), (6, (1.5, 15.)), (10, 15.)])
  n = t.size
  rolls = find_rolls(t, v, s, np.full(n, np.nan), np.zeros(n, dtype=bool))
  assert len(rolls) == 1 and 1.0 < rolls[0].min_speed < 2.0
  _, target, _ = label_frames(t, v, s, [], rolls)
  k = rolls[0].slow_index
  approach = (s > s[k] - LABEL_RANGE_M) & (np.arange(n) < k) & (v > 5.0)
  assert approach.any() and not np.any(target[approach] == 0)
  # slowing behind a car is not a rolling stop
  assert not find_rolls(t, v, s, np.full(n, 8.0), np.zeros(n, dtype=bool))
