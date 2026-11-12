from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.starpilot.lateral.curve_guard import (CurveGuard, LINE_CLEARANCE_M, MAX_REDUCTION_FRACTION, MIN_SPEED_MPS)

X = np.linspace(0.0, 60.0, 33)
OFFSET, HALF = 0.23, 1.83 / 2


def line(y):
  return NS(x=X.tolist(), y=list(y))


def model(curvature, inside_gap, outside_gap=1.0, inside_prob=0.95, outside_prob=0.95, edge_std=2.0):
  """A curve: the planned path, and lane lines placed `gap` metres from the car's body sides (y to the right)."""
  path = 0.5 * curvature * X ** 2
  center = path + OFFSET
  right_side, left_side = center + HALF, center - HALF
  if curvature > 0:   # right curve: inside is the right line
    left, right = left_side - outside_gap, right_side + inside_gap
    probs = [0.1, outside_prob, inside_prob, 0.1]
  else:
    left, right = left_side - inside_gap, right_side + outside_gap
    probs = [0.1, inside_prob, outside_prob, 0.1]
  return NS(position=line(path), laneLines=[line(left - 3.6), line(left), line(right), line(right + 3.6)],
            laneLineProbs=probs, laneLineStds=[0.1] * 4, roadEdges=[line(left - 5), line(right + 5)], roadEdgeStds=[edge_std] * 2)


def settle(guard, m, curvature, v=13.0, n=300, **kw):
  args = {'lat_active': True, 'blinker': False, 'lane_change': False, **kw}
  r = None
  for _ in range(n):
    r = guard.update(m, curvature, v, **args)
  return r


@pytest.mark.parametrize('curvature', [0.012, -0.012])
def test_clear_curve_is_untouched(curvature):
  g = CurveGuard(1.83, OFFSET)
  r = settle(g, model(curvature, inside_gap=0.4), curvature)
  assert r.curvature == curvature and not r.active and r.reason == 'clear'


@pytest.mark.parametrize('curvature', [0.012, -0.012])
def test_cutting_the_inside_takes_out_a_bounded_amount_of_turn(curvature):
  g = CurveGuard(1.83, OFFSET)
  r = settle(g, model(curvature, inside_gap=-0.5), curvature)
  assert r.active and r.reason == 'guarding' and r.inside == ('right' if curvature > 0 else 'left')
  assert np.sign(r.curvature) == np.sign(curvature)                       # never past straight
  assert abs(r.curvature) < abs(curvature)                                  # never adds turn
  assert abs(curvature) - abs(r.curvature) == pytest.approx(MAX_REDUCTION_FRACTION * abs(curvature), rel=1e-3)
  assert r.gap_m == pytest.approx(-0.5, abs=0.01)


def test_eases_in_rather_than_stepping():
  g = CurveGuard(1.83, OFFSET)
  m = model(0.012, inside_gap=-0.5)
  first = g.update(m, 0.012, 13.0, lat_active=True, blinker=False, lane_change=False)
  assert 0.012 - first.curvature < 0.1 * MAX_REDUCTION_FRACTION * 0.012
  later = settle(g, m, 0.012, n=100)
  assert later.curvature < first.curvature


def test_small_intrusion_asks_for_less():
  g1, g2 = CurveGuard(1.83, OFFSET), CurveGuard(1.83, OFFSET)
  slight = settle(g1, model(0.004, inside_gap=LINE_CLEARANCE_M - 0.05), 0.004, v=25.0)
  deep = settle(g2, model(0.004, inside_gap=-0.4), 0.004, v=25.0)
  assert 0.004 > slight.curvature > deep.curvature


@pytest.mark.parametrize('kw', [{'blinker': True}, {'lane_change': True}, {'lat_active': False}, {'v': MIN_SPEED_MPS - 0.5}])
def test_off_for_turn_signal_lane_change_disengaged_and_low_speed(kw):
  g = CurveGuard(1.83, OFFSET)
  v = kw.pop('v', 13.0)
  r = settle(g, model(0.012, inside_gap=-0.5), 0.012, v=v, **kw)
  assert r.curvature == pytest.approx(0.012) and not r.active


def test_needs_a_confident_inside_reference():
  g = CurveGuard(1.83, OFFSET)
  r = settle(g, model(0.012, inside_gap=-0.5, inside_prob=0.4), 0.012)
  assert r.curvature == 0.012 and r.reason == 'no confident inside reference'


def test_never_pushes_into_the_outside_line():
  g = CurveGuard(1.83, OFFSET)
  r = settle(g, model(0.012, inside_gap=-0.5, outside_gap=0.05), 0.012)   # no room on the outside
  assert r.curvature == 0.012 and r.reason == 'no room outside'


def test_straight_road_and_bad_data_are_untouched():
  g = CurveGuard(1.83, OFFSET)
  assert settle(g, model(0.001, inside_gap=-0.5), 0.001).curvature == 0.001
  assert settle(g, None, 0.012).curvature == 0.012
  assert settle(g, NS(position=line(X * 0), laneLines=[], laneLineProbs=[], laneLineStds=[], roadEdges=[], roadEdgeStds=[]),
                0.012).curvature == 0.012
  assert settle(g, model(0.012, inside_gap=-0.5), float('nan')).reason == 'inactive'


def test_disabled_guard_does_nothing():
  g = CurveGuard(1.83, OFFSET, enabled=False)
  assert settle(g, model(0.012, inside_gap=-0.5), 0.012).curvature == 0.012


def test_camera_offset_moves_the_body():
  # The same lines, judged with the lens at the car center, leave the right side 0.23 m further from the inside line.
  m = model(0.012, inside_gap=-0.1)
  assert settle(CurveGuard(1.83, OFFSET), m, 0.012).active
  assert not settle(CurveGuard(1.83, 0.0), m, 0.012).active
