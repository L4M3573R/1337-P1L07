"""No-blinker straight guard in shadow: gates, the map condition, and one log per turn with its outcome."""
from dataclasses import replace

import pytest

from openpilot.starpilot.navigation import straight_guard as sg
from openpilot.starpilot.navigation.side_road import Crossing

FOUR_WAY = Crossing(15.0, 88.0, -83.0, 25.0, 1.0)  # bookmark 1 (2026-11-15): street left, minor road right, road straight
POS = (40.0, -75.0, 207.0)


def sample(t=0.0, **kw):
  base = sg.Sample(t=t, speed=7.7, left_blinker=False, right_blinker=False, lat_active=True, steering_pressed=False,
                   steering_angle=-5.0, model_curvature=0.0068, lane_prob=0.01, edge_std=4.7, position=POS)
  return replace(base, **kw)


class Log:
  def __init__(self):
    self.events = []

  def __call__(self, feature, slot, min_gap_s, **fields):
    self.events.append((slot, fields))


def run(samples, crossing=FOUR_WAY, known=True):
  log, lookups = Log(), []

  def at(lat, lon, bearing):
    lookups.append((lat, lon, bearing))
    return known, crossing
  guard = sg.StraightGuardShadow(log=log)
  return [guard.step(s, at) for s in samples], log.events, lookups


def test_turning_side_gates():
  assert sg.turning_side(sample()) == 'right'
  assert sg.turning_side(sample(model_curvature=-0.01)) == 'left'
  for blocked in ({'right_blinker': True}, {'left_blinker': True}, {'lat_active': False}, {'speed': 1.0},
                  {'speed': sg.CUTOFF_MPS + 1}, {'model_curvature': 0.004}, {'model_curvature': float('nan')},
                  {'steering_pressed': True}, {'steering_angle': -31.7}):  # the driver turning, not the model
    assert sg.turning_side(sample(**blocked)) is None, blocked


def test_would_hold_needs_a_close_branch_on_that_side_and_a_straight_road():
  assert sg.would_hold('right', FOUR_WAY) and sg.would_hold('left', FOUR_WAY)
  assert not sg.would_hold('right', replace(FOUR_WAY, right_deg=None))
  assert not sg.would_hold('right', replace(FOUR_WAY, distance_m=sg.BRANCH_AHEAD_M + 1))
  assert not sg.would_hold('right', replace(FOUR_WAY, distance_m=-6.0))
  assert not sg.would_hold('right', replace(FOUR_WAY, bend_deg=-60.0))  # the road itself turns right here
  assert not sg.would_hold('right', replace(FOUR_WAY, continues_m=5.0))  # a T: there is no straight on
  assert not sg.would_hold('right', None)


def test_logs_once_then_the_outcome_and_rearms_after_a_quiet_second():
  steps = [sample(0.0), sample(0.3), sample(0.6, steering_pressed=True, steering_angle=24.0), sample(3.0),
           sample(3.5, model_curvature=0.0), sample(4.6, model_curvature=0.0), sample(4.9)]
  slots, events, lookups = run(steps)
  assert slots == ['would_hold', None, None, 'outcome', None, None, 'would_hold']
  hold = events[0][1]
  assert hold['side'] == 'right' and hold['junction_m'] == 15.0 and hold['branch_deg'] == -83.0 and hold['lane_prob'] == 0.01
  outcome = events[1][1]
  assert outcome['driver_took_wheel'] and outcome['max_steering_deg'] == 24.0 and not outcome['lat_lost']
  assert len(lookups) == 2  # no map search while an episode is open


def test_no_relog_while_the_same_turn_continues_and_lookups_are_rate_limited():
  slots, events, lookups = run([sample(0.0, model_curvature=0.0068)] + [sample(t) for t in (0.1, 0.2)],
                               crossing=replace(FOUR_WAY, distance_m=80.0))
  assert slots == [None, None, None] and events == [] and len(lookups) == 1  # 0.1/0.2 s inside LOOKUP_PERIOD_S


def test_missing_map_logs_no_map_and_never_holds():
  slots, events, _ = run([sample(0.0)], crossing=None, known=False)
  assert slots == ['no_map'] and events[0][0] == 'no_map' and events[0][1]['side'] == 'right'


def test_no_position_means_no_lookup():
  slots, events, lookups = run([sample(0.0, position=None)])
  assert slots == [None] and events == [] and lookups == []


def test_sample_from_reads_the_messages(monkeypatch):
  from types import SimpleNamespace as NS
  import openpilot.starpilot.gps.source as source
  gps = NS(latitude=40.0, longitude=-75.0, bearingDeg=207.0, bearingAccuracyDeg=2.0, speed=7.7)
  monkeypatch.setattr(source, 'select_source', lambda sm, now_ns: ('comma', (now_ns, gps)))
  sm = {'carState': NS(vEgo=7.7, leftBlinker=False, rightBlinker=True, steeringPressed=False, steeringAngleDeg=-3.0),
        'carControl': NS(latActive=True),
        'modelV2': NS(laneLineProbs=[0.9, 0.2, 0.4, 0.1], roadEdgeStds=[1.0, 4.0], action=NS(desiredCurvature=0.01))}
  s = sg.sample_from(sm, 5_000_000_000)
  assert s.t == 5.0 and s.right_blinker and s.lane_prob == 0.2 and s.edge_std == 4.0 and s.model_curvature == 0.01
  assert s.position == pytest.approx((40.0, -75.0, 207.0))


def test_quiet_after_the_driver_steers():
  steps = [sample(0.0, steering_pressed=True, model_curvature=0.0), sample(1.0), sample(1.6)]
  slots, events, _ = run(steps)
  assert slots == [None, None, 'would_hold']  # 1.0 s after a press is still the driver's turn
