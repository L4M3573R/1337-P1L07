"""Blinker turns timed to the road: wait for the mapped road or a nudge, then hold the turn until it is done."""
import pytest

from openpilot.starpilot.navigation import blinker_turns as bt
from openpilot.starpilot.navigation.road_tiles import encode_road_tile
from openpilot.starpilot.navigation.side_road import SideRoad
from openpilot.starpilot.navigation.tests.test_side_road import ROADS, car_at, tile_with

V = 7.0  # m/s, about 16 mph


@pytest.fixture
def turn_case(tmp_path):
  """A left blinker at 11 mph with steering active (the same setup the blinker-gate tests use)."""
  from dataclasses import replace
  from types import SimpleNamespace as NS
  from opendbc.car import structs
  from openpilot.common.params import Params
  from openpilot.starpilot.lateral.lane_change_preferences import KEY, LaneChangePolicy, to_value
  from openpilot.starpilot.navigation.tests.test_signal_guidance import NOW
  params = Params(str(tmp_path))
  cp = structs.CarParams.new_message(carFingerprint='real', brand='hyundai', steerControlType='torque')
  cp.safetyConfigs = [structs.CarParams.SafetyConfig.new_message(safetyModel='hyundai')]
  params.put(KEY, to_value(replace(LaneChangePolicy(), turn_signal_guidance=True)), block=True)
  class SM(dict):
    valid = dict.fromkeys(('carState', 'carControl', 'deviceState', 'longitudinalPlan'), True)
    alive = dict(valid)
    logMonoTime = dict.fromkeys(valid, NOW)
  sm = SM(carState=NS(canValid=True, canTimeout=False, gearShifter='drive', vEgo=5., standstill=False,
                     leftBlinker=True, rightBlinker=False, leftBlindspot=False, rightBlindspot=False),
          carControl=NS(latActive=True), deviceState=NS(started=True, startedMonoTime=1),
          longitudinalPlan=NS(shouldStop=False, modelMonoTime=NOW))
  return params, cp, sm


def lookup(t, distance, known=True):
  return bt.Lookup(t, 'left', SideRoad(distance, 90.0, 'street') if distance is not None else None, known)


def test_waits_for_the_mapped_road_then_turns_at_it():
  turn = bt.BlinkerTurn()
  found = lookup(0.0, 60.0)
  d = turn.update(0.0, 'left', V, 0.0, found, nudged=False)
  assert not d.turning and d.reason == 'map: waiting for the road' and d.distance_m == pytest.approx(60.0)
  t = 0.0
  while not d.turning and t < 20:
    t += 0.05
    d = turn.update(t, 'left', V, 0.0, found, nudged=False)  # same lookup: moved forward by the distance driven
  assert d.turning and d.reason == 'map: at the road'
  assert 60.0 - V * t == pytest.approx(bt.trigger_distance(V), abs=0.5)


def test_no_mapped_road_waits_for_a_nudge_and_never_turns_on_its_own():
  turn = bt.BlinkerTurn()
  for i in range(100):
    d = turn.update(i * 0.05, 'left', V, 0.0, lookup(i * 0.05, None), nudged=False)
    assert not d.turning
  assert d.reason == 'map: no road there, waiting for a nudge'
  assert turn.update(5.1, 'left', V, 0.0, None, nudged=True).turning
  assert bt.BlinkerTurn().update(0.0, 'left', V, 0.0, None, nudged=False).reason == 'no map here, waiting for a nudge'


def test_once_turning_it_holds_until_the_car_has_turned():
  turn = bt.BlinkerTurn()
  assert turn.update(0.0, 'left', V, 10.0, None, nudged=True).turning
  for i, heading in enumerate((5.0, -20.0, -50.0)):  # turning left: heading falls; no lookup, no nudge any more
    assert turn.update(0.1 * (i + 1), 'left', 4.0, heading, None, nudged=False).turning
  d = turn.update(0.5, 'left', 4.0, -62.0, None, nudged=False)  # 72 degrees turned
  assert not d.turning and d.reason == 'turn done'


def test_blinker_off_speeding_up_or_switching_sides_ends_it():
  turn = bt.BlinkerTurn()
  turn.update(0.0, 'left', V, 0.0, None, nudged=True)
  assert not turn.update(0.1, None, V, 0.0, None, nudged=False).turning
  turn.update(0.2, 'left', V, 0.0, None, nudged=True)
  assert not turn.update(0.3, 'left', bt.CUTOFF_MPS + bt.ABORT_MARGIN_MPS + 1, 0.0, None, nudged=False).turning
  turn.update(0.4, 'left', V, 0.0, None, nudged=True)
  assert not turn.update(0.5, 'right', V, 0.0, None, nudged=False).turning
  assert bt.BlinkerTurn().update(0.0, 'left', bt.CUTOFF_MPS + 0.1, 0.0, lookup(0.0, 5.0), nudged=False).reason == 'above turn speed'


@pytest.mark.parametrize('side, pressed, torque, expected', [
  ('left', True, 80.0, True), ('left', True, -80.0, False), ('right', True, -80.0, True), ('left', False, 80.0, False),
  (None, True, 80.0, False)])
def test_nudge_direction(side, pressed, torque, expected):
  assert bt.nudged_toward(side, pressed, torque) is expected


class Store:
  def __init__(self, tiles):
    self.raw = {t.key: encode_road_tile({c: [pts[o[i]:o[i + 1]] for i in range(len(o) - 1)] for c, (pts, o) in t.lines.items()})
                for t in tiles}

  def read(self, key):
    return self.raw.get(key)


def test_finder_reads_stored_tiles_and_says_when_the_map_knows_the_road():
  finder = bt.MapTurnFinder(store=Store([tile_with(ROADS)]))
  found = finder.lookup_now(1.0, *car_at(), 0.0, 'left')
  assert found.map_known and found.road is not None and found.road.distance_m == pytest.approx(80, abs=2)
  off_road = finder.lookup_now(1.0, *car_at(east=80), 0.0, 'left')
  assert not off_road.map_known and off_road.road is None
  assert not bt.MapTurnFinder(store=Store([])).lookup_now(1.0, *car_at(), 0.0, 'left').map_known


def test_settings_accept_speed_and_reject_unknown(tmp_path):
  path = tmp_path / 'blinker_turns.json'
  path.write_text('{"mode": "speed"}')
  assert bt.mode(path) == 'speed'
  path.write_text('{"mode": "sideways"}')
  assert bt.mode(path) == 'shadow'




def test_settings_default_is_shadow(tmp_path):
  assert bt.mode(tmp_path / 'missing.json') == 'shadow'
  (tmp_path / 'b.json').write_text('{"mode": "map"}')
  assert bt.mode(tmp_path / 'b.json') == 'map'


def test_shadow_mode_keeps_the_old_turn_and_logs_the_timed_decision(monkeypatch, turn_case):
  from openpilot.cereal import log
  from openpilot.starpilot.diagnostics.features import turn_decision
  from openpilot.starpilot.navigation.intent import TurnIntent
  from openpilot.starpilot.navigation.tests.test_signal_guidance import select
  params, cp, sm = turn_case
  monkeypatch.setattr(bt, 'mode', lambda path=None: 'shadow')
  sm['carState'].steeringPressed, sm['carState'].steeringTorque = False, 0.0
  owner = TurnIntent(params, cp)
  assert select(owner, sm) == log.Desire.turnLeft  # the old behavior still steers
  logged = turn_decision(owner, log.Desire.turnLeft)
  assert logged['timed'] == 'no map here, waiting for a nudge' and not logged['timed_turning']
  sm['carState'].steeringPressed, sm['carState'].steeringTorque = True, 90.0
  select(owner, sm)
  assert turn_decision(owner, log.Desire.turnLeft)['timed_turning']


def test_map_mode_waits_instead_of_turning_at_once(monkeypatch, turn_case):
  from openpilot.cereal import log
  from openpilot.starpilot.navigation.intent import TurnIntent
  from openpilot.starpilot.navigation.tests.test_signal_guidance import select
  params, cp, sm = turn_case
  monkeypatch.setattr(bt, 'mode', lambda path=None: 'map')
  sm['carState'].steeringPressed, sm['carState'].steeringTorque = False, 0.0
  owner = TurnIntent(params, cp)
  assert select(owner, sm) == log.Desire.none and owner.blocked_by == 'no map here, waiting for a nudge'
  sm['carState'].steeringPressed, sm['carState'].steeringTorque = True, 90.0
  assert select(owner, sm) == log.Desire.turnLeft and owner.source == 'nudge'
  sm['carState'].steeringPressed = False
  assert select(owner, sm) == log.Desire.turnLeft  # held after the nudge
