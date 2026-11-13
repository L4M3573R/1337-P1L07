from openpilot.starpilot.navigation.wire import navigation_envelope
from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import messaging, log
from openpilot.common.constants import CV
from openpilot.starpilot.navigation.intent import current_instruction, turn_desire, cruise_ceiling, turn_speed


@pytest.fixture(autouse=True)
def speed_mode_turns(monkeypatch):
  """These tests cover the blinker gates with the original turn-at-once timing ("speed" mode)."""
  from openpilot.starpilot.navigation import blinker_turns
  monkeypatch.setattr(blinker_turns, 'mode', lambda path=None: 'speed')



NOW = 20_000_000_000


@pytest.fixture
def sm():
  class SM(dict):
    valid = dict.fromkeys(('starpilotNavigation', 'deviceState', 'carState', 'carControl'), True)
    alive = dict(valid)
    logMonoTime = dict.fromkeys(valid, NOW)
  instruction = NS(maneuverType='turn', maneuverModifier='right', distanceMeters=30.)
  nav = NS(enabled=True, controlValid=True, status='guiding', sessionId='producer', revision='one', frameMonoTime=NOW,
           startedMonoTime=10_000_000_000, locationMonoTime=NOW, instruction=instruction, nextManeuver=instruction)
  nav.version = 2
  return SM(starpilotNavigation=NS(navigation=nav), deviceState=NS(started=True, startedMonoTime=10_000_000_000),
            carState=NS(canValid=True, canTimeout=False, gearShifter='drive', vEgo=5., standstill=False, leftBlinker=False,
                        rightBlinker=True, leftBlindspot=False, rightBlindspot=False, gasPressed=False, brakePressed=False),
            carControl=NS(latActive=True, longActive=True))


def test_signal_turn_and_existing_desire_preserved(sm):
  assert turn_desire(sm, NOW, log.Desire.none, supported=True) == log.Desire.turnRight
  assert turn_desire(sm, NOW, log.Desire.laneChangeLeft, supported=True) == log.Desire.laneChangeLeft
  assert turn_desire(sm, NOW, log.Desire.none, supported=False) == log.Desire.none
  sm['carState'].rightBlinker = False
  assert turn_desire(sm, NOW, log.Desire.none, supported=True) == log.Desire.none


@pytest.mark.parametrize('field,value', [('standstill', True), ('rightBlindspot', True), ('leftBlinker', True),
                                        ('vEgo', 14.), ('gearShifter', 'reverse'), ('canValid', False), ('canTimeout', True)])
def test_no_turn_without_vehicle_admission(sm, field, value):
  setattr(sm['carState'], field, value)
  assert turn_desire(sm, NOW, log.Desire.none, supported=True) == log.Desire.none


@pytest.mark.parametrize('field,value', [('controlValid', False), ('enabled', False), ('frameMonoTime', NOW-3_000_000_000),
                                        ('locationMonoTime', NOW+1), ('startedMonoTime', 1), ('sessionId', '')])
def test_stale_previous_drive_and_disabled_nav_are_inert(sm, field, value):
  setattr(sm['starpilotNavigation'].navigation, field, value)
  assert current_instruction(sm, NOW) is None


def test_original_turn_ceiling_and_override(sm):
  cp = NS(openpilotLongitudinalControl=True, passive=False, dashcamOnly=False, minSteerSpeed=0.)
  expected = ((14 * CV.MPH_TO_MS) ** 2 + 2 * .45 * (30 - 8)) ** .5
  assert cruise_ceiling(sm, cp, NOW, 20.) == pytest.approx(expected)
  sm['carState'].gasPressed = True
  assert cruise_ceiling(sm, cp, NOW, 20.) is None
  sm['carState'].gasPressed = False
  cp.openpilotLongitudinalControl = False
  assert cruise_ceiling(sm, cp, NOW, 20.) is None


@pytest.mark.parametrize('kind,modifier,mph', [('turn', 'left', 14), ('turn', 'sharpRight', 10), ('turn', 'uturn', 5),
                                             ('roundabout', 'right', 12), ('fork', 'right', None)])
def test_turn_target_matches_dom_table(kind, modifier, mph):
  result = turn_speed(NS(maneuverType=kind, maneuverModifier=modifier, distanceMeters=8.), 0.)
  assert result == pytest.approx(mph * CV.MPH_TO_MS) if mph is not None else result is None


def test_new_event_roundtrip_keeps_route_and_control_evidence():
  message = messaging.new_message('starpilotNavigation')
  message.starpilotNavigation = navigation_envelope({'sessionId': 'owner', 'frameMonoTime': NOW, 'startedMonoTime': 10_000_000_000,
                                    'enabled': True, 'revision': 'one', 'controlValid': True, 'status': 'guiding',
                                    'instruction': {'text': 'Turn right', 'maneuverType': 'turn', 'maneuverModifier': 'right', 'distanceMeters': 30.},
                                    'route': [{'latitude': 1., 'longitude': 2.}], 'locationMonoTime': NOW})
  with log.Event.from_bytes(message.to_bytes()) as received:
    assert received.which() == 'starpilotNavigation'
    assert received.starpilotNavigation.navigation.route[0].latitude == 1.
    assert received.starpilotNavigation.navigation.controlValid


def test_native_planner_navigation_cap_and_default_equivalence():
  import numpy as np
  from unittest.mock import patch
  from opendbc.car.hyundai.interface import CarInterface
  from opendbc.car.hyundai.values import CAR
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
  from openpilot.starpilot.longitudinal.tests.test_conditional_handoff import Frame, NOW as FRAME_NOW, DRIVE

  cp = CarInterface.get_non_essential_params(CAR.HYUNDAI_IONIQ_6)
  cp.openpilotLongitudinalControl = True
  baseline, absent, guided = (LongitudinalPlanner(cp, init_v=20.) for _ in range(3))
  changed = False
  for tick in range(40):
    frame = Frame()
    now = FRAME_NOW + tick * 50_000_000
    frame['carState'].vEgo = 20.
    frame['carState'].gearShifter = 'drive'
    frame.logMonoTime = dict.fromkeys(frame, now)
    with patch('openpilot.selfdrive.controls.lib.longitudinal_planner.navigation_ceiling', return_value=None):
      baseline.update(frame, now_ns=now)
    absent.update(frame, now_ns=now)
    assert baseline.output_a_target == absent.output_a_target
    np.testing.assert_array_equal(baseline.mpc.params, absent.mpc.params)
    nav = messaging.new_message('starpilotNavigation')
    nav.starpilotNavigation = navigation_envelope({'enabled': True, 'controlValid': True, 'status': 'guiding', 'sessionId': 'test',
                              'revision': 'one', 'frameMonoTime': now, 'startedMonoTime': DRIVE, 'locationMonoTime': now,
                              'instruction': {'maneuverType': 'turn', 'maneuverModifier': 'right', 'distanceMeters': 20.}})
    frame['starpilotNavigation'] = nav.starpilotNavigation
    frame.logMonoTime['starpilotNavigation'] = now
    frame.valid['starpilotNavigation'] = frame.alive['starpilotNavigation'] = True
    guided.update(frame, now_ns=now)
    assert guided.output_a_target <= baseline.output_a_target
    changed |= guided.output_a_target < baseline.output_a_target
  assert changed


def test_turn_stop_hold_reuses_planner_stop_until_standstill(sm):
  from openpilot.starpilot.navigation.intent import TurnIntent
  owner = TurnIntent()
  sm['longitudinalPlan'] = NS(modelMonoTime=NOW, shouldStop=False)
  sm.logMonoTime['longitudinalPlan'] = NOW
  sm.valid['longitudinalPlan'] = sm.alive['longitudinalPlan'] = True
  def sample(model_stop=False):
    return owner.select(sm, NOW, log.Desire.none, supported=True, model_stop=model_stop)
  assert sample() == log.Desire.turnRight
  sm['longitudinalPlan'].shouldStop = True
  assert sample() == log.Desire.none
  sm['longitudinalPlan'].shouldStop = False
  sm['starpilotNavigation'].navigation.revision = 'favorite-added'
  assert sample() == log.Desire.none
  sm['carState'].standstill = True
  assert sample() == log.Desire.none
  sm['carState'].standstill = False
  assert sample() == log.Desire.turnRight
  assert sample(model_stop=True) == log.Desire.none
  sm.valid['longitudinalPlan'] = False
  assert sample() == log.Desire.none


def test_signal_turn_guidance_resumes_after_stopping_at_the_turn(sm):
  """Blinker-only turn: slow for the stop, wait stopped (planner says stop), then pull away into the turn."""
  from openpilot.starpilot.navigation.intent import TurnIntent
  owner = TurnIntent()
  owner.guidance = NS(sample=lambda *_, **__: NS(turn_signal_guidance=True))
  owner.steering_capable = True
  sm['starpilotNavigation'].navigation.enabled = False  # no route: blinker-only guidance
  sm['carControl'].longActive = False  # Always On Lateral without cruise
  sm['longitudinalPlan'] = NS(modelMonoTime=NOW, shouldStop=False)
  sm.logMonoTime['longitudinalPlan'] = NOW
  sm.valid['longitudinalPlan'] = sm.alive['longitudinalPlan'] = True
  cs, plan = sm['carState'], sm['longitudinalPlan']

  def sample():
    return owner.select(sm, NOW, log.Desire.none, supported=True, model_stop=False)

  assert sample() == log.Desire.turnRight
  plan.shouldStop = True  # slowing for the stop line: no turn guidance yet
  assert sample() == log.Desire.none
  cs.standstill, cs.vEgo = True, 0.  # waiting to turn; planner still says stop
  assert sample() == log.Desire.none
  cs.standstill, cs.vEgo, plan.shouldStop = False, 1.5, False  # pulling away into the turn
  assert sample() == log.Desire.turnRight
  plan.shouldStop = True  # a new stop while moving holds again
  assert sample() == log.Desire.none


def test_signal_turn_holds_a_predicted_stop_until_standstill_like_starpilot(sm):
  """StarPilot turn desires: a stop predicted short of the turn holds guidance until the car stops or the blinker ends."""
  from openpilot.starpilot.navigation.intent import TurnIntent
  owner = TurnIntent()
  owner.guidance = NS(sample=lambda *_, **__: NS(turn_signal_guidance=True))
  owner.steering_capable = True
  sm['starpilotNavigation'].navigation.enabled = False
  plan = NS(modelMonoTime=NOW, shouldStop=False)
  sm['longitudinalPlan'] = plan
  sm.valid['longitudinalPlan'] = sm.alive['longitudinalPlan'] = True
  cs = sm['carState']

  def sample(now, model_stop=False):
    for name in ('carState', 'carControl', 'longitudinalPlan', 'deviceState'):
      sm.logMonoTime[name] = now
    plan.modelMonoTime = now
    return owner.select(sm, now, log.Desire.none, supported=True, model_stop=model_stop)

  assert sample(NOW) == log.Desire.turnRight
  assert sample(NOW + 50_000_000, model_stop=True) == log.Desire.none  # stop predicted short of the turn
  assert sample(NOW + 3_000_000_000) == log.Desire.none  # still rolling: held (no timed release)
  cs.standstill, cs.vEgo = True, 0.
  assert sample(NOW + 3_100_000_000) == log.Desire.none  # stopped: hold released, but no turn at standstill
  cs.standstill, cs.vEgo = False, 2.
  assert sample(NOW + 3_200_000_000) == log.Desire.turnRight  # pulling away into the turn
  assert sample(NOW + 3_250_000_000, model_stop=True) == log.Desire.none
  cs.rightBlinker = False
  sample(NOW + 3_300_000_000)
  cs.rightBlinker = True
  assert sample(NOW + 3_350_000_000) == log.Desire.turnRight  # a fresh signal starts clear


def test_approaching_turn_does_not_start_or_replace_lane_change(sm):
  from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
  from openpilot.starpilot.navigation.intent import matching_turn_signal
  from openpilot.starpilot.lateral.lane_change_preferences import LaneChangePolicy
  cs = sm['carState']
  cs.steeringPressed, cs.steeringTorque = True, -1.
  cs.vEgo = 21. * CV.MPH_TO_MS  # Above the current turn-guidance lane-change floor.
  helper = DesireHelper(LaneChangePolicy(minimum_speed_mps=0.))
  turn = matching_turn_signal(sm, NOW, supported=True)
  assert turn
  helper.update(cs, True, 1., navigation_turn=turn)
  assert helper.lane_change_state == log.LaneChangeState.off
  helper.lane_change_state = log.LaneChangeState.preLaneChange
  helper.update(cs, True, 1., navigation_turn=turn)
  assert helper.lane_change_state == log.LaneChangeState.off
  helper.lane_change_state = log.LaneChangeState.laneChangeStarting
  helper.lane_change_direction = log.LaneChangeDirection.right
  helper.update(cs, True, 1., navigation_turn=turn)
  assert helper.lane_change_state == log.LaneChangeState.laneChangeStarting
  assert helper.desire == log.Desire.laneChangeRight


def test_turn_decision_log_names_the_source_or_the_gate_that_held_it_back(sm):
  from openpilot.starpilot.navigation.intent import TurnIntent
  owner = TurnIntent()
  owner.guidance = NS(sample=lambda *_, **__: NS(turn_signal_guidance=True))
  owner.steering_capable = True
  sm['starpilotNavigation'].navigation.enabled = False
  sm['longitudinalPlan'] = NS(modelMonoTime=NOW, shouldStop=False)
  sm.logMonoTime['longitudinalPlan'] = NOW
  sm.valid['longitudinalPlan'] = sm.alive['longitudinalPlan'] = True
  cs = sm['carState']

  def sample():
    return owner.select(sm, NOW, log.Desire.none, supported=True, model_stop=False)

  assert sample() == log.Desire.turnRight and (owner.source, owner.blocked_by) == ('blinker', '')
  cs.vEgo = 25 * CV.MPH_TO_MS
  assert sample() == log.Desire.none and owner.blocked_by == 'speed outside 0.1-20 mph' and owner.source == ''
  cs.vEgo = 5.
  sm['longitudinalPlan'].shouldStop = True
  assert sample() == log.Desire.none and owner.blocked_by == 'stop predicted'
  sm['longitudinalPlan'].shouldStop = False
  cs.rightBlinker = False
  owner.signal_stop_hold = False
  assert sample() == log.Desire.none and owner.blocked_by == ''  # no blinker: nothing to explain
