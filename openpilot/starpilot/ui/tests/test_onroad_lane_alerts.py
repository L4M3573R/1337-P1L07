from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.selfdrived.events import EVENTS, ET, EventName
from openpilot.starpilot.ui.onroad_lane_alerts import lateral_lane_alert
from openpilot.starpilot.ui.onroad_state import AlertSize, OnroadAlert


def sources(state='preLaneChange', direction='left'):
  return {'car': NS(canValid=True, canTimeout=False, leftBlindspot=False, rightBlindspot=False),
          'control': NS(latActive=True), 'selfdrive': NS(enabled=False),
          'model': NS(meta=NS(laneChangeState=state, laneChangeDirection=direction))}


@pytest.mark.parametrize('state,direction,blocked,event', [
  ('preLaneChange', 'left', False, 'preLaneChangeLeft'),
  ('preLaneChange', 'right', False, 'preLaneChangeRight'),
  ('preLaneChange', 'left', True, 'laneChangeBlocked'),
  ('laneChangeStarting', 'right', False, 'laneChange'),
  ('laneChangeFinishing', 'left', False, 'laneChange'),
])
def test_lateral_only_uses_original_warning_text(state, direction, blocked, event):
  data = sources(state, direction)
  data['car'].leftBlindspot = blocked
  alert = lateral_lane_alert(OnroadAlert(), **data)
  native = EVENTS[getattr(EventName, event)][ET.WARNING]
  assert (alert.text1, alert.text2, alert.alert_type) == (native.alert_text_1, native.alert_text_2, event + '/warning')


@pytest.mark.parametrize('missing', ['car', 'control', 'model', 'selfdrive'])
def test_missing_display_source_does_not_invent_lane_notice(missing):
  data = sources()
  data[missing] = None
  assert lateral_lane_alert(OnroadAlert(), **data) == OnroadAlert()


def test_existing_alerts_and_actual_steering_state_take_priority():
  data = sources()
  existing = OnroadAlert(AlertSize.FULL, 'TAKE CONTROL', 'Communication Issue', True)
  assert lateral_lane_alert(existing, **data) is existing
  data['control'].latActive = False
  assert lateral_lane_alert(OnroadAlert(), **data) == OnroadAlert()
  data['control'].latActive = True
  data['selfdrive'].enabled = True
  assert lateral_lane_alert(OnroadAlert(), **data) == OnroadAlert()
  data['selfdrive'].enabled = False
  data['car'].canTimeout = True
  assert lateral_lane_alert(OnroadAlert(), **data) == OnroadAlert()


@pytest.mark.parametrize('state,direction', [('off', 'left'), ('preLaneChange', 'none')])
def test_no_notice_for_blinkers_without_lane_change_intent(state, direction):
  assert lateral_lane_alert(OnroadAlert(), **sources(state, direction)) == OnroadAlert()


def turn_sources(*, left=True, speed=4.0, prob=1.0, lat=True):
  from openpilot.cereal import log
  desire = [0.0] * 8
  desire[log.Desire.turnLeft if left else log.Desire.turnRight] = prob
  return {'car': NS(canValid=True, canTimeout=False, standstill=speed == 0, gearShifter='drive', vEgo=speed,
                    leftBlinker=left, rightBlinker=not left),
          'control': NS(latActive=lat), 'model': NS(meta=NS(desireState=desire))}


@pytest.mark.parametrize('left,text', [(True, 'Turning Left'), (False, 'Turning Right')])
def test_turn_guidance_shows_turning_notice(left, text):
  from openpilot.starpilot.ui.onroad_lane_alerts import turn_intent_alert
  alert = turn_intent_alert(OnroadAlert(), **turn_sources(left=left))
  assert (alert.size, alert.text1) == (AlertSize.SMALL, text)


@pytest.mark.parametrize('change', [{'speed': 12.0}, {'speed': 0}, {'prob': 0.2}, {'lat': False}])
def test_turning_notice_only_while_turn_guidance_is_in_use(change):
  from openpilot.starpilot.ui.onroad_lane_alerts import turn_intent_alert
  assert turn_intent_alert(OnroadAlert(), **turn_sources(**change)) == OnroadAlert()


def test_turning_notice_never_replaces_an_existing_alert():
  from openpilot.starpilot.ui.onroad_lane_alerts import turn_intent_alert
  existing = OnroadAlert(AlertSize.FULL, 'TAKE CONTROL', 'Communication Issue', True)
  assert turn_intent_alert(existing, **turn_sources()) is existing
