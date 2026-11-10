"""Use upstream lane-change notices while lateral control runs independently."""

from typing import Any

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.selfdrive.selfdrived.events import EVENTS, ET, EventName
from openpilot.starpilot.ui.onroad_state import AlertSize, OnroadAlert

TURN_GUIDANCE_MAX_SPEED = 20 * CV.MPH_TO_MS  # blinker turn guidance applies below lane-change speed
TURN_INTENT_MIN_PROB = 0.5


def turn_intent_alert(native: OnroadAlert, *, car: Any, control: Any, model: Any) -> OnroadAlert:
  """Show "Turning Left/Right" while blinker turn guidance is steering, like the original StarPilot notice.

  Shown only when the model itself reports that turn intent, so the notice confirms the turn is in use.
  """
  if (native.size != AlertSize.NONE or car is None or control is None or model is None or
      not getattr(control, 'latActive', False) or not getattr(car, 'canValid', False) or getattr(car, 'canTimeout', True) or
      getattr(car, 'standstill', True) or str(getattr(car, 'gearShifter', '')) != 'drive' or
      car.leftBlinker == car.rightBlinker or not 0.1 < getattr(car, 'vEgo', 0.0) < TURN_GUIDANCE_MAX_SPEED):
    return native
  desire = log.Desire.turnLeft if car.leftBlinker else log.Desire.turnRight
  try:
    probability = float(model.meta.desireState[desire])
  except (AttributeError, IndexError, TypeError, ValueError):
    return native
  if probability < TURN_INTENT_MIN_PROB:
    return native
  return OnroadAlert(AlertSize.SMALL, "Turning Left" if car.leftBlinker else "Turning Right", "", False,
                     ("turningLeft" if car.leftBlinker else "turningRight") + "/warning")


def lateral_lane_alert(native: OnroadAlert, *, car: Any, control: Any, model: Any, selfdrive: Any) -> OnroadAlert:
  """Accept only current display sources; never replace an existing alert."""
  if (native.size != AlertSize.NONE or any(source is None for source in (car, control, model, selfdrive)) or
      not getattr(control, 'latActive', False) or getattr(selfdrive, 'enabled', True) or
      not getattr(car, 'canValid', False) or getattr(car, 'canTimeout', True)):
    return native
  meta = getattr(model, 'meta', None)
  state, direction = str(getattr(meta, 'laneChangeState', 'off')), str(getattr(meta, 'laneChangeDirection', 'none'))
  if direction not in ('left', 'right'):
    return native
  if state == 'preLaneChange':
    blocked = car.leftBlindspot if direction == 'left' else car.rightBlindspot
    event = EventName.laneChangeBlocked if blocked else EventName.preLaneChangeLeft if direction == 'left' else EventName.preLaneChangeRight
  elif state in ('laneChangeStarting', 'laneChangeFinishing'):
    event = EventName.laneChange
  else:
    return native
  warning = EVENTS[event][ET.WARNING]
  name = next(name for name in ('preLaneChangeLeft', 'preLaneChangeRight', 'laneChangeBlocked', 'laneChange')
              if getattr(EventName, name) == event)
  return OnroadAlert((AlertSize.NONE, AlertSize.SMALL, AlertSize.MID, AlertSize.FULL)[warning.alert_size],
                     warning.alert_text_1, warning.alert_text_2, False, name + '/warning')
