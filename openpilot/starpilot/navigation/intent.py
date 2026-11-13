"""Route advice admitted through existing lateral and longitudinal owners."""

import math

from openpilot.cereal import log
from openpilot.common.constants import CV

from openpilot.starpilot.navigation.wire import navigation_state

SERVICE = 'starpilotNavigation'
MAX_AGE_NS = 2_500_000_000
SIGNAL_STOP_RELEASE_NS = 1_000_000_000  # retired: StarPilot holds a predicted stop until standstill
TARGET_MPH = {'uturn': 5., 'sharpLeft': 10., 'sharpRight': 10., 'left': 14., 'right': 14.}


def current_instruction(sm, now_ns: int):
  try:
    nav, device = navigation_state(sm[SERVICE]), sm['deviceState']
    stamp = sm.logMonoTime[SERVICE]
    device_stamp = sm.logMonoTime['deviceState']
    if (not sm.valid[SERVICE] or not sm.alive[SERVICE] or not nav.enabled or not nav.controlValid or
        nav.status != 'guiding' or not nav.sessionId or not nav.revision or
        not 0 < nav.frameMonoTime <= stamp <= now_ns <= nav.frameMonoTime + MAX_AGE_NS or
        not sm.valid['deviceState'] or not sm.alive['deviceState'] or not device.started or
        not 0 < device.startedMonoTime == nav.startedMonoTime <= device_stamp <= now_ns <= device_stamp + 2_000_000_000 or
        not nav.startedMonoTime <= nav.locationMonoTime <= now_ns <= nav.locationMonoTime + MAX_AGE_NS):
      return None
    for service in ('carState', 'carControl'):
      source = sm.logMonoTime[service]
      if not sm.valid[service] or not sm.alive[service] or not nav.startedMonoTime <= source <= now_ns <= source + 100_000_000:
        return None
    cs = sm['carState']
    if not cs.canValid or cs.canTimeout or str(cs.gearShifter) != 'drive':
      return None
    return nav
  except (KeyError, AttributeError, TypeError):
    return None


def turn_desire(sm, now_ns: int, current_desire, *, supported: bool):
  if not supported or current_desire != log.Desire.none:
    return current_desire
  nav = current_instruction(sm, now_ns)
  if nav is None:
    return current_desire
  cs, cc, instruction = sm['carState'], sm['carControl'], nav.instruction
  if (not cc.latActive or cs.standstill or not math.isfinite(cs.vEgo) or not 0 <= cs.vEgo < 14 or
      instruction.maneuverType != 'turn' or not math.isfinite(instruction.distanceMeters) or
      not 0 <= instruction.distanceMeters <= min(90., max(35., cs.vEgo * 6.))):
    return current_desire
  if instruction.maneuverModifier in ('left', 'sharpLeft') and cs.leftBlinker and not cs.rightBlinker and not cs.leftBlindspot:
    return log.Desire.turnLeft
  if instruction.maneuverModifier in ('right', 'sharpRight') and cs.rightBlinker and not cs.leftBlinker and not cs.rightBlindspot:
    return log.Desire.turnRight
  return current_desire


def turn_speed(maneuver, min_steer_speed: float) -> float | None:
  kind, modifier = maneuver.maneuverType, maneuver.maneuverModifier
  target = TARGET_MPH.get(modifier) if kind == 'turn' else None
  if modifier == 'uturn' or 'uturn' in kind or 'u-turn' in kind:
    target = 5.
  elif 'roundabout' in kind or 'rotary' in kind:
    target = 12.
  if target is None or not math.isfinite(maneuver.distanceMeters) or maneuver.distanceMeters < 0:
    return None
  target = max(target * CV.MPH_TO_MS, min_steer_speed)
  return math.sqrt(target * target + 2 * .45 * max(0., maneuver.distanceMeters - 8.))


def cruise_ceiling(sm, CP, now_ns: int, cruise_mps: float) -> float | None:
  nav = current_instruction(sm, now_ns)
  if nav is None:
    return None
  cs, cc = sm['carState'], sm['carControl']
  if (not CP.openpilotLongitudinalControl or CP.passive or CP.dashcamOnly or not cc.longActive or
      cs.gasPressed or cs.brakePressed or not math.isfinite(cruise_mps) or cruise_mps <= 0):
    return None
  minimum = float(CP.minSteerSpeed)
  if not math.isfinite(minimum):
    return None
  for instruction in (nav.instruction, nav.nextManeuver):
    target = turn_speed(instruction, max(minimum, 0.))
    if target is not None and target + .25 < cruise_mps:
      return target
  return None


def matching_turn_signal(sm, now_ns: int, *, supported: bool) -> bool:
  nav = current_instruction(sm, now_ns) if supported else None
  if nav is None:
    return False
  cs, instruction = sm['carState'], nav.instruction
  matching = ((instruction.maneuverModifier in ('left', 'sharpLeft') and cs.leftBlinker and not cs.rightBlinker) or
              (instruction.maneuverModifier in ('right', 'sharpRight') and cs.rightBlinker and not cs.leftBlinker))
  return bool(instruction.maneuverType == 'turn' and matching and math.isfinite(cs.vEgo) and
              math.isfinite(instruction.distanceMeters) and
              0 <= instruction.distanceMeters <= min(250., max(30., 15. + max(0., cs.vEgo) * 12.)))


class TurnIntent:
  def __init__(self, params=None, cp=None):
    from openpilot.starpilot.lateral.auto_lane_change import steering_capable
    from openpilot.starpilot.lateral.lane_change_preferences import LiveLaneChangePolicy
    self.guidance = LiveLaneChangePolicy(params) if params is not None else None
    self.steering_capable = steering_capable(cp)
    self.signal_stop_hold = False
    self.signal_stop_seen_ns = 0
    self.key = None
    self.stop_hold = False
    # Decision log for remote review: where this frame's turn desire came from, or what held it back.
    self.source = ''
    self.blocked_by = ''
    # Blinker turns timed to the road (map or nudge) instead of turning the moment the blinker goes on.
    from openpilot.starpilot.navigation.blinker_turns import BlinkerTurn, MapTurnFinder
    self.blinker_turn = BlinkerTurn()
    self.map_finder = MapTurnFinder(remote=True)  # the road search runs in sideroadd, never in modeld
    self._mode, self._mode_read_ns = 'shadow', 0
    self.turn_distance = None
    # Shadow mode: what the map/nudge timing would do, logged while the old behavior steers.
    self.shadow_turning, self.shadow_reason, self.shadow_start_m = False, '', None

  def _turn_mode(self, now_ns: int) -> str:
    if now_ns - self._mode_read_ns >= 1_000_000_000 or now_ns < self._mode_read_ns:
      from openpilot.starpilot.navigation.blinker_turns import mode
      self._mode, self._mode_read_ns = mode(), now_ns
    return self._mode

  def _timed_turn(self, sm, now_ns: int, cs) -> bool:
    """Map/nudge timing: True once the turn should start (and while it is in progress)."""
    from openpilot.starpilot.gps.source import select_source
    from openpilot.starpilot.navigation.blinker_turns import nudged_toward
    side = 'left' if cs.leftBlinker else 'right'
    fix = select_source(sm, now_ns)
    heading = lookup = None
    if fix is not None:
      gps = fix[1][1]
      if math.isfinite(gps.bearingDeg) and gps.speed > 1.0:
        heading = float(gps.bearingDeg)
        self.map_finder.request(gps.latitude, gps.longitude, heading, side)
      lookup = self.map_finder.latest(side)
    was_turning = self.blinker_turn.turning
    decision = self.blinker_turn.update(now_ns / 1e9, side, cs.vEgo, heading, lookup,
                                        nudged_toward(side, cs.steeringPressed, cs.steeringTorque))
    self.turn_distance = decision.distance_m
    self.shadow_turning, self.shadow_reason = decision.turning, decision.reason
    if decision.turning and not was_turning:
      self.shadow_start_m = None if decision.distance_m is None else round(decision.distance_m)
    elif not decision.turning:
      self.shadow_start_m = None
    return decision.turning

  def select(self, sm, now_ns: int, current_desire, *, supported: bool, model_stop: bool):
    self.source, self.blocked_by = '', ''
    nav = current_instruction(sm, now_ns) if supported else None
    key = (nav.sessionId, nav.startedMonoTime) if nav is not None else None
    if key != self.key:
      self.key, self.stop_hold = key, False
    if nav is None:
      return self.signal_desire(sm, now_ns, current_desire, supported=supported, model_stop=model_stop)
    cs = sm['carState']
    try:
      stamp, plan = sm.logMonoTime['longitudinalPlan'], sm['longitudinalPlan']
      plan_valid = (sm.valid['longitudinalPlan'] and sm.alive['longitudinalPlan'] and
                    nav.startedMonoTime <= stamp <= now_ns <= stamp + 150_000_000 and
                    nav.startedMonoTime <= plan.modelMonoTime <= stamp)
    except (KeyError, AttributeError, TypeError):
      plan_valid = False
    if cs.standstill or cs.leftBlinker == cs.rightBlinker:
      self.stop_hold = False
    elif model_stop or (plan_valid and plan.shouldStop):
      self.stop_hold = True
    if self.stop_hold or not plan_valid:
      self.blocked_by = 'navigation stop hold' if self.stop_hold else 'plan not current'
      return current_desire
    selected = turn_desire(sm, now_ns, current_desire, supported=supported)
    if selected != log.Desire.none:
      if selected != current_desire:
        self.source = 'navigation'
      return selected
    return self.signal_desire(sm, now_ns, current_desire, supported=supported, model_stop=model_stop)

  def signal_desire(self, sm, now_ns, current_desire, *, supported, model_stop):
    if self.guidance is None:
      return current_desire
    try:
      cs, cc, device, plan = (sm[name] for name in ('carState', 'carControl', 'deviceState', 'longitudinalPlan'))
      signaling = bool(cs.leftBlinker or cs.rightBlinker)
      policy = self.guidance.sample(now_ns, signaling=signaling,
                                    maneuver_active=current_desire != log.Desire.none)
      # StarPilot turn desires: once a stop is predicted short of the turn, hold the turn until the
      # car stops (a stopped car's planner keeps reporting shouldStop) or the blinker turns off.
      stop_predicted = bool(model_stop or plan.shouldStop)
      turn_mode = self._turn_mode(now_ns)
      timed = turn_mode == 'map'
      if not signaling:
        self.blinker_turn.update(now_ns / 1e9, None, cs.vEgo, None, None, False)
        self.shadow_turning, self.shadow_reason, self.shadow_start_m = False, '', None
      elif turn_mode == 'shadow' and cs.leftBlinker != cs.rightBlinker:
        self._timed_turn(sm, now_ns, cs)  # decide and log only, on every blinker use; the old behavior still steers
      if timed:
        stop_predicted = False  # the turn waits for the road itself, so a predicted stop needs no extra hold
      if not signaling or cs.standstill:
        self.signal_stop_hold = False
      elif stop_predicted:
        self.signal_stop_hold = True
        self.signal_stop_seen_ns = now_ns
      # The first gate that fails is logged (blockedBy) when exactly one blinker is on.
      gates = (('unsupported model', not supported), ('car cannot steer', not self.steering_capable),
               ('guidance off', not policy.turn_signal_guidance),
               # Below the cutoff the turn wins over a queued or running lane change (as in sunnypilot); only an
               # existing turn is kept as it is.
               ('other maneuver', current_desire not in (log.Desire.none, log.Desire.laneChangeLeft, log.Desire.laneChangeRight)),
               ('stop predicted', self.signal_stop_hold), ('steering inactive', not cc.latActive),
               ('CAN invalid', not cs.canValid or cs.canTimeout), ('not in drive', str(cs.gearShifter) != 'drive'),
               ('standstill', cs.standstill),
               ('speed outside 0.1-20 mph', not math.isfinite(cs.vEgo) or not 0.1 < cs.vEgo < 20 * CV.MPH_TO_MS),
               ('no single blinker', cs.leftBlinker == cs.rightBlinker), ('not started', not device.started))
      blocked = next((name for name, failed in gates if failed), None)
      if blocked is not None:
        self.blocked_by = blocked if cs.leftBlinker != cs.rightBlinker else ''
        return current_desire
      for service, age in (('carState', 100_000_000), ('carControl', 100_000_000),
                           ('longitudinalPlan', 150_000_000), ('deviceState', 2_000_000_000)):
        stamp = sm.logMonoTime[service]
        if (not sm.valid[service] or not sm.alive[service] or
            not 0 < device.startedMonoTime <= stamp <= now_ns <= stamp + age):
          self.blocked_by = f'stale {service}'
          return current_desire
      if not device.startedMonoTime <= plan.modelMonoTime <= sm.logMonoTime['longitudinalPlan']:
        self.blocked_by = 'stale plan'
        return current_desire
      if (cs.leftBlinker and cs.leftBlindspot) or (cs.rightBlinker and cs.rightBlindspot):
        self.blocked_by = 'blind spot'
        return current_desire
      if timed and not self._timed_turn(sm, now_ns, cs):
        self.blocked_by = self.shadow_reason
        return current_desire
      self.source = 'map' if timed and self.blinker_turn.turning and self.turn_distance is not None else \
                    'nudge' if timed else 'blinker'
      return log.Desire.turnLeft if cs.leftBlinker else log.Desire.turnRight
    except (KeyError, AttributeError, TypeError, ValueError):
      pass
    return current_desire
