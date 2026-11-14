"""Turn speed for signalled turns, learned from the driver's own turns by how sharp the turn is.

The blinker-turn logic in modeld (intent.py) knows the side, whether a turn is under way and, from the map, the distance
to the side road and its angle; it publishes that to shared memory (write_status). plannerd (TurnSpeedHost) reads it,
looks up the learned turn speed for that side and sharpness, and caps the cruise target so the car eases (about
TURN_DECEL, like lifting off) to that speed where the turn begins, holds it through the turn, and lets go once it is
done. The cap only ever lowers the target: a car ahead, Force Stop and the pedals still win in the planner.

Learning (TurnLearner): every signalled turn below LEARN_MAX_MPS that turns at least TURN_DONE_DEG (integrated from the
car's yaw rate) gives a sample, the slowest speed through the turn. A turn the driver drove (their speed) teaches it
directly; in a turn the comma drove, braking teaches the speed braked to and the gas the speed pressed to, and no pedal
confirms the current speed. Sharpness: the side road's angle from the map (shallow under 70 degrees, square 70-110,
sharp over 110) and the side (rights are tighter than lefts); unknown when there was no map road.

Steering: the turn speed is never more than the comma's steering can hold through that turn. Each turn also teaches how
tight that kind of turn really is (peak curvature from the yaw rate); the speed is capped where the sideways
acceleration v^2 * curvature reaches the comma's measured steering limit with its margin (map_target.torque_cap): about
1.2 m/s2 at turn speeds, so a 10 m radius right turn is capped near 7.5 mph even if the driver takes it at 11.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

MPH = 0.44704
STATUS_PATH = Path('/dev/shm/starpilot_turn_status.json')
STATUS_MAX_AGE_S = 0.5
TURN_DECEL = 1.0              # m/s^2: how gently the car slows for the turn
APPROACH_MAX_M = 150.0        # a mapped road further than this does not slow the car yet
LEARN_MAX_MPS = 22 * MPH      # turns taken faster than this are not learned (a curve, not a turn)
TURN_DONE_DEG = 60.0
EMA_NEW = 0.3                 # weight of a new sample
CONFIRM_WEIGHT = 0.1          # a comma-driven turn with no pedal nudges the speed toward what was driven
TURN_SPEED_RANGE = (5 * MPH, 22 * MPH)
# starting speeds (mph) until the driver's own turns are learned: (side, sharpness)
DEFAULT_MPH = {('right', 'sharp'): 9.0, ('right', 'square'): 11.0, ('right', 'shallow'): 16.0, ('right', 'unknown'): 11.0,
               ('left', 'sharp'): 11.0, ('left', 'square'): 14.0, ('left', 'shallow'): 18.0, ('left', 'unknown'): 13.0}


# starting turn radius (m) until real turns are measured
DEFAULT_RADIUS_M = {('right', 'sharp'): 7.0, ('right', 'square'): 9.0, ('right', 'shallow'): 20.0, ('right', 'unknown'): 10.0,
                    ('left', 'sharp'): 9.0, ('left', 'square'): 14.0, ('left', 'shallow'): 25.0, ('left', 'unknown'): 13.0}
CURVATURE_MIN_V = 2.0          # m/s: slower, yaw / speed is too noisy to measure the turn's tightness


def steer_limit_speed(curvature: float) -> float:
  """The fastest speed (m/s) the comma's steering can hold through a turn of this curvature (with its margin)."""
  from openpilot.starpilot.curve_speed.map_target import torque_cap
  if not math.isfinite(curvature) or curvature <= 0:
    return TURN_SPEED_RANGE[1]
  best = TURN_SPEED_RANGE[0]
  v = TURN_SPEED_RANGE[0]
  while v <= TURN_SPEED_RANGE[1]:
    if v * v * curvature <= torque_cap(v):
      best = v
    v += 0.1
  return best


def sharpness(angle_deg: float | None) -> str:
  if angle_deg is None or not math.isfinite(angle_deg):
    return 'unknown'
  a = abs(angle_deg)
  return 'shallow' if a < 70.0 else 'square' if a <= 110.0 else 'sharp'


def write_status(doc: dict, path: Path = STATUS_PATH) -> None:
  try:
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(doc, separators=(',', ':')))
    tmp.replace(path)
  except OSError:
    pass


def read_status(now: float, path: Path = STATUS_PATH) -> dict | None:
  """modeld's turn state if fresh (monotonic clock): {'side', 'turning', 'distance_m', 'angle_deg'}, else None."""
  try:
    doc = json.loads(path.read_text())
    if (isinstance(doc, dict) and isinstance(doc.get('t'), (int, float)) and 0 <= now - doc['t'] <= STATUS_MAX_AGE_S and
        doc.get('side') in ('left', 'right')):
      return doc
  except (OSError, ValueError, TypeError):
    pass
  return None


def trigger_distance(v: float) -> float:
  from openpilot.starpilot.navigation.blinker_turns import trigger_distance as td
  return td(v)


@dataclass
class TurnSpeeds:
  """Learned turn speed (m/s) per (side, sharpness): {key: [speed, samples]}."""
  table: dict[str, list] = field(default_factory=dict)
  curvatures: dict[str, list] = field(default_factory=dict)   # {key: [peak curvature 1/m, samples]}

  def curvature(self, side: str, sharp: str) -> float:
    entry = self.curvatures.get(self.key(side, sharp))
    if entry and entry[1] > 0:
      return float(entry[0])
    if sharp != 'unknown':
      fallback = self.curvatures.get(self.key(side, 'unknown'))
      if fallback and fallback[1] > 0:
        return float(fallback[0]) * DEFAULT_RADIUS_M[(side, 'unknown')] / DEFAULT_RADIUS_M[(side, sharp)]
    return 1.0 / DEFAULT_RADIUS_M[(side, sharp)]

  def learn_curvature(self, side: str, sharp: str, curvature: float) -> None:
    if not math.isfinite(curvature) or not 0.005 < curvature < 0.5:
      return
    entry = self.curvatures.get(self.key(side, sharp))
    if not entry or entry[1] == 0:
      self.curvatures[self.key(side, sharp)] = [round(curvature, 4), 1]
    else:
      entry[0] = round((1 - EMA_NEW) * entry[0] + EMA_NEW * curvature, 4)
      entry[1] += 1

  def driver_speed(self, side: str, sharp: str) -> float:
    """The learned speed the driver takes this kind of turn at (before the steering limit)."""
    return self._learned_speed(side, sharp)

  def speed(self, side: str, sharp: str) -> float:
    """The turn speed: the driver's, but never more than the comma's steering can hold through this turn."""
    return min(self._learned_speed(side, sharp), steer_limit_speed(self.curvature(side, sharp)))

  @staticmethod
  def key(side: str, sharp: str) -> str:
    return f'{side}|{sharp}'

  def _learned_speed(self, side: str, sharp: str) -> float:
    entry = self.table.get(self.key(side, sharp))
    if entry and entry[1] > 0:
      return float(entry[0])
    if sharp != 'unknown':   # learned for this side without a map angle beats a default
      fallback = self.table.get(self.key(side, 'unknown'))
      if fallback and fallback[1] > 0:
        scale = DEFAULT_MPH[(side, sharp)] / DEFAULT_MPH[(side, 'unknown')]
        return float(min(max(fallback[0] * scale, TURN_SPEED_RANGE[0]), TURN_SPEED_RANGE[1]))
    return DEFAULT_MPH[(side, sharp)] * MPH

  def learn(self, side: str, sharp: str, sample: float, weight: float = EMA_NEW) -> None:
    sample = min(max(sample, TURN_SPEED_RANGE[0]), TURN_SPEED_RANGE[1])
    entry = self.table.get(self.key(side, sharp))
    if not entry or entry[1] == 0:
      self.table[self.key(side, sharp)] = [round(sample, 3), 1]
    else:
      entry[0] = round((1 - weight) * entry[0] + weight * sample, 3)
      entry[1] += 1

  @classmethod
  def load(cls, path: Path) -> TurnSpeeds:
    try:
      doc = json.loads(path.read_text())
      def entries(name):
        return {k: [float(v[0]), int(v[1])] for k, v in doc.get(name, {}).items()
                if isinstance(k, str) and k.count('|') == 1 and isinstance(v, list) and len(v) == 2}
      return cls(entries('speeds'), entries('curvatures'))
    except (OSError, ValueError, TypeError, AttributeError, IndexError):
      return cls()

  def save(self, path: Path) -> None:
    try:
      path.parent.mkdir(parents=True, exist_ok=True)
      tmp = path.with_suffix('.tmp')
      tmp.write_text(json.dumps({'version': 1, 'speeds': self.table, 'curvatures': self.curvatures}, indent=1))
      tmp.replace(path)
    except OSError:
      pass


def ceiling(status: dict | None, speeds: TurnSpeeds) -> tuple[float | None, float | None]:
  """(cruise ceiling m/s or None, the turn speed used) for modeld's turn state."""
  if status is None:
    return None, None
  side, sharp = status['side'], sharpness(status.get('angle_deg'))
  v_turn = speeds.speed(side, sharp)
  if status.get('turning'):
    return v_turn, v_turn
  d = status.get('distance_m')
  if not isinstance(d, (int, float)) or not math.isfinite(d) or d > APPROACH_MAX_M:
    return None, v_turn
  # reach the turn speed where the turn begins, slowing at TURN_DECEL before it
  return math.sqrt(v_turn * v_turn + 2.0 * TURN_DECEL * max(0.0, d - trigger_distance(v_turn))), v_turn


@dataclass
class TurnLearner:
  """Watches signalled turns and teaches TurnSpeeds the slowest speed through each one."""
  speeds: TurnSpeeds
  side: str | None = None
  sharp: str = 'unknown'
  heading_deg: float = 0.0
  min_v: float = math.inf
  max_v_turning: float = 0.0
  driver_frames: int = 0
  comma_frames: int = 0
  braked: bool = False
  gassed: bool = False
  last_t: float | None = None
  learned: int = 0
  done: bool = False             # this blinker use already gave its sample
  k_smooth: float = 0.0
  k_peak: float = 0.0

  def reset(self) -> None:
    self.side, self.sharp, self.heading_deg, self.min_v, self.max_v_turning = None, 'unknown', 0.0, math.inf, 0.0
    self.driver_frames = self.comma_frames = 0
    self.braked = self.gassed = False
    self.done = False
    self.k_smooth = self.k_peak = 0.0

  def observe(self, t: float, side: str | None, v: float, yaw_rate: float, long_active: bool, gas: bool, brake: bool,
              angle_deg: float | None = None) -> str | None:
    """Feed every plan; returns the sample's key when a turn was learned."""
    dt = 0.0 if self.last_t is None else min(max(t - self.last_t, 0.0), 0.5)
    self.last_t = t
    if side is None or v > LEARN_MAX_MPS + 3 * MPH:
      if side is None and self.side is not None:
        self.reset()   # blinker off before the turn finished: not a turn
      elif side is not None:
        self.reset()
      return None
    if side != self.side:
      self.reset()
      self.side = side
    if self.done:
      return None
    if angle_deg is not None and self.sharp == 'unknown':
      self.sharp = sharpness(angle_deg)
    if math.isfinite(yaw_rate):
      self.heading_deg += math.degrees(yaw_rate) * dt
      if v > CURVATURE_MIN_V:   # how tight the turn is: smoothed yaw / speed, its peak through the turn
        self.k_smooth += (abs(yaw_rate) / v - self.k_smooth) * min(1.0, dt / 0.5)
        self.k_peak = max(self.k_peak, self.k_smooth)
    turned = abs(self.heading_deg)
    if turned >= 15.0:
      self.min_v = min(self.min_v, v)
      self.max_v_turning = max(self.max_v_turning, v)
      if long_active:
        self.comma_frames += 1
      else:
        self.driver_frames += 1
      self.braked |= brake and long_active
      self.gassed |= gas and long_active
    if turned < TURN_DONE_DEG or not math.isfinite(self.min_v):
      return None
    key = TurnSpeeds.key(self.side, self.sharp)
    self.speeds.learn_curvature(self.side, self.sharp, self.k_peak)
    if self.driver_frames >= self.comma_frames:
      self.speeds.learn(self.side, self.sharp, self.min_v)                  # the driver's own speed
    elif self.braked:
      self.speeds.learn(self.side, self.sharp, self.min_v)                  # braked down to this: slower
    elif self.gassed:
      self.speeds.learn(self.side, self.sharp, self.max_v_turning)          # pressed on to this: faster
    else:
      self.speeds.learn(self.side, self.sharp, self.min_v, CONFIRM_WEIGHT)  # the comma's speed was fine
    self.learned += 1
    self.done = True   # no second sample until the blinker goes off
    return key


def speeds_path() -> Path:
  from openpilot.starpilot.storage import starpilot_storage_root
  return starpilot_storage_root() / 'turn_speed.json'


class TurnSpeedHost:
  """plannerd: the turn-speed ceiling for this plan (or None), learning from every signalled turn as it goes."""

  def __init__(self, path: Path | None = None, status_path: Path = STATUS_PATH, clock=None):
    import time
    self.path = path
    self.status_path = status_path
    self.clock = clock or time.monotonic
    self.speeds: TurnSpeeds | None = None
    self.learner: TurnLearner | None = None
    self.log = None

  def _ensure(self) -> None:
    if self.speeds is None:
      self.path = self.path if self.path is not None else speeds_path()
      self.speeds = TurnSpeeds.load(self.path)
      self.learner = TurnLearner(self.speeds)
      from openpilot.starpilot.diagnostics.features import ChangeLog
      self.log = ChangeLog('turn_speed', min_gap_s=1.0)

  def sample(self, sm) -> float | None:
    try:
      self._ensure()
      now = self.clock()
      cs, cc = sm['carState'], sm['carControl']
      status = read_status(now, self.status_path)
      side = 'left' if cs.leftBlinker and not cs.rightBlinker else 'right' if cs.rightBlinker and not cs.leftBlinker else None
      if status is not None and status['side'] != side:
        status = None   # stale side: the blinker changed
      long_active = bool(cc.longActive and cc.enabled)
      learned = self.learner.observe(now, side, float(cs.vEgo), float(cs.yawRate), long_active, bool(cs.gasPressed),
                                     bool(cs.brakePressed), status.get('angle_deg') if status else None)
      if learned is not None:
        self.speeds.save(self.path)
      cap, v_turn = ceiling(status, self.speeds)
      if self.log is not None:
        self.log.observe({'slowing': cap is not None and cap < float(cs.vEgo) + 0.5, 'turning': bool(status and status.get('turning')),
                          'sharpness': sharpness(status.get('angle_deg')) if status else None},
                         {'turn_mph': round(v_turn / MPH, 1) if v_turn else None, 'cap_mph': round(cap / MPH, 1) if cap else None,
                          'distance_m': status.get('distance_m') if status else None, 'v_mph': round(float(cs.vEgo) / MPH, 1),
                          'learned': learned}, now=now)
      return cap if long_active else None
    except Exception as e:
      from openpilot.starpilot.diagnostics.features import log_error
      log_error('turn_speed', e, where='host')
      return None
