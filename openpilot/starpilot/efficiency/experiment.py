"""Self-learning fuel efficiency: while the comma controls speed it tries different ways of driving, the BlueDriver
measures each one, and the winner is kept per situation.

Each CATEGORY applies in one situation (a launch, a hill, cruising, following a car, ...) and has variants. When the car
enters that situation (an episode) one variant is drawn: each variant until it has been tried, then the learned winner
4 times in 5 and another one 1 time in 5, so a change (season, tires) is still noticed. A variant only shapes the plan
through limits the planner already takes the minimum of: a lower cruise target, a lower acceleration cap, Pulse and
Glide on or off. The one exception is the downhill carry, up to DOWNHILL_CARRY_MPH over the set speed (the driver's
allowance). Variants that cost time are not drawn once the drive has used its time budget (TIME_BUDGET_S_PER_H).
Following a car, Force Stop, curve speed and the driver's pedals always come first: the planner applies these limits
after its own, and every episode ends the moment the driver touches a pedal.

plannerd calls Experimenter.sample() every plan; efficiencyd tags the fuel ledger with tag() of the episode in force;
experiment_learn turns the tagged stretches into the winners file read here (load_winners).
"""
from __future__ import annotations

import json
import math
import random
from dataclasses import dataclass, field
from pathlib import Path

MPH = 0.44704
TIME_BUDGET_S_PER_H = 150.0    # at most 2.5 extra minutes per engaged hour (the driver's limit: 2-3)
BUDGET_GRACE_S = 30.0          # what a drive may spend before its first engaged minutes have earned it
EXPLORE = 0.2                  # after learning, a non-winning variant is still tried this often
EPISODE_M = 1000.0             # cruising and following episodes are redrawn every kilometre
GRADE_ENTER = 0.025            # a hill episode starts past 2.5 % grade...
GRADE_EXIT = 0.015             # ...and lasts until the grade is back under 1.5 %
MIN_EPISODE_S = 15.0           # an episode lasts at least this long (unless a pedal, a stop or disengaging ends it):
                               # shorter ones cannot be measured (the BlueDriver reads fuel about once a second)
CRUISE_MIN_MPS = 25 * MPH      # hill, cruise and Pulse and Glide experiments run above this
PULSE_GLIDE_MAX_MPS = 60 * MPH # above 60 mph Pulse and Glide measured 5 % worse than steady cruise (2026-11-14)
EV_MAX_MPS = 40 * MPH          # the hybrid's electric cruising range (the learner's EV cut-off refines it later)
EV_MIN_SOC = 45.0              # battery %: below this the engine runs anyway, so the EV experiment would only cost time
LAUNCH_END_MPS = 20 * MPH
LAUNCH_MAX_S = 20.0
FOLLOW_GAP_S = 3.0             # a car ahead within this time gap starts "following"...
FOLLOW_EXIT_GAP_S = 4.5        # ...which lasts until the gap opens past this
RAMP_MPS2 = 0.3                # how fast a speed offset is put on or taken off
DOWNHILL_CARRY_MPH = 2.0
ACCEL_VARIANTS = {'gentle': 0.5, 'moderate': 0.9}   # m/s^2 caps for launches and speeding up ('normal': the planner's)
NORMAL_ACCEL = 1.2             # about the planner's own cruise acceleration, for the time-cost estimate
ACCELERATE_GAP_MPS = 3 * MPH   # the set speed this far above the car's speed: an "accelerate" episode
FOLLOW_SMOOTH_ACCEL = 0.6
EV_GENTLE_ACCEL = 0.3
SPEED_BANDS = ((30 * MPH, '25-30'), (45 * MPH, '30-45'), (60 * MPH, '45-60'), (math.inf, '60+'))

# category -> variants (the first is the baseline: the comma's normal driving)
CATEGORIES: dict[str, tuple[str, ...]] = {
  'launch': ('normal', 'moderate', 'gentle'),
  'accelerate': ('normal', 'moderate', 'gentle'),
  'follow': ('normal', 'smooth'),
  'uphill': ('hold', 'sag3'),
  'downhill': ('hold', 'carry2'),
  'ev': ('normal', 'gentle'),
  'pulse_glide': ('steady', 'pg'),
  'cruise': ('set', 'under2', 'under4'),
}
COSTS_TIME = {('launch', 'gentle'), ('launch', 'moderate'), ('accelerate', 'gentle'), ('accelerate', 'moderate'),
              ('uphill', 'sag3'), ('pulse_glide', 'pg'), ('cruise', 'under2'), ('cruise', 'under4')}
UNDER_MPH = {'sag3': 3.0, 'under2': 2.0, 'under4': 4.0}


def band(v: float) -> str:
  return next(name for top, name in SPEED_BANDS if v < top)


@dataclass(frozen=True)
class Frame:
  t: float                      # monotonic seconds
  v_ego: float                  # m/s
  v_set: float                  # the driver's set speed, m/s (wheel speed)
  long_active: bool
  gas: bool = False
  brake: bool = False
  standstill: bool = False
  grade: float | None = None    # rise over run (pitch), None when unknown
  lead_gap_s: float = math.inf  # time gap to the car ahead
  stop_ahead: bool = False      # Force Stop, the model's stop, or a stopping lead
  soc: float | None = None      # hybrid battery %, from the BlueDriver
  pulse_glide_by_driver: bool = False  # the driver chose Pulse and Glide on or off this drive: leave it alone


@dataclass(frozen=True)
class Plan:
  speed_ceiling_mps: float | None = None   # lower the cruise target to this
  speed_bonus_mps: float = 0.0             # downhill carry: raise an un-limited cruise target by this
  accel_max: float | None = None           # cap the cruise acceleration
  pulse_glide: bool | None = None          # turn Pulse and Glide on / off (None: leave it)
  tag: str | None = None                   # 'category|context|variant' for the fuel ledger


@dataclass
class Episode:
  category: str
  context: str
  variant: str
  start_t: float
  start_m: float
  number: int

  def tag(self) -> str:
    return f'{self.category}|{self.context}|{self.variant}'


STATUS_PATH = Path('/dev/shm/starpilot_efficiency_experiment.json')
STATUS_MAX_AGE_S = 2.0


def write_status(doc: dict, path: Path = STATUS_PATH) -> None:
  """plannerd -> efficiencyd: the episode in force (atomic replace; a missing /dev/shm just skips it)."""
  try:
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(doc, separators=(',', ':')))
    tmp.replace(path)
  except OSError:
    pass


def read_tag(now: float, path: Path = STATUS_PATH) -> str | None:
  """The experiment tag in force, or None when plannerd's status is missing or stale (monotonic clock)."""
  try:
    doc = json.loads(path.read_text())
    if isinstance(doc, dict) and isinstance(doc.get('t'), (int, float)) and 0 <= now - doc['t'] <= STATUS_MAX_AGE_S:
      tag = doc.get('tag')
      return tag if isinstance(tag, str) and tag.count('|') == 2 else None
  except (OSError, ValueError, TypeError):
    pass
  return None


def load_winners(path: Path | None) -> dict[str, str]:
  """{'category|context': variant} learned so far; {} when there is no (valid) file yet."""
  try:
    if path is None:
      return {}
    doc = json.loads(path.read_text())
    winners = doc.get('winners', {}) if isinstance(doc, dict) else {}
    return {k: v for k, v in winners.items() if isinstance(k, str) and isinstance(v, str) and
            k.split('|')[0] in CATEGORIES and v in CATEGORIES[k.split('|')[0]]}
  except (OSError, ValueError, TypeError, AttributeError):
    return {}


@dataclass
class Experimenter:
  winners: dict[str, str] = field(default_factory=dict)
  tried: dict[str, int] = field(default_factory=dict)   # 'category|context|variant' -> episodes this drive
  rng: random.Random = field(default_factory=random.Random)
  episode: Episode | None = None
  distance_m: float = 0.0
  engaged_s: float = 0.0
  spent_s: float = 0.0           # estimated extra trip time from experiments this drive
  offset_mps: float = 0.0        # applied speed offset (negative: under the set speed), ramped
  episodes: int = 0
  last_t: float | None = None
  was_standstill: bool = False
  cruise_turn: int = 0           # alternates the cruising experiments between Pulse and Glide and cruise speed

  # ---------- budget ----------
  def budget_left_s(self) -> float:
    return BUDGET_GRACE_S + TIME_BUDGET_S_PER_H * self.engaged_s / 3600.0 - self.spent_s

  # ---------- situations ----------
  def situation(self, f: Frame) -> tuple[str, str] | None:
    """(category, context) the car is in now, or None (no experiment applies)."""
    if not f.long_active or f.gas or f.brake or f.stop_ahead:
      return None
    ep = self.episode
    if ep is not None and self._continues(ep, f):
      return ep.category, ep.context
    return self._enter(f)

  def _ev_ok(self, f: Frame, margin: float = 0.0) -> bool:
    return (5.0 < f.v_ego < EV_MAX_MPS + margin and f.soc is not None and f.soc >= EV_MIN_SOC and
            abs(f.v_ego - f.v_set) < 3.0 + margin)

  def _continues(self, ep: Episode, f: Frame) -> bool:
    if ep.category == 'launch':
      return f.v_ego < LAUNCH_END_MPS and f.t - ep.start_t < LAUNCH_MAX_S
    if f.standstill:
      return False
    if ep.category == 'accelerate':
      return f.v_set - f.v_ego > 1.0
    dwell = f.t - ep.start_t < MIN_EPISODE_S
    if ep.category == 'follow':
      return dwell or f.lead_gap_s < FOLLOW_EXIT_GAP_S
    if ep.category == 'uphill':
      return dwell or (f.grade is not None and f.grade > GRADE_EXIT)
    if ep.category == 'downhill':
      return dwell or (f.grade is not None and f.grade < -GRADE_EXIT)
    if ep.category == 'ev':
      return dwell or self._ev_ok(f, margin=1.0)
    # cruising (Pulse and Glide, cruise speed): until the road tilts into a hill or the speed drops away
    return dwell or (f.v_ego >= CRUISE_MIN_MPS - 2.0 and (f.grade is None or abs(f.grade) < GRADE_ENTER))

  def _enter(self, f: Frame) -> tuple[str, str] | None:
    if self.was_standstill and not f.standstill and f.v_ego < 3.0:
      return 'launch', 'launch'
    if f.standstill:
      return None
    if f.lead_gap_s < FOLLOW_GAP_S and f.v_ego > 5.0:
      return 'follow', band(f.v_ego)
    if f.v_ego > 5.0 and f.v_set - f.v_ego > ACCELERATE_GAP_MPS:
      return 'accelerate', band(f.v_set)   # learned per target speed
    if f.v_ego >= CRUISE_MIN_MPS and f.grade is not None and f.grade > GRADE_ENTER:
      return 'uphill', band(f.v_ego)
    if f.v_ego >= CRUISE_MIN_MPS and f.grade is not None and f.grade < -GRADE_ENTER:
      return 'downhill', band(f.v_ego)
    # level cruising: the experiments take turns
    options = []
    if f.v_ego >= CRUISE_MIN_MPS and f.v_set >= CRUISE_MIN_MPS:
      if not f.pulse_glide_by_driver and f.v_ego < PULSE_GLIDE_MAX_MPS:
        options.append(('pulse_glide', band(f.v_ego)))
      options.append(('cruise', band(f.v_ego)))
    if self._ev_ok(f):
      options.append(('ev', 'ev'))
    if not options:
      return None
    return options[self.cruise_turn % len(options)]

  def draw(self, category: str, context: str) -> str:
    variants = CATEGORIES[category]
    allowed = [v for v in variants if (category, v) not in COSTS_TIME or self.budget_left_s() > 0]
    if len(allowed) == 1:
      return allowed[0]
    winner = self.winners.get(f'{category}|{context}')
    if winner is None or winner not in allowed:
      # still learning: the least-tried variant this drive (ties at random), so every variant gets measured
      fewest = min(self.tried.get(f'{category}|{context}|{v}', 0) for v in allowed)
      return self.rng.choice([v for v in allowed if self.tried.get(f'{category}|{context}|{v}', 0) == fewest])
    if self.rng.random() < EXPLORE:
      return self.rng.choice([v for v in allowed if v != winner])
    return winner

  # ---------- per plan ----------
  def sample(self, f: Frame) -> Plan:
    try:
      return self._sample(f)
    except Exception as e:  # an experiment must never disturb the plan
      self.episode = None
      self.offset_mps = 0.0
      from openpilot.starpilot.diagnostics.features import log_error
      log_error('efficiency_experiment', e, where='sample')
      return Plan()

  def _sample(self, f: Frame) -> Plan:
    dt = 0.0 if self.last_t is None else min(max(f.t - self.last_t, 0.0), 0.5)
    self.last_t = f.t
    if f.long_active:
      self.engaged_s += dt
      self.distance_m += max(f.v_ego, 0.0) * dt
    where = self.situation(f)
    ep = self.episode
    if where is None or ep is None or (where[0], where[1]) != (ep.category, ep.context) or \
       (ep.category in ('follow', 'pulse_glide', 'cruise', 'ev') and self.distance_m - ep.start_m >= EPISODE_M):
      if ep is not None and ep.category in ('pulse_glide', 'cruise', 'ev'):
        self.cruise_turn += 1
      self.episode = ep = None
      if where is not None:
        variant = self.draw(*where)
        self.episodes += 1
        self.episode = ep = Episode(where[0], where[1], variant, f.t, self.distance_m, self.episodes)
        key = ep.tag()
        self.tried[key] = self.tried.get(key, 0) + 1
        if ep.category in ('launch', 'accelerate') and variant in ACCEL_VARIANTS:
          # speeding up by dv at a takes dv / (2 a) longer than being at speed: the extra over the normal rate
          dv = max(min(f.v_set, LAUNCH_END_MPS) if ep.category == 'launch' else f.v_set - f.v_ego, 0.0)
          self.spent_s += dv / 2.0 * (1.0 / ACCEL_VARIANTS[variant] - 1.0 / NORMAL_ACCEL)
    self.was_standstill = f.standstill
    return self._plan(f, ep, dt)

  def _plan(self, f: Frame, ep: Episode | None, dt: float) -> Plan:
    target_offset = 0.0
    accel_max = None
    pulse_glide = None
    if ep is not None:
      v = ep.variant
      if v in UNDER_MPH:
        target_offset = -UNDER_MPH[v] * MPH
      elif ep.category == 'downhill' and v == 'carry2':
        target_offset = DOWNHILL_CARRY_MPH * MPH
      elif ep.category in ('launch', 'accelerate') and v in ACCEL_VARIANTS:
        accel_max = ACCEL_VARIANTS[v]
      elif ep.category == 'follow' and v == 'smooth':
        accel_max = FOLLOW_SMOOTH_ACCEL
      elif ep.category == 'ev' and v == 'gentle':
        accel_max = EV_GENTLE_ACCEL
      if ep.category == 'pulse_glide' and not f.pulse_glide_by_driver:
        pulse_glide = v == 'pg'
    step = RAMP_MPS2 * dt
    self.offset_mps = min(max(target_offset, self.offset_mps - step), self.offset_mps + step)
    if f.gas or f.brake or not f.long_active:
      self.offset_mps = 0.0
    if self.offset_mps < 0 and f.v_set > 0:
      # time cost of driving under the set speed: distance at the lower speed takes v_set / v longer
      lower = max(f.v_set + self.offset_mps, 1.0)
      self.spent_s += dt * (f.v_set / lower - 1.0)
    if pulse_glide:
      self.spent_s += dt * 0.04  # Pulse and Glide averages about half its band under the set speed
    ceiling = f.v_set + self.offset_mps if self.offset_mps < 0 else None
    bonus = self.offset_mps if self.offset_mps > 0 else 0.0
    return Plan(ceiling, bonus, accel_max, pulse_glide, ep.tag() if ep is not None else None)

  def status(self) -> dict:
    ep = self.episode
    return {'tag': ep.tag() if ep is not None else None, 'episode': ep.number if ep is not None else 0,
            'spent_s': round(self.spent_s, 1), 'budget_left_s': round(self.budget_left_s(), 1),
            'engaged_min': round(self.engaged_s / 60.0, 1)}
