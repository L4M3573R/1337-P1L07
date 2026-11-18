"""Self-learning fuel efficiency with dials: instead of a few fixed variants, each situation has settings it tunes on
its own, starting from the comma's normal driving and closing in on what saves the most fuel (the driver, 2026-11-17:
"I wanted it to dial in on its own savings"; steady driving should find how far to let the speed dip before it
gets back on the throttle, and when Pulse and Glide is worth it at all).

Per situation (category|context, as in experiment.py) there is a center (the best setting found so far, starting at
normal) and a step per dial. An episode drives either the center or a probe: the center with one dial moved one step
up or down. experiment_learn judges each probe against the center on the same roads (curve class, climate, season,
corrected for climb and speed change); a probe that saves at least MIN_GAIN with CONFIDENCE becomes the new center
(so it keeps moving the way that pays), and once both directions of a dial have been measured without beating the
center, that dial's step halves (it closes in) until it reaches its finest step. A dial at its finest step with
neither side winning is settled. Moves that cost time are only accepted while the whole set of centers stays within
the driver's 4 minutes per engaged hour (experiment.TIME_BUDGET_S_PER_H); each episode's time also counts against
that drive's budget, and probes that cost time are not drawn once it is used.

A variant is written as its dial values, e.g. 'a0.85' or 'b2.0p1.2g0', so the fuel ledger records exactly what ran.
The driver's limits are dial ranges: dips and sags at most 2 mph under the set speed, downhill carry at most 3 over,
the Pulse and Glide band at most 5 mph, never a shorter following distance (following only shapes acceleration).
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

MPH = 0.44704
EXPLOIT = 0.5            # an episode drives the center this often, a probe otherwise
PULSE_GLIDE_ON_MPH = 0.75   # a Pulse and Glide band under this is off
DIALS_EPOCH = 1          # dial definitions; a state file of another epoch starts over


@dataclass(frozen=True)
class Dial:
  key: str        # one letter, as written in the variant
  name: str
  low: float
  high: float
  start: float    # the comma's normal driving
  step: float     # first probe distance
  finest: float   # the step it stops halving at
  slower: int     # +1: a higher value is slower driving, -1: a lower value is, 0: neither (no time cost, no veto)
  digits: int = 2

  def clamp(self, x: float) -> float:
    return min(self.high, max(self.low, x))

  def fmt(self, x: float) -> str:
    return f'{x:.{self.digits}f}'


# category -> its dials. Acceleration shares are of the planner's own cruise acceleration at that speed.
DIALS: dict[str, tuple[Dial, ...]] = {
  'launch': (Dial('a', 'take-off strength', 0.5, 1.0, 1.0, 0.15, 0.05, -1),),
  'accelerate': (Dial('a', 'acceleration strength', 0.5, 1.0, 1.0, 0.15, 0.05, -1),
                 Dial('e', 'ease off before the set speed (mph)', 0.0, 6.0, 0.0, 2.0, 0.5, +1, 1)),
  'follow': (Dial('a', 'speed-up strength behind a car', 0.4, 1.0, 1.0, 0.2, 0.05, -1),),
  'ev': (Dial('a', 'acceleration strength in EV range', 0.25, 1.0, 1.0, 0.25, 0.05, -1),),
  'stop': (Dial('r', 'stopping rate (m/s2; lower starts slowing earlier and gentler)', 0.5, 2.0, 2.0, 0.5, 0.1, -1),),
  'pulse_glide': (Dial('b', 'Pulse and Glide band (mph; 0 = off)', 0.0, 5.0, 0.0, 2.0, 0.5, +1, 1),
                  Dial('p', 'pulse strength (share of the normal pulse)', 0.6, 1.4, 1.0, 0.2, 0.05, 0),
                  Dial('g', 'glide regen (m/s2 beyond the plain coast; 0 = coast)', 0.0, 0.6, 0.0, 0.2, 0.05, 0)),
  'cruise': (Dial('u', 'cruise under the set speed (mph)', 0.0, 2.0, 0.0, 1.0, 0.25, +1),),
  'hold': (Dial('d', 'dip allowed before the throttle comes back (mph)', 0.0, 2.0, 0.0, 1.0, 0.25, +1),
           Dial('k', 'recovery strength after a dip', 0.3, 1.0, 1.0, 0.25, 0.05, -1)),
  'uphill': (Dial('s', 'sag on uphills (mph)', 0.0, 2.0, 0.0, 1.0, 0.25, +1),),
  'downhill': (Dial('c', 'carry over the set speed downhill (mph)', 0.0, 3.0, 0.0, 1.5, 0.25, 0),),
}

_PART = re.compile(r'([a-z])(-?\d+(?:\.\d+)?)')


def start_values(category: str) -> dict[str, float]:
  return {d.key: d.start for d in DIALS[category]}


def encode(category: str, values: dict[str, float]) -> str:
  return ''.join(d.key + d.fmt(d.clamp(values.get(d.key, d.start))) for d in DIALS[category])


def decode(category: str, variant: str) -> dict[str, float] | None:
  """Dial values of a variant string, or None when it is not a valid one for this category (an old named variant)."""
  if category not in DIALS or not isinstance(variant, str):
    return None
  parts = _PART.findall(variant)
  if ''.join(k + v for k, v in parts) != variant:
    return None
  values = {k: float(v) for k, v in parts}
  dials = {d.key: d for d in DIALS[category]}
  if set(values) != set(dials) or any(not dials[k].low - 1e-9 <= x <= dials[k].high + 1e-9 for k, x in values.items()):
    return None
  return values


def is_dial_variant(category: str, variant: str) -> bool:
  return decode(category, variant) is not None


# ---------- per-situation state (center and steps), stored in experiments.json ----------
@dataclass
class Situation:
  center: dict[str, float]
  step: dict[str, float]
  cost_min_per_100mi: float = 0.0   # the center's extra time over normal driving
  moves: int = 0

  def probes(self, category: str) -> list[tuple[str, str, int]]:
    """(variant, dial key, direction) for every one-step move of one dial that stays in range."""
    out = []
    for d in DIALS[category]:
      if category == 'pulse_glide' and d.key in ('p', 'g') and self.center['b'] < PULSE_GLIDE_ON_MPH:
        continue   # with Pulse and Glide off its pulse and glide change nothing: only the band is worth probing
      for direction in (+1, -1):
        x = d.clamp(self.center[d.key] + direction * self.step[d.key])
        if abs(x - self.center[d.key]) > 1e-9:
          out.append((encode(category, {**self.center, d.key: x}), d.key, direction))
    return out

  def as_doc(self) -> dict:
    return {'center': self.center, 'step': self.step, 'cost_min_per_100mi': round(self.cost_min_per_100mi, 3),
            'moves': self.moves}


def new_situation(category: str) -> Situation:
  return Situation(start_values(category), {d.key: d.step for d in DIALS[category]})


def load_state(path: Path | None) -> dict[str, Situation]:
  """'category|context' -> Situation; {} (start over) when there is no valid state of this DIALS_EPOCH."""
  try:
    doc = json.loads(path.read_text()) if path is not None else {}
    dials = doc.get('dials', {}) if isinstance(doc, dict) else {}
    if not isinstance(dials, dict) or dials.get('_epoch') != DIALS_EPOCH:
      return {}
    out = {}
    for key, s in dials.items():
      category = key.split('|')[0]
      if key == '_epoch' or category not in DIALS or not isinstance(s, dict):
        continue
      center = decode(category, encode(category, s.get('center', {})))
      step = s.get('step', {})
      if center is None or not isinstance(step, dict):
        continue
      steps = {d.key: min(d.step, max(d.finest, float(step.get(d.key, d.step)))) for d in DIALS[category]}
      out[key] = Situation(center, steps, float(s.get('cost_min_per_100mi', 0.0)), int(s.get('moves', 0)))
    return out
  except (OSError, ValueError, TypeError, AttributeError):
    return {}


def state_doc(state: dict[str, Situation]) -> dict:
  return {'_epoch': DIALS_EPOCH, **{k: s.as_doc() for k, s in sorted(state.items())}}


# ---------- what a setting does to the plan, and what it costs in time ----------
def slower_than(category: str, values: dict[str, float], other: dict[str, float]) -> bool:
  """values drives slower than other on at least one dial (so it can cost time / be vetoed)."""
  return any(d.slower and (values[d.key] - other[d.key]) * d.slower > 1e-9 for d in DIALS[category])


def costs_time(category: str, values: dict[str, float]) -> bool:
  return slower_than(category, values, start_values(category))


def vetoed_values(category: str, context: str, vetoes: dict[str, int], limit: int) -> list[dict[str, float]]:
  """The dial settings the driver pressed on during in this situation (slower-than-normal ones only)."""
  prefix = f'{category}|{context}|'
  out = []
  for key, count in vetoes.items():
    if count >= limit and key.startswith(prefix):
      values = decode(category, key[len(prefix):])
      if values is not None and costs_time(category, values):
        out.append(values)
  return out


def blocked_by_veto(category: str, values: dict[str, float], vetoed: list[dict[str, float]]) -> bool:
  """A veto is a bound, not one setting: what was too slow for the driver rules out every setting that is as slow or
  slower on every dial (2026-11-17 review: with 1 mph under vetoed, 2 mph under was still drawn)."""
  slow = [d for d in DIALS.get(category, ()) if d.slower]
  return bool(slow) and any(all((values[d.key] - v[d.key]) * d.slower >= -1e-9 for d in slow) for v in vetoed)


def step_back(category: str, center: dict[str, float], step: dict[str, float], vetoed: list[dict[str, float]]) -> dict[str, float]:
  """The center, moved toward normal driving on every slower dial until no veto covers it."""
  center = dict(center)
  for _ in range(64):
    if not blocked_by_veto(category, center, vetoed):
      break
    moved = False
    for d in DIALS[category]:
      if d.slower and (center[d.key] - d.start) * d.slower > 1e-9:
        x = center[d.key] - d.slower * max(step.get(d.key, d.finest), d.finest)
        center[d.key] = d.clamp(max(x, d.start) if d.slower > 0 else min(x, d.start))
        moved = True
    if not moved:
      break
  return center


@dataclass(frozen=True)
class Effect:
  target_offset_mps: float = 0.0      # under (negative) or over (positive, downhill) the set speed, ramped in
  accel_max: float | None = None      # cap on the cruise acceleration
  stop_rate: float | None = None      # a stop: hold one steady slowdown at this rate from first sight
  pulse_glide: tuple[float, float, float] | None = None   # band (m/s), pulse scale, glide extra: Pulse and Glide on
  pulse_glide_off: bool = False       # Pulse and Glide explicitly off (band 0)


def effect(category: str, values: dict[str, float], *, v_ego: float, v_target: float, normal: float,
           below_target_mps: float) -> Effect:
  """What a setting does now. normal: the planner's own cruise acceleration at this speed; below_target_mps: how far
  the car is under the speed it is being asked for (its own target, an experiment's offset included)."""
  if category in ('launch', 'follow', 'ev'):
    return Effect(accel_max=values['a'] * normal if values['a'] < 1.0 - 1e-9 else None)
  if category == 'accelerate':
    a = values['a'] * normal
    ease = values['e'] * MPH
    if ease > 0 and below_target_mps < ease:
      a *= 0.35 + 0.65 * max(below_target_mps, 0.0) / ease   # tapers toward the set speed instead of arriving hard
    return Effect(accel_max=a if a < normal - 1e-9 else None)
  if category == 'stop':
    return Effect(stop_rate=values['r'] if values['r'] < 2.0 - 1e-9 else None)
  if category == 'pulse_glide':
    if values['b'] < PULSE_GLIDE_ON_MPH:
      return Effect(pulse_glide_off=True)
    return Effect(pulse_glide=(values['b'] * MPH, values['p'], values['g']))
  if category == 'cruise':
    return Effect(target_offset_mps=-values['u'] * MPH)
  if category == 'hold':
    dip = values['d'] * MPH
    if dip > 0 and below_target_mps < dip:
      return Effect(accel_max=0.0)   # the throttle stays where it is: the speed may dip this far before it acts
    if values['k'] < 1.0 - 1e-9 and below_target_mps > 0:
      return Effect(accel_max=values['k'] * normal)   # and climbs back this gently
    return Effect()
  if category == 'uphill':
    return Effect(target_offset_mps=-values['s'] * MPH)
  if category == 'downhill':
    return Effect(target_offset_mps=values['c'] * MPH)
  return Effect()


def episode_time_s(category: str, values: dict[str, float], *, v_ego: float, v_target: float, normal_at) -> float:
  """Extra seconds an episode starting now costs over normal driving, for the ones whose cost is known up front
  (speeding up: dv / 2a longer at a lower rate). The ongoing costs (under the set speed, dips, Pulse and Glide, stops)
  are counted per second by the experimenter."""
  if category not in ('launch', 'accelerate'):
    return 0.0
  dv = max(v_target - v_ego, 0.0)
  if dv <= 0:
    return 0.0
  normal = normal_at(v_ego + dv / 2.0)
  a = max(values['a'], 0.05) * normal
  extra = dv / 2.0 * (1.0 / a - 1.0 / normal)
  if category == 'accelerate' and values['e'] > 0:
    ease = min(values['e'] * MPH, dv)
    extra += ease / 2.0 * (1.0 / (0.675 * a) - 1.0 / a)   # the taper averages about two thirds of the rate
  return max(extra, 0.0)


def ongoing_time_s(category: str, values: dict[str, float], v_target: float, dt: float) -> float:
  """Extra seconds per dt of driving under the set speed: distance at a lower speed takes v_set / v longer."""
  if v_target <= 1.0:
    return 0.0
  under = {'cruise': values.get('u', 0.0), 'uphill': values.get('s', 0.0),
           'hold': values.get('d', 0.0) / 2.0,                      # a dip averages about half its allowance
           'pulse_glide': values.get('b', 0.0) / 2.0 if values.get('b', 0.0) >= PULSE_GLIDE_ON_MPH else 0.0}.get(category, 0.0)
  if under <= 0:
    return 0.0
  return dt * (v_target / max(v_target - under * MPH, 1.0) - 1.0)
