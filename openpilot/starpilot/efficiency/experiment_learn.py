"""Self-learning fuel efficiency, the judging half: which variant of each experiment (experiment.py) used the least fuel.

Fuel is the BlueDriver's measured gallons per stretch, adjusted for the hybrid battery (charge used is fuel spent
earlier, charge gained is fuel banked: ledger.GAL_PER_BATTERY_KWH), then corrected for the road and the speed change:
the fuel a stretch spent (or saved) climbing (its height change, from the pitch) and changing speed (half the change of
speed squared) is removed, at costs fitted on every stored stretch (fuel = a * metres + b * climb + c * speed change), so
a variant is not flattered by happening to run slightly downhill or by ending slower. Two ways of scoring a variant:
  - steady situations (hills, cruise, Pulse and Glide, following, EV): gallons per mile over its stretches;
  - launches and speeding up: gallons per mile over a fixed distance from where each episode began (WINDOW_M), so a
    gentle start is not flattered by covering less road in the same time.
Fair comparisons (2026-11-15):
  - season: only runs from the same outside-temperature band as the newest drive count (SEASON_BANDS, the BlueDriver's
    ambient reading), so summer runs are never weighed against winter ones;
  - curves: each variant's fuel and time are taken per curve class (straight / gentle curves / curvy, from the yaw
    rate) and weighted alike for every variant, over the classes every variant has driven.
A variant qualifies when every variant has enough measurements (MIN_MILES / MIN_EPISODES), it beats the baseline (the
comma's normal driving) by MIN_GAIN and stays ahead of it in CONFIDENCE of bootstrap resamples. Which qualifying variant
is used weighs fuel against time: time-free savings always, then the driver's budget (experiment.TIME_BUDGET_S_PER_H)
is spent across all situations on the steps that save the most fuel per extra minute, so the minutes go where they
pay most and a dearer step that adds little is left out (diminishing returns). A step must also save at least
MIN_GAL_PER_MINUTE per extra minute, budget left or not: below that the time is not worth the fuel.
The winners go to <efficiency>/experiments.json, which the planner's experimenter reads at the start of each drive.
"""
from __future__ import annotations

import json
import math
import random
import statistics
from pathlib import Path
from typing import NamedTuple

import numpy as np

from openpilot.starpilot.efficiency.experiment import CATEGORIES, TIME_BUDGET_S_PER_H
from openpilot.starpilot.efficiency.ledger import BATTERY_KWH, GAL_PER_BATTERY_KWH

METERS_PER_MILE = 1609.344
MIN_MILES = 3.0          # per variant, steady situations
MIN_EPISODES = 8         # per variant, launches and speeding up
MIN_GAIN = 0.03          # 3 % less fuel than the baseline
CONFIDENCE = 0.8
RESAMPLES = 400
WINDOW_M = {'launch': 400.0, 'accelerate': 600.0}
# physics fallback for the corrections (until enough stretches to fit them): a 1450 kg car at 30 % engine-to-wheel
MASS_KG, G, ETA, J_PER_GAL = 1450.0, 9.81, 0.30, 121.3e6
DEFAULT_CLIMB = MASS_KG * G / (ETA * J_PER_GAL)   # gallons per metre climbed (about 0.0004)
DEFAULT_SPEED = MASS_KG / (ETA * J_PER_GAL)       # gallons per (m/s)^2 of half-speed-squared gained
MIN_FIT_STRETCHES = 300
SEASON_BANDS = ((10.0, 'cold'), (22.0, 'mild'), (math.inf, 'warm'))   # outside temperature, C: upper bound, name
STRATUM_MIN_MILES = 0.5   # a curve class counts only when every variant drove at least this much of it
STRATUM_MIN_EPISODES = 2
MIN_GAL_PER_MINUTE = 0.02  # a minute must save at least this much fuel (~6-7 cents at $3.20/gal, ~$4 per hour of time)


class Run(NamedTuple):
  gal: float
  metres: float
  seconds: float = 0.0
  curve: str = 'straight'
  season: str | None = None


def season_of(ambient_c) -> str | None:
  if not isinstance(ambient_c, (int, float)) or not math.isfinite(ambient_c):
    return None
  return next(name for top, name in SEASON_BANDS if ambient_c < top)


def current_season(drives: list[list[dict]]) -> str | None:
  """The temperature band of the newest drive that has a temperature reading."""
  for stretches in reversed(drives):
    temps = [s['ambient_c'] for s in stretches if isinstance(s, dict) and isinstance(s.get('ambient_c'), (int, float))]
    if temps:
      return season_of(statistics.median(temps))
  return None


def _curve(stretch: dict) -> str:
  ctx = stretch.get('ctx')
  return str(ctx[2]) if isinstance(ctx, (list, tuple)) and len(ctx) > 2 else 'straight'


def climb_m(stretch: dict) -> float:
  pitch, metres = stretch.get('pitch'), stretch.get('m') or 0.0
  return metres * math.sin(pitch) if isinstance(pitch, (int, float)) and math.isfinite(pitch) else 0.0


def speed_change(stretch: dict) -> float:
  v0, v1 = stretch.get('v0'), stretch.get('v1')
  return (v1 * v1 - v0 * v0) / 2.0 if isinstance(v0, (int, float)) and isinstance(v1, (int, float)) else 0.0


def fit_corrections(drives: list[list[dict]]) -> dict:
  """Fuel per metre climbed and per unit of speed gained, fitted on every stored stretch with measured fuel (least
  squares, fuel = a * metres + b * climb + c * speed change); physics defaults when too few or not physical."""
  rows = [(s.get('m') or 0.0, climb_m(s), speed_change(s), g) for d in drives for s in d
          if (g := adjusted_gal(s)) is not None and (s.get('m') or 0.0) > 50.0]
  out = {'climb': DEFAULT_CLIMB, 'speed': DEFAULT_SPEED, 'source': 'physics', 'stretches': len(rows)}
  with_speed = [r for r in rows if r[2] != 0.0]
  if len(rows) < MIN_FIT_STRETCHES:
    return out
  x = np.array([[r[0], r[1], r[2]] for r in rows])
  y = np.array([r[3] for r in rows])
  if len(with_speed) < MIN_FIT_STRETCHES:
    x = x[:, :2]
  coef, *_ = np.linalg.lstsq(x, y, rcond=None)
  climb = float(coef[1])
  speed = float(coef[2]) if len(coef) > 2 else DEFAULT_SPEED
  # a fit far from physics (noise, too little hill data) is not trusted
  if 0.25 * DEFAULT_CLIMB <= climb <= 4 * DEFAULT_CLIMB:
    out.update(climb=climb, source='fitted')
  if len(coef) > 2 and 0.25 * DEFAULT_SPEED <= speed <= 4 * DEFAULT_SPEED:
    out.update(speed=speed)
  return out


def corrected_gal(stretch: dict, corrections: dict) -> float | None:
  gal = adjusted_gal(stretch)
  if gal is None:
    return None
  return gal - corrections['climb'] * climb_m(stretch) - corrections['speed'] * speed_change(stretch)


def adjusted_gal(stretch: dict) -> float | None:
  gal = stretch.get('obd_gal')
  if not isinstance(gal, (int, float)) or gal < 0:
    return None
  soc_d = stretch.get('soc_d')
  battery = (soc_d / 100.0) * BATTERY_KWH * GAL_PER_BATTERY_KWH if isinstance(soc_d, (int, float)) else 0.0
  return gal - battery   # charge used (soc_d < 0) adds the fuel it stands for


def samples(drives: list[list[dict]], corrections: dict | None = None,
            season: str | None = None) -> dict[tuple[str, str, str], list[Run]]:
  """(category, context, variant) -> runs per stretch (steady) or per episode window (launches): gallons corrected for
  climb and speed change, metres, seconds, curve class and season. With season, runs of other bands are left out."""
  corrections = corrections or {'climb': 0.0, 'speed': 0.0}
  out: dict[tuple[str, str, str], list[Run]] = {}
  for stretches in drives:
    prev_tag = None
    for i, s in enumerate(stretches):
      tag = s.get('exp')
      new_episode = tag != prev_tag
      prev_tag = tag
      if not isinstance(tag, str) or tag.count('|') != 2:
        continue
      category, context, variant = tag.split('|')
      if category not in CATEGORIES or variant not in CATEGORIES[category]:
        continue
      run_season = season_of(s.get('ambient_c'))
      if season is not None and run_season != season:
        continue
      if category in WINDOW_M:
        if not new_episode:
          continue
        gal, metres, seconds = 0.0, 0.0, 0.0
        for nxt in stretches[i:]:
          g, m = corrected_gal(nxt, corrections), nxt.get('m') or 0.0
          if g is None:
            gal = None
            break
          gal, metres, seconds = gal + g, metres + m, seconds + (nxt.get('dur') or 0.0)
          if metres >= WINDOW_M[category]:
            break
        if gal is not None and metres >= WINDOW_M[category]:
          out.setdefault((category, context, variant), []).append(Run(gal, metres, seconds, _curve(s), run_season))
      else:
        g, m = corrected_gal(s, corrections), s.get('m') or 0.0
        if g is not None and m > 0:
          out.setdefault((category, context, variant), []).append(Run(g, m, s.get('dur') or 0.0, _curve(s), run_season))
  return out


def gal_per_100mi(rows) -> float:
  metres = sum(r[1] for r in rows)
  return 100.0 * sum(r[0] for r in rows) / (metres / METERS_PER_MILE) if metres > 0 else float('inf')


def enough(category: str, rows) -> bool:
  if category in WINDOW_M:
    return len(rows) >= MIN_EPISODES
  return sum(r[1] for r in rows) / METERS_PER_MILE >= MIN_MILES


def _common_strata(category: str, rows: dict[str, list]) -> dict[str, float]:
  """Curve class -> weight, over the classes every variant has driven enough of (weights: all variants' metres)."""
  classes = set.intersection(*[{_run_curve(r) for r in rs} for rs in rows.values()]) if rows else set()
  keep = {}
  for c in classes:
    per = [[r for r in rs if _run_curve(r) == c] for rs in rows.values()]
    ok = (all(len(p) >= STRATUM_MIN_EPISODES for p in per) if category in WINDOW_M else
          all(sum(r[1] for r in p) / METERS_PER_MILE >= STRATUM_MIN_MILES for p in per))
    if ok:
      keep[c] = sum(r[1] for p in per for r in p)
  total = sum(keep.values())
  return {c: m / total for c, m in keep.items()} if total > 0 else {}


def _run_curve(r) -> str:
  return r[3] if len(r) > 3 else 'straight'


def _rates(rows: list, strata: dict[str, float]) -> tuple[float, float]:
  """(gallons per 100 mi, minutes per 100 mi) weighted over the curve classes."""
  gal = minutes = 0.0
  for c, w in strata.items():
    part = [r for r in rows if _run_curve(r) == c]
    miles = sum(r[1] for r in part) / METERS_PER_MILE
    if miles <= 0:
      return float('inf'), float('inf')
    gal += w * 100.0 * sum(r[0] for r in part) / miles
    minutes += w * 100.0 * sum((r[2] if len(r) > 2 else 0.0) for r in part) / 60.0 / miles
  return gal, minutes


def _resample(rows: list, rng: random.Random) -> list:
  by_curve: dict[str, list] = {}
  for r in rows:
    by_curve.setdefault(_run_curve(r), []).append(r)
  return [rng.choice(part) for part in by_curve.values() for _ in part]


def judge(data: dict, rng: random.Random | None = None, *, situation_miles_per_hour: dict[str, float] | None = None,
          budget_min_per_hr: float = TIME_BUDGET_S_PER_H / 60.0) -> dict[str, dict]:
  """'category|context' -> {'winner': variant or None, 'gal_per_100mi', 'min_per_100mi', 'status', ...}.

  situation_miles_per_hour: how many miles an engaged hour spends in each situation (the time budget is spent by fuel
  saved per extra minute across situations); without it every situation is given the whole budget."""
  rng = rng or random.Random(0)
  results, options = {}, {}
  for category, variants in CATEGORIES.items():
    for context in sorted({k[1] for k in data if k[0] == category}):
      key = f'{category}|{context}'
      rows = {v: data.get((category, context, v), []) for v in variants}
      entry = {'winner': None, 'gal_per_100mi': {}, 'min_per_100mi': {},
               'measured': {v: (len(r) if category in WINDOW_M else round(sum(x[1] for x in r) / METERS_PER_MILE, 2))
                            for v, r in rows.items()}}
      results[key] = entry
      missing = [v for v in variants if not enough(category, rows[v])]
      strata = _common_strata(category, rows) if not missing else {}
      if missing or not strata:
        entry['status'] = (f"collecting ({', '.join(missing)} need more)" if missing else
                           'collecting (no curve class every variant has driven enough)')
        continue
      rates = {v: _rates(r, strata) for v, r in rows.items()}
      entry['gal_per_100mi'] = {v: round(g, 3) for v, (g, _) in rates.items()}
      entry['min_per_100mi'] = {v: round(t, 2) for v, (_, t) in rates.items()}
      entry['curves'] = {c: round(w, 2) for c, w in strata.items()}
      boots = []
      for _ in range(RESAMPLES):
        boots.append({v: _rates(_resample(r, rng), strata)[0] for v, r in rows.items()})
      base = variants[0]
      base_gal, base_min = rates[base]
      candidates = []
      for v in variants[1:]:
        gal, minutes = rates[v]
        gain = 1.0 - gal / base_gal if base_gal > 0 else 0.0
        confidence = sum(b[v] < b[base] for b in boots) / RESAMPLES
        if gain >= MIN_GAIN and confidence >= CONFIDENCE:
          candidates.append((v, base_gal - gal, max(0.0, minutes - base_min), gain, confidence))
      best = min(rates, key=lambda v: rates[v][0])
      entry['confidence'] = round(sum(all(b[best] <= b[v] for v in variants) for b in boots) / RESAMPLES, 2)
      if not candidates:
        if best == base and entry['confidence'] >= CONFIDENCE:
          entry['winner'], entry['status'] = base, 'normal driving is best'
        elif best == base:
          entry['status'] = 'no clear winner yet'
        else:
          gain = 1.0 - rates[best][0] / base_gal if base_gal > 0 else 0.0
          entry['status'] = f'{best} leads by {100 * gain:.1f}% but not yet sure'
        continue
      # the frontier: cheapest first, each dearer variant only if it saves more
      frontier, top = [], 0.0
      for c in sorted(candidates, key=lambda c: (c[2], -c[1])):
        if c[1] > top:
          frontier.append(c)
          top = c[1]
      options[key] = frontier
      entry['winner'] = base
  _spend_budget(results, options, situation_miles_per_hour, budget_min_per_hr)
  return results


def _spend_budget(results: dict, options: dict, miles_per_hour: dict[str, float] | None, budget: float) -> None:
  """Time-free savings first, then the budget on the steps with the most fuel saved per extra minute."""
  chosen: dict[str, tuple] = {}
  for key, frontier in options.items():
    free = [c for c in frontier if c[2] <= 0.0]
    if free:
      chosen[key] = max(free, key=lambda c: c[1])
  left = budget
  while True:
    steps = []
    for key, frontier in options.items():
      cur = chosen.get(key)
      saved, cost = (cur[1], cur[2]) if cur else (0.0, 0.0)
      nxt = next((c for c in frontier if c[2] > cost and c[1] > saved), None)
      if nxt is None:
        continue
      share = (miles_per_hour or {}).get(key)
      per_hr = (nxt[2] - cost) * share / 100.0 if share is not None else 0.0   # minutes per engaged hour
      ratio = (nxt[1] - saved) / (nxt[2] - cost)   # gal/100 mi over min/100 mi: gallons per extra minute
      if ratio >= MIN_GAL_PER_MINUTE:
        steps.append((ratio, per_hr, key, nxt))
    affordable = [st for st in steps if st[1] <= left + 1e-9]
    if not affordable:
      break
    ratio, per_hr, key, nxt = max(affordable)
    chosen[key] = nxt
    left -= per_hr
  for key, frontier in options.items():
    entry = results[key]
    best = max(frontier, key=lambda c: c[1])
    if key in chosen:
      v, saved, cost, gain, confidence = chosen[key]
      entry['winner'] = v
      value = 'no extra time' if cost <= 0 else f'{cost:.1f} min/100 mi, {saved / cost:.3f} gal per extra minute'
      entry['status'] = f'{v} saves {100 * gain:.1f}% ({saved:.2f} gal/100 mi) for {value}'
      if best[0] != v:
        entry['status'] += f'; {best[0]} saves more but the extra minutes pay less'
    elif best[2] > 0 and best[1] / best[2] < MIN_GAL_PER_MINUTE:
      entry['status'] = (f'{best[0]} saves {100 * best[3]:.1f}% but only {best[1] / best[2]:.3f} gal per extra minute: '
                         + 'not worth the time')
    else:
      entry['status'] = (f'{best[0]} saves {100 * best[3]:.1f}% but costs {best[2]:.1f} min/100 mi: '
                         + 'the time budget saves more fuel elsewhere')
  results['_budget'] = {'budget_min_per_hr': round(budget, 2), 'left_min_per_hr': round(left, 2)}


def situation_miles(drives: list[list[dict]], season: str | None = None) -> dict[str, float]:
  """'category|context' -> miles per engaged hour spent in it (comma-driven stretches of the same season)."""
  miles: dict[str, float] = {}
  hours = 0.0
  for stretches in drives:
    for s in stretches:
      if not isinstance(s, dict) or s.get('mode') == 'driver' or (season is not None and season_of(s.get('ambient_c')) != season):
        continue
      hours += (s.get('dur') or 0.0) / 3600.0
      tag = s.get('exp')
      if isinstance(tag, str) and tag.count('|') == 2:
        key = '|'.join(tag.split('|')[:2])
        miles[key] = miles.get(key, 0.0) + (s.get('m') or 0.0) / METERS_PER_MILE
  return {k: m / hours for k, m in miles.items()} if hours > 0 else {}


def load_drives(root: Path) -> list[list[dict]]:
  drives = []
  for path in sorted((root / 'drives').glob('*.json'), key=lambda p: p.stat().st_mtime):
    try:
      drives.append(list(json.loads(path.read_text()).get('stretches', [])))
    except (OSError, ValueError, TypeError, AttributeError):
      continue
  return drives


def update(root: Path) -> dict:
  """Judge every stored drive and write <root>/experiments.json (the winners the experimenter reads)."""
  drives = load_drives(root)
  corrections = fit_corrections(drives)
  season = current_season(drives)
  results = judge(samples(drives, corrections, season), situation_miles_per_hour=situation_miles(drives, season))
  budget = results.pop('_budget', None)
  doc = {'version': 1, 'winners': {k: r['winner'] for k, r in results.items() if r['winner']}, 'results': results,
         'corrections': corrections, 'season': season, 'budget': budget}
  path = root / 'experiments.json'
  tmp = path.with_suffix('.tmp')
  tmp.write_text(json.dumps(doc, indent=1))
  tmp.replace(path)
  return doc


def report_lines(doc: dict) -> list[str]:
  c = doc.get('corrections') or {}
  lines = ['Fuel experiments (BlueDriver, battery-adjusted, corrected for climb and speed change'
           + (f": {c.get('climb', 0) * 1000:.2f} gal per km climbed, {c.get('source', '?')})" if c else ')')]
  if doc.get('season'):
    lines.append(f"  season: {doc['season']} drives only (outside temperature band)")
  b = doc.get('budget') or {}
  if b:
    lines.append(f"  time budget: {b.get('budget_min_per_hr', 0):g} min/hr, {b.get('left_min_per_hr', 0):g} left after the chosen variants")
  for key, r in sorted(doc.get('results', {}).items()):
    scores = ', '.join(f'{v} {x:.2f}' for v, x in r['gal_per_100mi'].items())
    lines.append(f"  {key:22s} {r['status']}  [gal/100mi: {scores}]")
  return lines
