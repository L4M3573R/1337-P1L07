"""Self-learning fuel efficiency, the judging half: which variant of each experiment (experiment.py) used the least fuel.

Fuel is the BlueDriver's measured gallons per stretch, adjusted for the hybrid battery (charge used is fuel spent
earlier, charge gained is fuel banked: ledger.GAL_PER_BATTERY_KWH), then corrected for the road and the speed change:
the fuel a stretch spent (or saved) climbing (its height change, from the pitch) and changing speed (half the change of
speed squared) is removed, at costs fitted on every stored stretch (fuel = a * metres + b * climb + c * speed change), so
a variant is not flattered by happening to run slightly downhill or by ending slower. Two ways of scoring a variant:
  - steady situations (hills, cruise, Pulse and Glide, following, EV): gallons per mile over its stretches;
  - launches and speeding up: gallons per mile over a fixed distance from where each episode began (WINDOW_M), so a
    gentle start is not flattered by covering less road in the same time.
A variant wins only when every variant has enough measurements (MIN_MILES / MIN_EPISODES), it beats the baseline (the
comma's normal driving) by MIN_GAIN, and it stays ahead of every other variant in CONFIDENCE of bootstrap resamples.
The winners go to <efficiency>/experiments.json, which the planner's experimenter reads at the start of each drive.
"""
from __future__ import annotations

import json
import math
import random
from pathlib import Path

import numpy as np

from openpilot.starpilot.efficiency.experiment import CATEGORIES
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


def samples(drives: list[list[dict]], corrections: dict | None = None) -> dict[tuple[str, str, str], list[tuple[float, float]]]:
  """(category, context, variant) -> [(gallons, metres)] per stretch (steady) or per episode window (launches), the
  gallons corrected for climb and speed change."""
  corrections = corrections or {'climb': 0.0, 'speed': 0.0}
  out: dict[tuple[str, str, str], list[tuple[float, float]]] = {}
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
      if category in WINDOW_M:
        if not new_episode:
          continue
        gal, metres = 0.0, 0.0
        for nxt in stretches[i:]:
          g, m = corrected_gal(nxt, corrections), nxt.get('m') or 0.0
          if g is None:
            gal = None
            break
          gal, metres = gal + g, metres + m
          if metres >= WINDOW_M[category]:
            break
        if gal is not None and metres >= WINDOW_M[category]:
          out.setdefault((category, context, variant), []).append((gal, metres))
      else:
        g, m = corrected_gal(s, corrections), s.get('m') or 0.0
        if g is not None and m > 0:
          out.setdefault((category, context, variant), []).append((g, m))
  return out


def gal_per_100mi(rows: list[tuple[float, float]]) -> float:
  metres = sum(m for _, m in rows)
  return 100.0 * sum(g for g, _ in rows) / (metres / METERS_PER_MILE) if metres > 0 else float('inf')


def enough(category: str, rows: list[tuple[float, float]]) -> bool:
  if category in WINDOW_M:
    return len(rows) >= MIN_EPISODES
  return sum(m for _, m in rows) / METERS_PER_MILE >= MIN_MILES


def judge(data: dict, rng: random.Random | None = None) -> dict[str, dict]:
  """'category|context' -> {'winner': variant or None, 'gal_per_100mi': {variant: x}, 'status': text}."""
  rng = rng or random.Random(0)
  results = {}
  for category, variants in CATEGORIES.items():
    for context in sorted({k[1] for k in data if k[0] == category}):
      rows = {v: data.get((category, context, v), []) for v in variants}
      scores = {v: round(gal_per_100mi(r), 3) for v, r in rows.items() if r}
      entry = {'winner': None, 'gal_per_100mi': scores,
               'measured': {v: (len(r) if category in WINDOW_M else round(sum(m for _, m in r) / METERS_PER_MILE, 2))
                            for v, r in rows.items()}}
      missing = [v for v in variants if not enough(category, rows[v])]
      if missing:
        entry['status'] = f"collecting ({', '.join(missing)} need more)"
        results[f'{category}|{context}'] = entry
        continue
      best = min(scores, key=scores.get)
      base = variants[0]
      gain = 1.0 - scores[best] / scores[base] if scores[base] > 0 else 0.0
      wins = 0
      for _ in range(RESAMPLES):
        boot = {v: gal_per_100mi([rng.choice(r) for _ in r]) for v, r in rows.items()}
        wins += all(boot[best] <= boot[v] for v in variants)
      confidence = wins / RESAMPLES
      entry['confidence'] = round(confidence, 2)
      if best == base:
        entry['winner'] = base if confidence >= CONFIDENCE else None
        entry['status'] = 'normal driving is best' if entry['winner'] else 'no clear winner yet'
      elif gain >= MIN_GAIN and confidence >= CONFIDENCE:
        entry['winner'] = best
        entry['status'] = f'{best} saves {100 * gain:.1f}%'
      else:
        entry['status'] = f'{best} leads by {100 * gain:.1f}% but not yet sure (confidence {confidence:.0%})'
      results[f'{category}|{context}'] = entry
  return results


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
  results = judge(samples(drives, corrections))
  doc = {'version': 1, 'winners': {k: r['winner'] for k, r in results.items() if r['winner']}, 'results': results,
         'corrections': corrections}
  path = root / 'experiments.json'
  tmp = path.with_suffix('.tmp')
  tmp.write_text(json.dumps(doc, indent=1))
  tmp.replace(path)
  return doc


def report_lines(doc: dict) -> list[str]:
  c = doc.get('corrections') or {}
  lines = ['Fuel experiments (BlueDriver, battery-adjusted, corrected for climb and speed change'
           + (f": {c.get('climb', 0) * 1000:.2f} gal per km climbed, {c.get('source', '?')})" if c else ')')]
  for key, r in sorted(doc.get('results', {}).items()):
    scores = ', '.join(f'{v} {x:.2f}' for v, x in r['gal_per_100mi'].items())
    lines.append(f"  {key:22s} {r['status']}  [gal/100mi: {scores}]")
  return lines
