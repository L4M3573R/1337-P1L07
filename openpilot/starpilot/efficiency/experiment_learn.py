"""Self-learning fuel efficiency, the judging half: which variant of each experiment (experiment.py) used the least fuel.

Fuel is the BlueDriver's measured gallons per stretch, adjusted for the hybrid battery (charge used is fuel spent
earlier, charge gained is fuel banked: ledger.GAL_PER_BATTERY_KWH). Two ways of scoring a variant:
  - steady situations (hills, cruise, Pulse and Glide, following, EV): gallons per mile over its stretches;
  - launches and speeding up: gallons per mile over a fixed distance from where each episode began (WINDOW_M), so a
    gentle start is not flattered by covering less road in the same time.
A variant wins only when every variant has enough measurements (MIN_MILES / MIN_EPISODES), it beats the baseline (the
comma's normal driving) by MIN_GAIN, and it stays ahead of every other variant in CONFIDENCE of bootstrap resamples.
The winners go to <efficiency>/experiments.json, which the planner's experimenter reads at the start of each drive.
"""
from __future__ import annotations

import json
import random
from pathlib import Path

from openpilot.starpilot.efficiency.experiment import CATEGORIES
from openpilot.starpilot.efficiency.ledger import BATTERY_KWH, GAL_PER_BATTERY_KWH

METERS_PER_MILE = 1609.344
MIN_MILES = 3.0          # per variant, steady situations
MIN_EPISODES = 8         # per variant, launches and speeding up
MIN_GAIN = 0.03          # 3 % less fuel than the baseline
CONFIDENCE = 0.8
RESAMPLES = 400
WINDOW_M = {'launch': 400.0, 'accelerate': 600.0}


def adjusted_gal(stretch: dict) -> float | None:
  gal = stretch.get('obd_gal')
  if not isinstance(gal, (int, float)) or gal < 0:
    return None
  soc_d = stretch.get('soc_d')
  battery = (soc_d / 100.0) * BATTERY_KWH * GAL_PER_BATTERY_KWH if isinstance(soc_d, (int, float)) else 0.0
  return gal - battery   # charge used (soc_d < 0) adds the fuel it stands for


def samples(drives: list[list[dict]]) -> dict[tuple[str, str, str], list[tuple[float, float]]]:
  """(category, context, variant) -> [(gallons, metres)] per stretch (steady) or per episode window (launches)."""
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
          g, m = adjusted_gal(nxt), nxt.get('m') or 0.0
          if g is None:
            gal = None
            break
          gal, metres = gal + g, metres + m
          if metres >= WINDOW_M[category]:
            break
        if gal is not None and metres >= WINDOW_M[category]:
          out.setdefault((category, context, variant), []).append((gal, metres))
      else:
        g, m = adjusted_gal(s), s.get('m') or 0.0
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
  results = judge(samples(load_drives(root)))
  doc = {'version': 1, 'winners': {k: r['winner'] for k, r in results.items() if r['winner']}, 'results': results}
  path = root / 'experiments.json'
  tmp = path.with_suffix('.tmp')
  tmp.write_text(json.dumps(doc, indent=1))
  tmp.replace(path)
  return doc


def report_lines(doc: dict) -> list[str]:
  lines = ['Fuel experiments (BlueDriver, battery-adjusted):']
  for key, r in sorted(doc.get('results', {}).items()):
    scores = ', '.join(f'{v} {x:.2f}' for v, x in r['gal_per_100mi'].items())
    lines.append(f"  {key:22s} {r['status']}  [gal/100mi: {scores}]")
  return lines
