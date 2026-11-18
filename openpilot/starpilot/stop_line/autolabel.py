#!/usr/bin/env python3
"""Label new stops without a person, only where the evidence is strong; the rest wait in a review list (PC side).

  python -m openpilot.starpilot.stop_line.autolabel <data_dir>

For every stop lines.py found a line at that nobody has reviewed yet (not in lines_review.json), a stop is accepted:
  known     its line is within KNOWN_M of a line already verified at that spot by another drive, same heading;
  evidence  a full stop whose line was seen in MIN_SUPPORT+ frames and where chestnut planned its stop there
            (CHESTNUT_FRAMES+ frames within AGREE_M), or saw a stop sign there and chestnut agreed a little;
  sign      a rolling stop past a stop sign the sign detector saw, line seen in MIN_SUPPORT+ frames.
Queue stops only ever count as known (the car ahead's bumper can look like a line), rolling stops at lights never
(the light turned green). Checked against 197 hand-reviewed stops (2026-11-15): 48 of 49 accepted were right, 55% of
the good ones accepted. Writes <data_dir>/auto_labels.json (every decision and why) and review_queue.json.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

KNOWN_M = 3.0
KNOWN_HEADING_DEG = 30.0
KNOWN_SUPPORT = 10
MIN_SUPPORT = 15
AGREE_M = 6.0
CHESTNUT_FRAMES = 5
SIGN_CHESTNUT_FRAMES = 2
SIGN_RANGE_M = 60.0
CAMERA_BEFORE_LINE_M = 3.15  # used only until lines_review.json has enough checked full stops to measure it


def evidence(d: dict, kind: str, index: int, line_odometer: float, camera_before_line: float) -> dict:
  """What the drive says about one stop: chestnut agreement, stop sign, where the stop point is on the map."""
  s = d['s']
  point = line_odometer - camera_before_line
  m = (point - s > 10) & (point - s < 60) & np.isfinite(d['model_stop'])
  agree = int(np.sum(np.abs(s[m] + d['model_stop'][m] - point) < AGREE_M))
  sign = bool(np.any(d['sign'][(s > point - SIGN_RANGE_M) & (s < point + 5)]))
  bearing, ahead = float(d['bearing'][index]), point - float(s[index])
  lat, lon = float(d['lat'][index]), float(d['lon'][index])
  if np.isfinite(bearing) and np.isfinite(lat):
    lat, lon = (lat + ahead * np.cos(np.radians(bearing)) / 111_000,
                lon + ahead * np.sin(np.radians(bearing)) / (111_000 * np.cos(np.radians(lat))))
  return dict(agree=agree, sign=sign, lat=lat, lon=lon, bearing=bearing)


def near(a: dict, b: dict) -> bool:
  if not all(np.isfinite(x) for x in (a['lat'], a['lon'], b['lat'], b['lon'], a['bearing'], b['bearing'])):
    return False
  dist = np.hypot((a['lat'] - b['lat']) * 111_000, (a['lon'] - b['lon']) * 111_000 * np.cos(np.radians(a['lat'])))
  return bool(dist < KNOWN_M and abs((a['bearing'] - b['bearing'] + 180) % 360 - 180) < KNOWN_HEADING_DEG)


def decide(kind: str, support: int, ev: dict, verified: list[dict], route: str) -> tuple[str | None, str | None]:
  """(rule, tag) when accepted, else (None, None)."""
  for v in verified:
    if support >= KNOWN_SUPPORT and v['route'] != route and near(ev, v):
      return 'known', v['tag']
  tag = 'sign' if ev['sign'] else 'light'
  if kind == 'full' and support >= MIN_SUPPORT and (ev['agree'] >= CHESTNUT_FRAMES or (ev['sign'] and ev['agree'] >= SIGN_CHESTNUT_FRAMES)):
    return 'evidence', tag
  if kind == 'rolling' and support >= MIN_SUPPORT and ev['sign']:
    return 'sign', 'sign'
  return None, None


def main() -> None:
  data_dir = Path(sys.argv[1])
  lines = json.loads((data_dir / 'lines.json').read_text())['stops']
  reviewed = json.loads((data_dir / 'lines_review.json').read_text()) if (data_dir / 'lines_review.json').exists() else []
  first_tags = {(r['route'], r['stop']): r['tag'] for r in json.loads((data_dir / 'review.json').read_text())}
  offsets = [r['line_ahead_m'] for r in reviewed if r['kind'] == 'full' and r['line_ok']]
  camera_before_line = float(np.median(offsets)) if len(offsets) >= 10 else CAMERA_BEFORE_LINE_M
  cache: dict[str, dict] = {}

  def drive(route):
    if route not in cache:
      cache[route] = dict(np.load(data_dir / 'routes' / f'{route}.npz'))
    return cache[route]

  verified = []
  for r in reviewed:
    if not r['line_ok']:
      continue
    tag = r['tag']
    if r['kind'] == 'full':
      stops = json.loads(str(drive(r['route'])['stops']))
      n = next((i for i, x in enumerate(stops) if x['index'] == r['index']), None)
      tag = first_tags.get((r['route'], n))
    if tag in ('light', 'sign'):
      verified.append(dict(evidence(drive(r['route']), r['kind'], r['index'], r['line_odometer'], camera_before_line),
                           route=r['route'], tag=tag))
  done = {(r['route'], r['kind'], r['index']) for r in reviewed}
  decisions, queue = [], []
  for j in lines:
    key = (j['route'], j['kind'], j['index'])
    if j['line'] is None or key in done:
      continue
    ev = evidence(drive(j['route']), j['kind'], j['index'], j['line']['odometer'], camera_before_line)
    rule, tag = decide(j['kind'], j['line']['support'], ev, verified, j['route'])
    row = dict(route=j['route'], kind=j['kind'], index=j['index'], line_odometer=j['line']['odometer'],
               line_ahead_m=j['line']['ahead_of_event_m'], support=j['line']['support'], accepted=rule is not None,
               rule=rule, tag=tag, chestnut_frames=ev['agree'], sign_seen=ev['sign'])
    decisions.append(row)
    if rule is None:
      queue.append(row)
  (data_dir / 'auto_labels.json').write_text(json.dumps(decisions, indent=1))
  (data_dir / 'review_queue.json').write_text(json.dumps(queue, indent=1))
  print(f"{len(decisions)} unreviewed stops with a line: {sum(d['accepted'] for d in decisions)} accepted "
        f"({', '.join(f'{k} {sum(d['rule'] == k for d in decisions)}' for k in ('known', 'evidence', 'sign'))}), "
        f"{len(queue)} left for review")


if __name__ == '__main__':
  main()
