"""Engine temperature for drives recorded before the ledger kept it (added 2026-11-25), taken from the BlueDriver logs.

Without it a drive's cold-engine driving is guessed as its first COLD_START_S. bluedriverd writes one row a second
(drives/<id>.csv, newest MAX_DRIVE_LOGS kept) stamped with the same monotonic clock as the ledger's stretch times, so a
stretch gets the coldest coolant logged while it was driven. A log belongs to a drive when the two names (wall-clock
stamps) are within MATCH_S and their times overlap; when several drives fit (the comma used to restart for every
drive, so each one's clock starts near zero) the nearest start wins, and a near tie (the clock was not set yet, so many
drives share one stamp) is skipped. Runs on the PC copy (pc_summary), never on the comma.
"""
from __future__ import annotations

import csv
import json
import math
import time
from pathlib import Path

MATCH_S = 1800.0      # a log and a drive started within this of each other (wall-clock names)
EDGE_S = 2.0          # a stretch also takes a reading this close to its ends
TIE_S = 60.0          # a log goes to the drive that started nearest it, unless another started about as near


def stamp(name: str) -> float | None:
  try:
    return time.mktime(time.strptime(name[:15], '%Y%m%d-%H%M%S'))
  except (ValueError, OverflowError):
    return None


def read_log(path: Path) -> list[tuple[float, float]]:
  """(t, coolant C) rows, sorted by t; [] when unreadable."""
  rows = []
  try:
    with open(path, newline='') as f:
      for row in csv.DictReader(f):
        try:
          t, c = float(row.get('t') or 'nan'), float(row.get('coolant') or 'nan')
        except ValueError:
          continue
        if math.isfinite(t) and math.isfinite(c) and -40.0 <= c <= 150.0:
          rows.append((t, c))
  except OSError:
    return []
  return sorted(rows)


def _span(doc: dict) -> tuple[float, float] | None:
  ts = [(s['t'], s['t'] + float(s.get('dur') or 0.0)) for s in doc.get('stretches') or []
        if isinstance(s, dict) and isinstance(s.get('t'), (int, float))]
  return (min(a for a, _ in ts), max(b for _, b in ts)) if ts else None


def fill(doc: dict, rows: list[tuple[float, float]]) -> int:
  """Give each stretch without coolant the coldest reading logged while it was driven; returns how many got one."""
  n = 0
  for s in doc.get('stretches') or []:
    if not isinstance(s, dict) or not isinstance(s.get('t'), (int, float)) or isinstance(s.get('coolant'), (int, float)):
      continue
    lo, hi = s['t'] - EDGE_S, s['t'] + float(s.get('dur') or 0.0) + EDGE_S
    seen = [c for t, c in rows if lo <= t <= hi]
    if seen:
      s['coolant'] = round(min(seen))
      n += 1
  return n


def backfill(drives_dir: Path, logs_dir: Path) -> int:
  """Fill in coolant on the stored drives that have none; returns how many drive files changed."""
  docs = {}
  for path in sorted(drives_dir.glob('*.json')):
    try:
      doc = json.loads(path.read_text())
    except (OSError, ValueError):
      continue
    stretches = [s for s in doc.get('stretches') or [] if isinstance(s, dict)]
    span, start = _span(doc), stamp(path.stem)
    if span and start is not None and not any(isinstance(s.get('coolant'), (int, float)) for s in stretches):
      docs[path] = (doc, span, start)
  if not docs:
    return 0
  owner: dict[Path, list[Path]] = {}
  for log in sorted(logs_dir.glob('*.csv')) if logs_dir.is_dir() else []:
    start = stamp(log.stem)
    if start is None:
      continue
    near = [p for p, (_, _, s) in docs.items() if abs(s - start) <= MATCH_S]
    if not near:
      continue
    rows = read_log(log)
    if not rows:
      continue
    fits = sorted((abs(docs[p][2] - start), p) for p in near if rows[0][0] <= docs[p][1][1] and docs[p][1][0] <= rows[-1][0])
    if fits and (len(fits) == 1 or fits[1][0] - fits[0][0] >= TIE_S):
      owner.setdefault(fits[0][1], []).append(log)
  changed = 0
  for path, logs in owner.items():
    doc = docs[path][0]
    rows = sorted(r for log in logs for r in read_log(log))
    if fill(doc, rows):
      doc['coolant_from'] = 'bluedriver log'
      tmp = path.with_suffix('.tmp')
      tmp.write_text(json.dumps(doc))
      tmp.replace(path)
      changed += 1
  return changed
