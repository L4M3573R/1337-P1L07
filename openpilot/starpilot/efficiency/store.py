"""Per-drive energy ledger files under the StarPilot storage root.

Each drive writes one small JSON file (checkpointed during the drive, atomically replaced). Totals
across drives are computed by summing those files, so there is no separate running total to drift.
"""
from __future__ import annotations

import json
import os
import tempfile
from dataclasses import asdict, fields
from pathlib import Path

from openpilot.starpilot.efficiency.ledger import FLOW_LEN, Stretch, Totals, summarize

VERSION = 1
# The wheel-speed factor the car's speed and distance are on now (opendbc hyundai interface, Elantra Hybrid: the
# cluster's scale since 2026-11-18). Each drive file records the factor it was made with; rescale() brings older drives
# (factor 1.0, the bare wheel average) onto today's scale when they are read, so speeds and miles compare across the change.
SPEED_FACTOR = 1.0223
DISTANCE_FIELDS = ('distance', 'fuel_distance', 'obd_distance')
MAX_DRIVES = 500
MAX_STRETCH_RECORDS = 3000  # about 10 hours of driving in one file
KEY_SEPARATOR = "|"
# Cold-engine driving (warming up after a start: the engine runs for heat and charges the battery whoever drives) is
# left out of the comma-versus-driver comparison: the driver always drives it. A stretch is cold when it was driven
# before the coolant first reached ENGINE_WARM_C; drives recorded before the coolant was kept (2026-11-25) drop their
# first COLD_START_S instead (pc_summary fills older drives in from the BlueDriver logs where it can).
ENGINE_WARM_C = 65.0
COLD_START_S = 600.0


def default_root() -> Path:
  from openpilot.starpilot.storage import starpilot_storage_root
  return starpilot_storage_root() / "efficiency"


def _totals_dict(t: Totals) -> dict[str, float]:
  return {f.name: round(getattr(t, f.name), 3) for f in fields(Totals)}


def rescale(doc: dict) -> dict:
  """A stored drive on today's speed scale (in place): distances and stretch speeds times SPEED_FACTOR over the factor
  the drive was recorded with. The energy fields are left as recorded (they were worked out from the old speeds)."""
  if not isinstance(doc, dict):
    return doc
  recorded = doc.get('speed_factor', 1.0)
  if not isinstance(recorded, (int, float)) or isinstance(recorded, bool) or not 0.9 <= recorded <= 1.1:
    recorded = 1.0
  k = SPEED_FACTOR / recorded
  if abs(k - 1.0) < 1e-6:
    return doc
  for totals in [doc.get('total'), *(doc.get('contexts') or {}).values()]:
    if isinstance(totals, dict):
      for name in DISTANCE_FIELDS:
        if isinstance(totals.get(name), (int, float)):
          totals[name] = totals[name] * k
  for st in doc.get('stretches') or []:
    if isinstance(st, dict):
      for name in ('m', 'v0', 'v1'):
        if isinstance(st.get(name), (int, float)):
          st[name] = st[name] * k
  doc['speed_factor'] = SPEED_FACTOR
  return doc


def drive_document(drive_id: str, stretches: list[Stretch], flow: dict | None = None, speed_factor: float = 1.0) -> dict:
  total, by_context = summarize(stretches)
  doc = {"version": VERSION, "drive": drive_id, "speed_factor": round(float(speed_factor), 4), "total": _totals_dict(total),
         "contexts": {KEY_SEPARATOR.join(key): _totals_dict(t) for key, t in sorted(by_context.items())}}
  if flow:
    doc["flow"] = {k: [round(x, 1) for x in v] for k, v in sorted(flow.items())}
  doc["stretches"] = [s.record() for s in stretches[-MAX_STRETCH_RECORDS:]]
  return doc


def _atomic_write(path: Path, data: dict) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.")
  try:
    with os.fdopen(fd, "w") as f:
      json.dump(data, f, separators=(",", ":"))
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, path)
  finally:
    if os.path.exists(tmp):
      os.unlink(tmp)


def save_drive(drive_id: str, stretches: list[Stretch], root: Path | None = None, flow: dict | None = None,
               speed_factor: float = 1.0) -> Path:
  root = default_root() if root is None else root
  if not drive_id or any(ch in drive_id for ch in "/\\") or drive_id.startswith("."):
    raise ValueError("invalid drive id")
  path = root / "drives" / f"{drive_id}.json"
  _atomic_write(path, drive_document(drive_id, stretches, flow, speed_factor))
  prune(root)
  return path


def prune(root: Path, keep: int = MAX_DRIVES) -> None:
  drives = sorted((root / "drives").glob("*.json"), key=lambda p: p.stat().st_mtime)
  for old in drives[:-keep] if keep > 0 else drives:
    old.unlink(missing_ok=True)


def cold_stretches(doc: dict) -> list[dict]:
  """The drive's warm-up: its stretch records before the engine first got warm (by coolant, or by time when the drive
  has no coolant). Only the warm-up counts: the hybrid's engine cools back down while it drives electric (logs
  2026-11-14: 72 C down to 38 C in town), and that is driving, not a cold start. A drive whose engine never gets warm
  (short winter trips peak at 40-55 C) is all warm-up up to its last reading."""
  stretches = sorted((s for s in doc.get('stretches') or [] if isinstance(s, dict) and isinstance(s.get('t'), (int, float))),
                     key=lambda x: x['t'])
  if not stretches:
    return []
  first = stretches[0]['t']
  known = [s for s in stretches if isinstance(s.get('coolant'), (int, float))]
  if not known:
    return [s for s in stretches if s['t'] - first < COLD_START_S]
  warm = [s['t'] for s in known if s['coolant'] >= ENGINE_WARM_C]
  if warm:
    return [s for s in stretches if s['t'] < warm[0]]
  # Never warm: everything up to the last reading; past it (the BlueDriver link dropped) only the first COLD_START_S.
  last = known[-1]['t']
  return [s for s in stretches if s['t'] <= last or s['t'] - first < COLD_START_S]


def _drop_cold(doc: dict, contexts: dict[str, Totals]) -> None:
  """Take the cold-engine stretches back out of the drive's per-context totals (measured fuel, battery and distance)."""
  for s in cold_stretches(doc):
    ctx = s.get('ctx')
    if not isinstance(ctx, list) or len(ctx) != 4:
      continue
    t = contexts.get(KEY_SEPARATOR.join([*map(str, ctx), str(s.get('mode')), str(s.get('drive_mode') or 'unknown')]))
    if t is None:
      continue
    m, dur = float(s.get('m') or 0.0), float(s.get('dur') or 0.0)
    t.distance, t.duration = max(t.distance - m, 0.0), max(t.duration - dur, 0.0)
    t.propulsion = max(t.propulsion - float(s.get('prop_j') or 0.0), 0.0)
    t.regen = max(t.regen - float(s.get('regen_j') or 0.0), 0.0)
    t.friction = max(t.friction - float(s.get('fric_j') or 0.0), 0.0)
    if isinstance(s.get('obd_gal'), (int, float)):
      t.obd_fuel_gal = max(t.obd_fuel_gal - s['obd_gal'], 0.0)
      t.obd_distance = max(t.obd_distance - m, 0.0)
      t.obd_soc_delta -= float(s.get('soc_d') or 0.0)


def load_totals(root: Path | None = None, warm_only: bool = False) -> tuple[Totals, dict[str, Totals], int]:
  """Sum every stored drive: (overall, per-context, number of drives). Unreadable files are skipped. warm_only leaves
  out cold-engine driving from the per-context totals (the comparison)."""
  root = default_root() if root is None else root
  overall, contexts, count = Totals(), {}, 0
  for path in sorted((root / "drives").glob("*.json")):
    try:
      doc = json.loads(path.read_text())
      rescale(doc)
      if doc.get("version") != VERSION:
        continue
      _add(overall, doc["total"])
      drive_contexts: dict[str, Totals] = {}
      for key, values in doc["contexts"].items():
        _add(drive_contexts.setdefault(key, Totals()), values)
      if warm_only:
        _drop_cold(doc, drive_contexts)
      for key, t in drive_contexts.items():
        contexts.setdefault(key, Totals()).add_totals(t)
      count += 1
    except (OSError, ValueError, KeyError, TypeError):
      continue
  return overall, contexts, count


def load_drives(root: Path | None = None, warm_only: bool = False) -> list[tuple[str, float, dict[str, Totals]]]:
  """Every stored drive as (drive id, distance in m, per-context totals), oldest name first. Unreadable files are skipped.
  warm_only leaves out cold-engine driving from the per-context totals (the comparison)."""
  root = default_root() if root is None else root
  drives = []
  for path in sorted((root / "drives").glob("*.json")):
    try:
      doc = json.loads(path.read_text())
      rescale(doc)
      if doc.get("version") != VERSION:
        continue
      contexts = {}
      for key, values in doc["contexts"].items():
        _add(contexts.setdefault(key, Totals()), values)
      if warm_only:
        _drop_cold(doc, contexts)
      drives.append((str(doc.get("drive") or path.stem), float(doc["total"].get("distance", 0.0)), contexts))
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
      continue
  return drives


def _add(target: Totals, values: dict) -> None:
  for name in asdict(target):
    setattr(target, name, getattr(target, name) + float(values.get(name, 0.0)))


def load_flow(root: Path | None = None) -> dict[str, list[float]]:
  """Power-flow histogram merged across stored drives: 'speed|accel|phase' -> ledger.FLOW_LEN numbers (older drives
  stored the first five; the BlueDriver engine counts are zero there)."""
  root = default_root() if root is None else root
  merged: dict[str, list[float]] = {}
  for path in sorted((root / "drives").glob("*.json")):
    try:
      doc = json.loads(path.read_text())
      rescale(doc)
      if doc.get("version") != VERSION:
        continue
      for key, values in doc.get("flow", {}).items():
        entry = merged.setdefault(key, [0.0] * FLOW_LEN)
        for i, value in enumerate(values[:FLOW_LEN]):
          entry[i] += float(value)
    except (OSError, ValueError, KeyError, TypeError, IndexError):
      continue
  return merged
