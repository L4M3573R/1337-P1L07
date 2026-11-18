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


def load_totals(root: Path | None = None) -> tuple[Totals, dict[str, Totals], int]:
  """Sum every stored drive: (overall, per-context, number of drives). Unreadable files are skipped."""
  root = default_root() if root is None else root
  overall, contexts, count = Totals(), {}, 0
  for path in sorted((root / "drives").glob("*.json")):
    try:
      doc = json.loads(path.read_text())
      rescale(doc)
      if doc.get("version") != VERSION:
        continue
      _add(overall, doc["total"])
      for key, values in doc["contexts"].items():
        _add(contexts.setdefault(key, Totals()), values)
      count += 1
    except (OSError, ValueError, KeyError, TypeError):
      continue
  return overall, contexts, count


def load_drives(root: Path | None = None) -> list[tuple[str, float, dict[str, Totals]]]:
  """Every stored drive as (drive id, distance in m, per-context totals), oldest name first. Unreadable files are skipped."""
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
