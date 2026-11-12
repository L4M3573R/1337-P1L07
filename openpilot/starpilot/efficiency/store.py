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

from openpilot.starpilot.efficiency.ledger import Stretch, Totals, summarize

VERSION = 1
MAX_DRIVES = 500
KEY_SEPARATOR = "|"


def default_root() -> Path:
  from openpilot.starpilot.storage import starpilot_storage_root
  return starpilot_storage_root() / "efficiency"


def _totals_dict(t: Totals) -> dict[str, float]:
  return {f.name: round(getattr(t, f.name), 3) for f in fields(Totals)}


def drive_document(drive_id: str, stretches: list[Stretch], flow: dict | None = None) -> dict:
  total, by_context = summarize(stretches)
  doc = {"version": VERSION, "drive": drive_id, "total": _totals_dict(total),
         "contexts": {KEY_SEPARATOR.join(key): _totals_dict(t) for key, t in sorted(by_context.items())}}
  if flow:
    doc["flow"] = {k: [round(x, 1) for x in v] for k, v in sorted(flow.items())}
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


def save_drive(drive_id: str, stretches: list[Stretch], root: Path | None = None, flow: dict | None = None) -> Path:
  root = default_root() if root is None else root
  if not drive_id or any(ch in drive_id for ch in "/\\") or drive_id.startswith("."):
    raise ValueError("invalid drive id")
  path = root / "drives" / f"{drive_id}.json"
  _atomic_write(path, drive_document(drive_id, stretches, flow))
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
      if doc.get("version") != VERSION:
        continue
      _add(overall, doc["total"])
      for key, values in doc["contexts"].items():
        _add(contexts.setdefault(key, Totals()), values)
      count += 1
    except (OSError, ValueError, KeyError, TypeError):
      continue
  return overall, contexts, count


def _add(target: Totals, values: dict) -> None:
  for name in asdict(target):
    setattr(target, name, getattr(target, name) + float(values.get(name, 0.0)))


def load_flow(root: Path | None = None) -> dict[str, list[float]]:
  """Power-flow histogram merged across stored drives: 'speed|accel|phase' -> [samples, ev, regen, engine, meter sum]."""
  root = default_root() if root is None else root
  merged: dict[str, list[float]] = {}
  for path in sorted((root / "drives").glob("*.json")):
    try:
      doc = json.loads(path.read_text())
      if doc.get("version") != VERSION:
        continue
      for key, values in doc.get("flow", {}).items():
        entry = merged.setdefault(key, [0.0] * 5)
        for i in range(5):
          entry[i] += float(values[i])
    except (OSError, ValueError, KeyError, TypeError, IndexError):
      continue
  return merged
