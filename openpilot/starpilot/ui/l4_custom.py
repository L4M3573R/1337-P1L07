"""Settings behind the "L4 custom" tile: personal steering (NNLC), curve guard, BlueDriver logging and Pulse and Glide band.
(Conditional Experimental and lane offset keep their own selectors; the curve guard reads the lane offset.)

Each read falls back to the default when its source is missing or malformed; each write is atomic.
Personal steering and the curve guard take effect at the start of the next drive; the rest apply live.
"""
from __future__ import annotations

import json
import math
import os
import tempfile
from pathlib import Path

NNLC_MODES = ('auto', 'off', 'on')  # auto: the off-car trainer's trial decides; off/on: the driver's override
PG_DELTAS_MPH = (3.0, 5.0, 7.0)
PG_DELTAS_KPH = (5.0, 8.0, 11.0)
MAX_JSON_BYTES = 4096


def storage_root() -> Path:
  from openpilot.starpilot.storage import starpilot_storage_root
  return starpilot_storage_root()


def _read_json(path: Path) -> dict | None:
  try:
    raw = path.read_bytes()
    doc = json.loads(raw) if len(raw) <= MAX_JSON_BYTES else None
    return doc if isinstance(doc, dict) else None
  except (OSError, ValueError):
    return None


def _write_json(path: Path, doc: dict) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f'.{path.name}.')
  try:
    with os.fdopen(fd, 'w') as f:
      json.dump(doc, f)
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, path)
  finally:
    if os.path.exists(tmp):
      os.unlink(tmp)


# ---------- personal steering ----------
def nnlc_override(root: Path | None = None) -> str:
  doc = _read_json((storage_root() if root is None else root) / 'nnlc' / 'user.json')
  mode = doc.get('mode') if doc and doc.get('version') == 1 else None
  return mode if mode in NNLC_MODES else 'auto'


def set_nnlc_override(mode: str, root: Path | None = None) -> None:
  if mode not in NNLC_MODES:
    raise ValueError('unknown personal steering mode')
  _write_json((storage_root() if root is None else root) / 'nnlc' / 'user.json', {'version': 1, 'mode': mode})


def nnlc_model_present(root: Path | None = None) -> bool:
  doc = _read_json((storage_root() if root is None else root) / 'nnlc' / 'config.json')
  return bool(doc and isinstance(doc.get('model'), str))


# ---------- curve guard ----------
def curve_guard_enabled(root: Path | None = None) -> bool:
  doc = _read_json((storage_root() if root is None else root) / 'curve_guard.json')
  return bool(doc and doc.get('enabled') is True)


def set_curve_guard(enabled: bool, root: Path | None = None) -> None:
  _write_json((storage_root() if root is None else root) / 'curve_guard.json', {'enabled': bool(enabled)})


# ---------- BlueDriver logging ----------
def bluedriver_enabled(root: Path | None = None) -> bool:
  doc = _read_json((storage_root() if root is None else root) / 'bluedriver' / 'config.json')
  return bool(doc and doc.get('enabled') is True)


def set_bluedriver(enabled: bool, root: Path | None = None) -> None:
  _write_json((storage_root() if root is None else root) / 'bluedriver' / 'config.json', {'enabled': bool(enabled)})


def fuel_comparison(root: Path | None = None) -> str:
  """The comma's fuel against the driver's on matching driving, from the stored ledger (BlueDriver fuel)."""
  from openpilot.starpilot.efficiency.compare import headline, stored_comparison
  try:
    return headline(stored_comparison(None if root is None else root / 'efficiency'))
  except Exception:  # a settings page must never fail on a damaged ledger
    return "unavailable"


# ---------- params helpers ----------
def _float_param(params, key: str, default: float) -> float:
  try:
    value = float(Path(params.get_param_path(key)).read_bytes())
  except (OSError, ValueError):
    return default
  return value if math.isfinite(value) else default


def is_metric(params) -> bool:
  try:
    return Path(params.get_param_path('IsMetric')).read_bytes().strip() == b'1'
  except OSError:
    return False


# ---------- Pulse and Glide band ----------
def pg_delta_options(params) -> tuple[float, ...]:
  return PG_DELTAS_KPH if is_metric(params) else PG_DELTAS_MPH


def pg_delta(params) -> float:
  return _float_param(params, 'PulseGlideSpeedDelta', 5.0)


def set_pg_delta(params, value: float) -> None:
  if not math.isfinite(value) or not 0.5 <= value <= 30.0:
    raise ValueError('band outside 0.5-30')
  params.put('PulseGlideSpeedDelta', float(value), block=True)

