"""Settings behind the "L4 custom" tile: personal steering (NNLC), curve guard, Pulse and Glide band,
Conditional Experimental mode and lane offset.

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
LANE_OFFSETS_M = (-0.23, -0.12, 0.0)
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


# ---------- lane offset ----------
def lane_offset(params) -> float:
  return _float_param(params, 'LaneCenterOffset', 0.0)


def lane_offset_options(params) -> tuple[float, ...]:
  current = round(lane_offset(params), 2)
  return tuple(sorted(set(LANE_OFFSETS_M) | ({current} if -0.3 <= current <= 0.3 else set())))


def set_lane_offset(params, value: float) -> None:
  if not math.isfinite(value) or not -0.3 <= value <= 0.3:
    raise ValueError('offset outside -0.3..0.3 m')
  params.put('LaneCenterOffset', float(value), block=True)


# ---------- Conditional Experimental ----------
def cem_on(params) -> bool:
  from openpilot.starpilot.conditional_mode.actions import DOCUMENT_KEY, MAX_DOCUMENT_BYTES
  from openpilot.starpilot.saved_source import read_saved
  document, ok = read_saved(params, DOCUMENT_KEY, MAX_DOCUMENT_BYTES)
  if not ok or document is None:
    return False
  try:
    return json.loads(document).get('mode') == 'conditional_experimental'
  except (ValueError, AttributeError):
    return False


def set_cem(params, on: bool, authorized) -> bool:
  """Switch between Conditional Experimental and stock through the same locked commit the other selectors use."""
  from openpilot.starpilot.conditional_mode.actions import DOCUMENT_KEY, MAX_DOCUMENT_BYTES, MODE, commit_mode, edit
  from openpilot.starpilot.conditional_mode.preferences import PreferenceError, SavedPreferences, decode_preferences
  from openpilot.starpilot.saved_source import read_saved
  document, ok1 = read_saved(params, DOCUMENT_KEY, MAX_DOCUMENT_BYTES)
  baseline, ok2 = read_saved(params, 'ExperimentalMode', 8)
  safe, ok3 = read_saved(params, 'SafeMode', 8)
  units, ok4 = read_saved(params, 'IsMetric', 8)
  if not (ok1 and ok2 and ok3 and ok4):
    return False
  try:
    preferences = SavedPreferences() if document is None else decode_preferences(document)
    raw = edit(preferences, MODE, 'Conditional Experimental' if on else 'Stock', metric=units == b'1')
  except PreferenceError:
    return False
  result = commit_mode(params, raw, baseline or b'0', expected=document, expected_baseline=baseline,
                       expected_units=units, expected_safe=safe, authorized=authorized)
  return bool(result.committed)
