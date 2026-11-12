"""Chooses, once per drive, whether the personal NNLC feedforward steers this drive.

State lives under <starpilot storage>/nnlc/ and is written by the off-car trainer:
  config.json  {"version": 1, "mode": "off" | "trial" | "on", "model": "models/<file>.json", "model_sha256": "<hex>"}
  state.json   written here: drive count and the last choice, so "trial" alternates drive by drive

Anything missing, malformed or mismatched means a standard drive. A choice is made only at startup,
never mid-drive.
"""
from __future__ import annotations

import json
import os
import re
import tempfile
from dataclasses import dataclass
from pathlib import Path

from openpilot.starpilot.lateral.nnlc.model import ModelError, NnlcModel

VERSION = 1
MODES = ('off', 'trial', 'on')
NNLC, STANDARD = 'nnlc', 'standard'
_MODEL_NAME = re.compile(r'models/[A-Za-z0-9_.-]{1,96}\.json\Z')
_SHA = re.compile(r'[0-9a-f]{64}\Z')
MAX_CONFIG_BYTES = 4096


@dataclass(frozen=True)
class DriveChoice:
  mode: str  # NNLC or STANDARD
  reason: str
  model: NnlcModel | None = None
  model_sha256: str = ''
  drive: int = 0


def default_root() -> Path:
  from openpilot.starpilot.storage import starpilot_storage_root
  return starpilot_storage_root() / 'nnlc'


def _read_json(path: Path) -> dict | None:
  try:
    raw = path.read_bytes()
    if len(raw) > MAX_CONFIG_BYTES:
      return None
    doc = json.loads(raw)
    return doc if isinstance(doc, dict) and doc.get('version') == VERSION else None
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


def choose_for_drive(root: Path | None = None) -> DriveChoice:
  root = default_root() if root is None else root
  config = _read_json(root / 'config.json')
  if config is None:
    return DriveChoice(STANDARD, 'not configured')
  mode = config.get('mode')
  if mode not in MODES:
    return DriveChoice(STANDARD, 'bad mode')
  if mode == 'off':
    return DriveChoice(STANDARD, 'off')
  name, sha = config.get('model'), config.get('model_sha256')
  if not isinstance(name, str) or not _MODEL_NAME.fullmatch(name) or not isinstance(sha, str) or not _SHA.fullmatch(sha):
    return DriveChoice(STANDARD, 'bad model reference')
  try:
    model, loaded_sha = NnlcModel.load(root / name)
  except (OSError, ModelError):
    return DriveChoice(STANDARD, 'model unreadable')
  if loaded_sha != sha:
    return DriveChoice(STANDARD, 'model checksum mismatch')

  state = _read_json(root / 'state.json') or {'version': VERSION}
  drive = int(state.get('drives', 0)) + 1 if isinstance(state.get('drives', 0), int) else 1
  if mode == 'on':
    choice = DriveChoice(NNLC, 'on', model, sha, drive)
  else:
    same_model = state.get('model_sha256') == sha
    last = state.get('last_choice') if same_model else STANDARD
    choice = DriveChoice(STANDARD if last == NNLC else NNLC, 'trial', model, sha, drive)
  try:
    _write_json(root / 'state.json', {'version': VERSION, 'drives': drive, 'last_choice': choice.mode, 'model_sha256': sha})
  except OSError:
    # Without a record the next drive could repeat this choice; a standard drive is the safe repeat.
    return DriveChoice(STANDARD, 'state not writable')
  return choice
