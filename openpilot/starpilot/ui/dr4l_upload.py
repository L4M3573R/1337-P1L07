"""The home screen's DR4L upload line: while the DR4L server pulls this comma's drive logs it writes a small progress
file here (<starpilot storage>/dr4l_pull.json), and the home screen shows "DR4L Uploading: 46%" counting up to 100%.

The file is ignored once it stops changing (a stalled or interrupted pull disappears on its own), and a finished pull
stays at "DR4L Uploading: 100%" for a minute. Age is measured on this comma's monotonic clock from when the file last changed,
so the server's clock never matters. Reads are cached so the render loop never touches the disk every frame.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

STALE_S = 45.0       # a pull that stopped updating this long ago is no longer shown
DONE_SHOW_S = 60.0   # how long "done" stays up
READ_EVERY_S = 2.0
MAX_BYTES = 4096


def status_path() -> Path:
  from openpilot.starpilot.storage import starpilot_storage_root
  return starpilot_storage_root() / 'dr4l_pull.json'


def upload_text(doc: dict | None, age: float) -> str:
  """The line to show for a progress document that last changed `age` seconds ago, or '' when nothing is current."""
  if not isinstance(doc, dict):
    return ''
  try:
    done, total = int(doc['done']), int(doc['total'])
    percent = float(doc.get('percent', 100.0 * done / total if total else 100.0))
  except (KeyError, TypeError, ValueError):
    return ''
  if not math.isfinite(percent) or total <= 0 or not 0 <= done <= total or not math.isfinite(age):
    return ''
  if doc.get('state') == 'done':
    return 'DR4L Uploading: 100%' if age <= DONE_SHOW_S else ''
  if age > STALE_S:
    return ''
  return f'DR4L Uploading: {max(0.0, min(percent, 99.0)):.0f}%'


class UploadStatus:
  """Cached reader for the render loop."""

  def __init__(self, path: Path | None = None, clock=time.monotonic):
    self._path, self._clock = path, clock
    self._read_at = -math.inf
    self._doc: dict | None = None
    self._changed_at = -math.inf
    self._baseline = True  # the first read only sets what "unchanged" looks like: an old file is not a live pull

  def text(self) -> str:
    now = self._clock()
    if now - self._read_at >= READ_EVERY_S:
      self._read_at = now
      try:
        path = self._path if self._path is not None else status_path()
        raw = path.read_bytes()
        doc = json.loads(raw) if len(raw) <= MAX_BYTES else None
      except (OSError, ValueError):
        doc = None
      if self._baseline:
        self._doc, self._baseline = doc, False
      elif doc != self._doc:
        self._doc, self._changed_at = doc, now
    return upload_text(self._doc, now - self._changed_at)
