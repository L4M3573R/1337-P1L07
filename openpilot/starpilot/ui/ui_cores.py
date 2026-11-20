"""Which CPU cores the UI's main thread may run on, switchable mid-drive for interleaved tests.

The optional "ui_cores" key in cpu_boost.json (StarPilot storage root, shared with starpilot/power/cpu_boost.py,
whose reader ignores this key) lists the cores, e.g. {"ui_cores": [0, 1, 2, 6]}. It is reread every few seconds.
A missing file, missing key or invalid list means the stock set chosen in selfdrive/ui/ui.py.
"""
from __future__ import annotations

import json
import os
from collections.abc import Callable
from pathlib import Path

CONFIG_NAME = "cpu_boost.json"
READ_PERIOD_S = 5.0


def parse_cores(doc: object, cpu_count: int) -> frozenset[int] | None:
  """A non-empty list of distinct core numbers below cpu_count, or None."""
  if not isinstance(doc, dict):
    return None
  cores = doc.get("ui_cores")
  if not isinstance(cores, list) or not 0 < len(cores) <= cpu_count:
    return None
  if any(type(c) is not int or not 0 <= c < cpu_count for c in cores) or len(set(cores)) != len(cores):
    return None
  return frozenset(cores)


def default_config_path() -> Path:
  from openpilot.starpilot.storage import starpilot_storage_root
  return starpilot_storage_root() / CONFIG_NAME


class UiCoreSwitch:
  def __init__(self, stock: set[int] | frozenset[int], path: Path | None = None,
               log: Callable[..., None] | None = None, cpu_count: int | None = None):
    self.stock = frozenset(stock)
    self._path = path
    self._log = log
    self._cpu_count = cpu_count or os.cpu_count() or 8
    self._cores = self.stock
    self._read_at: float | None = None

  def _read(self) -> frozenset[int]:
    try:
      path = self._path if self._path is not None else default_config_path()
      cores = parse_cores(json.loads(path.read_bytes()[:4096]), self._cpu_count)
    except (OSError, ValueError, RuntimeError):
      cores = None
    return cores if cores is not None else self.stock

  def current(self, now: float) -> frozenset[int]:
    if self._read_at is None or not 0 <= now - self._read_at < READ_PERIOD_S:
      self._read_at = now
      cores = self._read()
      if cores != self._cores:
        self._cores = cores
        if self._log is not None:
          self._log("ui_cores", cores=sorted(cores), stock=cores == self.stock)
    return self._cores
