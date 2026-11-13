"""Fuel economy over time, per drive: the driver's and the comma's measured, battery-adjusted MPG.

Each stored drive file already holds per-context totals tagged with who drove; this sums them per side.
A side gets a point for a drive only with at least MIN_MILES of BlueDriver-covered driving in it.
Pure data; the PC summary (pc_summary.py) charts it.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from openpilot.starpilot.efficiency.compare import COMMA_MODES, METERS_PER_MILE
from openpilot.starpilot.efficiency.ledger import BATTERY_KWH, GAL_PER_BATTERY_KWH

MIN_MILES = 1.0
ROLLING_DRIVES = 5
MAX_POINTS = 60


@dataclass(frozen=True)
class DrivePoint:
  t: float  # when the drive was last saved (unix time)
  you_mpg: float | None
  comma_mpg: float | None
  you_mi: float
  comma_mi: float


def _side_mpg(fuel: float, soc: float, meters: float) -> float | None:
  miles = meters / METERS_PER_MILE
  adjusted = fuel - soc / 100.0 * BATTERY_KWH * GAL_PER_BATTERY_KWH
  return miles / adjusted if miles >= MIN_MILES and adjusted > 0 else None


def drive_point(t: float, doc: dict) -> DrivePoint | None:
  sums = {'driver': [0.0, 0.0, 0.0], 'comma': [0.0, 0.0, 0.0]}  # fuel gal, soc pts, metres
  for key, values in doc.get('contexts', {}).items():
    mode = key.split('|')[-1]
    side = 'driver' if mode == 'driver' else 'comma' if mode in COMMA_MODES else None
    if side is None:
      continue
    s = sums[side]
    s[0] += float(values.get('obd_fuel_gal', 0.0))
    s[1] += float(values.get('obd_soc_delta', 0.0))
    s[2] += float(values.get('obd_distance', 0.0))
  you, comma = _side_mpg(*sums['driver']), _side_mpg(*sums['comma'])
  if you is None and comma is None:
    return None
  return DrivePoint(t, you, comma, sums['driver'][2] / METERS_PER_MILE, sums['comma'][2] / METERS_PER_MILE)


def load_points(root: Path | None = None) -> list[DrivePoint]:
  from openpilot.starpilot.efficiency.store import VERSION, default_root
  root = default_root() if root is None else root
  points = []
  for path in (root / 'drives').glob('*.json'):
    try:
      doc = json.loads(path.read_text())
      if doc.get('version') == VERSION and (p := drive_point(path.stat().st_mtime, doc)) is not None:
        points.append(p)
    except (OSError, ValueError, TypeError, AttributeError):
      continue
  return sorted(points, key=lambda p: p.t)[-MAX_POINTS:]


def rolling(values: list[float | None], n: int = ROLLING_DRIVES) -> list[float | None]:
  """Mean of the last n known values at each point (None until the first known value)."""
  out, window = [], []
  for v in values:
    if v is not None:
      window = (window + [v])[-n:]
    out.append(sum(window) / len(window) if window else None)
  return out

