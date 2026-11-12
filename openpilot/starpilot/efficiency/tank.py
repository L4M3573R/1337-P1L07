"""Real fuel from the dash: miles since the last fill-up divided by the dash's average MPG.

The dash average (and range) resets at every fill-up, so gallons used since the fill-up are
miles / average. The miles are counted here and carried across drives in a small state file; a
fill-up is detected when the range jumps up. Until the first fill-up is seen (or the miles are
seeded), the gallons are unknown rather than guessed.

  python -m openpilot.starpilot.efficiency.tank --seed-miles 31.2   # miles driven since the last fill-up
"""
from __future__ import annotations

import argparse
import json
import math
import os
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path

VERSION = 1
FILL_RANGE_JUMP_MI = 50  # a range increase this large between readings means the tank was filled
MIN_AVG_MPG = 1.0


@dataclass
class TankState:
  version: int = VERSION
  miles_since_fill: float | None = None  # None until a fill-up is seen or seeded
  last_range_mi: int | None = None
  fills_seen: int = 0

  def drive(self, miles: float) -> None:
    if self.miles_since_fill is not None and math.isfinite(miles) and miles > 0:
      self.miles_since_fill += miles

  def observe_range(self, range_mi: int) -> bool:
    """Returns True when this reading reveals a fill-up."""
    filled = self.last_range_mi is not None and range_mi - self.last_range_mi > FILL_RANGE_JUMP_MI
    if filled:
      self.miles_since_fill = 0.0
      self.fills_seen += 1
    self.last_range_mi = range_mi
    return filled

  def gallons_used(self, avg_mpg: float) -> float | None:
    """Gallons since the fill-up implied by the dash average, or None when unknown."""
    if self.miles_since_fill is None or not math.isfinite(avg_mpg) or avg_mpg < MIN_AVG_MPG:
      return None
    return self.miles_since_fill / avg_mpg


def default_path() -> Path:
  from openpilot.starpilot.efficiency.store import default_root
  return default_root() / "tank.json"


def load(path: Path | None = None) -> TankState:
  path = default_path() if path is None else path
  try:
    doc = json.loads(path.read_text())
    if doc.get("version") != VERSION:
      return TankState()
    miles = doc.get("miles_since_fill")
    rng = doc.get("last_range_mi")
    return TankState(VERSION, float(miles) if miles is not None else None, int(rng) if rng is not None else None,
                     int(doc.get("fills_seen", 0)))
  except (OSError, ValueError, TypeError, AttributeError):
    return TankState()


def save(state: TankState, path: Path | None = None) -> None:
  path = default_path() if path is None else path
  path.parent.mkdir(parents=True, exist_ok=True)
  fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.")
  try:
    with os.fdopen(fd, "w") as f:
      json.dump(asdict(state), f)
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, path)
  finally:
    if os.path.exists(tmp):
      os.unlink(tmp)


def main() -> None:
  parser = argparse.ArgumentParser(description="Show or seed the fill-up tracker")
  parser.add_argument("--seed-miles", type=float, help="miles driven since the last fill-up")
  args = parser.parse_args()
  state = load()
  if args.seed_miles is not None:
    if not math.isfinite(args.seed_miles) or args.seed_miles < 0:
      raise SystemExit("seed miles must be a non-negative number")
    state.miles_since_fill = args.seed_miles
    save(state)
  print(json.dumps(asdict(state)))


if __name__ == "__main__":
  main()
