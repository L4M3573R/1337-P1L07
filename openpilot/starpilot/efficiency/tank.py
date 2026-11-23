"""Real fuel from the dash: miles since the last fill-up divided by the dash's average MPG.

The dash average (and range) resets at every fill-up, so gallons used since the fill-up are
miles / average. The miles are counted here and carried across drives in a small state file. A
fill-up is the fuel gauge (BlueDriver PID 2F, smoothed) rising FILL_LEVEL_JUMP points or more: the hybrid's range
estimate jumps 50+ mi on its own, and range-jump detection logged four fake fills between 11-20 and 11-23 (the real
fills were 11-16 6 -> 96 % and 11-23 6 -> 93 %; the gauge's own wobble stays under ~15 points). Until the first
fill-up is seen (or the miles are seeded), the gallons are unknown rather than guessed.

  python -m openpilot.starpilot.efficiency.tank --seed-miles 31.2   # miles driven since the last fill-up
  python -m openpilot.starpilot.efficiency.tank --pump-gal 8.757    # gallons from the receipt for the fill just seen
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import tempfile
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path

VERSION = 1
FILL_RANGE_JUMP_MI = 50  # a range increase this large between readings (offline log replays only: no gauge there)
FILL_LEVEL_JUMP = 25.0   # gauge points (of 100) the smoothed fuel level must rise to count as a fill-up (~3 gal)
LEVEL_WINDOW = 15        # readings (one a second) at drive start whose median is checked against the last drive's level
LEVEL_HISTORY = 120      # readings in the level kept for the next drive: a 2 min median rides out slosh on hills
MIN_AVG_MPG = 1.0
PUMP_LOG = 'pump_fills.json'


@dataclass
class TankState:
  version: int = VERSION
  miles_since_fill: float | None = None  # None until a fill-up is seen or seeded
  last_range_mi: int | None = None
  fills_seen: int = 0
  fill_drive: str | None = None  # the efficiency drive id the last fill-up was seen in (None: unknown, older state)
  last_level: float | None = None  # smoothed fuel gauge, %, last seen (carried across drives: fills happen car-off)
  _levels: deque = field(default_factory=lambda: deque(maxlen=LEVEL_HISTORY), repr=False, compare=False)
  _checked: bool = field(default=False, repr=False, compare=False)

  def drive(self, miles: float) -> None:
    if self.miles_since_fill is not None and math.isfinite(miles) and miles > 0:
      self.miles_since_fill += miles

  def observe_range(self, range_mi: int) -> bool:
    """Records the dash range; True when it jumped up (a hint only: the hybrid's estimate jumps on its own)."""
    jumped = self.last_range_mi is not None and range_mi - self.last_range_mi > FILL_RANGE_JUMP_MI
    self.last_range_mi = range_mi
    return jumped

  def observe_level(self, level: float) -> bool:
    """One fuel gauge reading, %. True when this drive's first smoothed level is FILL_LEVEL_JUMP above the level the
    last drive ended on (it then starts the new tank). Checked once per drive (one TankState per drive): tanks are
    filled with the car off, and mid-drive slosh can swing even a median 20+ points (11-18: 55 <-> 75 on hills)."""
    if not math.isfinite(level) or not 0.0 <= level <= 100.0:
      return False
    self._levels.append(level)
    if self._checked:
      self.last_level = statistics.median(self._levels)
      return False
    if len(self._levels) < LEVEL_WINDOW:
      return False
    self._checked = True
    smooth = statistics.median(self._levels)
    filled = self.last_level is not None and smooth - self.last_level >= FILL_LEVEL_JUMP
    if filled:
      self.fill()
    self.last_level = smooth
    return filled

  def fill(self) -> None:
    self.miles_since_fill = 0.0
    self.fills_seen += 1

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
    fill_drive = doc.get("fill_drive")
    level = doc.get("last_level")
    return TankState(VERSION, float(miles) if miles is not None else None, int(rng) if rng is not None else None,
                     int(doc.get("fills_seen", 0)), fill_drive if isinstance(fill_drive, str) else None,
                     float(level) if isinstance(level, (int, float)) and 0.0 <= level <= 100.0 else None)
  except (OSError, ValueError, TypeError, AttributeError):
    return TankState()


def save(state: TankState, path: Path | None = None) -> None:
  path = default_path() if path is None else path
  path.parent.mkdir(parents=True, exist_ok=True)
  fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.")
  try:
    with os.fdopen(fd, "w") as f:
      json.dump(state_dict(state), f)
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, path)
  finally:
    if os.path.exists(tmp):
      os.unlink(tmp)


def state_dict(state: TankState) -> dict:
  return {k: v for k, v in asdict(state).items() if not k.startswith("_")}


def load_pump_log(root: Path) -> list[dict]:
  try:
    entries = json.loads((root / PUMP_LOG).read_text())
    return [e for e in entries if isinstance(e, dict)] if isinstance(entries, list) else []
  except (OSError, ValueError):
    return []


def record_pump(root: Path, gallons: float, filled_drive: str, since_drive: str | None, now: float | None = None) -> dict:
  """Keep the receipt gallons for the fill-up seen at `filled_drive`; the tank it ends began at `since_drive`."""
  if not math.isfinite(gallons) or not 0.5 <= gallons <= 20.0:
    raise ValueError("pump gallons must be between 0.5 and 20")
  entry = {"filled_drive": filled_drive, "since_drive": since_drive, "pump_gal": round(float(gallons), 3),
           "at": time.strftime("%Y-%m-%d %H:%M", time.localtime(now))}
  log = [e for e in load_pump_log(root) if e.get("filled_drive") != filled_drive] + [entry]
  path = root / PUMP_LOG
  path.parent.mkdir(parents=True, exist_ok=True)
  tmp = path.with_name(f".{path.name}.tmp")
  tmp.write_text(json.dumps(log, indent=1))
  os.replace(tmp, path)
  return entry


def main() -> None:
  parser = argparse.ArgumentParser(description="Show or seed the fill-up tracker")
  parser.add_argument("--seed-miles", type=float, help="miles driven since the last fill-up")
  parser.add_argument("--pump-gal", type=float, help="gallons from the receipt for the latest fill-up")
  parser.add_argument("--since", help="drive id the ended tank began at (default: the previous pump entry's fill)")
  args = parser.parse_args()
  state = load()
  if args.seed_miles is not None:
    if not math.isfinite(args.seed_miles) or args.seed_miles < 0:
      raise SystemExit("seed miles must be a non-negative number")
    state.miles_since_fill = args.seed_miles
    save(state)
  if args.pump_gal is not None:
    if not state.fill_drive:
      raise SystemExit("no fill-up seen yet to attach the gallons to")
    root = default_path().parent
    since = args.since or next((e["filled_drive"] for e in reversed(load_pump_log(root))
                                if e.get("filled_drive") != state.fill_drive), None)
    print(json.dumps(record_pump(root, args.pump_gal, state.fill_drive, since)))
  print(json.dumps(state_dict(state)))


if __name__ == "__main__":
  main()
