"""The comma's driving against the driver's, in measured fuel, on comparable driving only.

Stretches are grouped by context (speed band, grade, curves, traffic and the car's drive mode, so Sport is
only ever compared with Sport). In every context where both the
driver and the comma have enough measured (BlueDriver) miles, each side's battery-adjusted fuel per mile
is taken, and the comma's rate is applied to the driver's miles there. Summed over those contexts this
answers: "driving the same kind of roads you drove, how much fuel would the comma have used?"
Contexts only one side drove are left out, so highway-versus-city mixes cannot skew the result.

The hybrid battery is settled per side over all of its driving, not context by context (2026-11-25): the battery is
shared, so charge one side builds in one context (a downhill, a cold start) is spent in another, and matching only some
contexts charged the comma for every drain while its recharging stayed outside the match (chill read 11% worse on fuel
it actually burned 2.6% less of). Each side's battery change per mile over everything it drove is priced in fuel on the
matched miles. Cold-engine driving (store.cold_stretches) is left out entirely: the driver always drives the warm-up,
when the engine runs for heat and charges the battery whoever drives.

The settings tile (tank_headline) compares per tank: the driver's numbers keep averaging over every stored drive, while
the comma's count only since the last fill-up, so each tank grades how the comma drives now (its fuel experiments keep
changing it) against the driver's steady baseline. Each finished tank's result is kept in tank_history.json.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path

from openpilot.starpilot.efficiency.ledger import BATTERY_KWH, GAL_PER_BATTERY_KWH, Totals

METERS_PER_MILE = 1609.344
COMMA_MODES = ('chill', 'experimental', 'pulse-glide')
MIN_MILES_EACH = 0.5  # per side, per context
MIN_MATCHED_MILES = 5.0  # before a result is shown
TANK_MATCH_SLACK_MI = 0.5  # walking back over drives to the fill-up: within this of the tank's miles is close enough
TANK_HISTORY = 'tank_history.json'
MAX_TANK_HISTORY = 200
GAL_PER_SOC_POINT = BATTERY_KWH * GAL_PER_BATTERY_KWH / 100.0


@dataclass(frozen=True)
class Comparison:
  matched_miles: float  # driver miles in contexts both sides drove
  contexts: int
  driver_gal: float  # fuel the driver used on those miles (battery-adjusted)
  comma_gal: float  # fuel the comma would have used on the same miles at its own rates
  comma_miles: float = 0.0  # the comma's own miles in those contexts
  driver_fuel_gal: float = 0.0  # the same, measured fuel only (before the battery is priced in)
  comma_fuel_gal: float = 0.0

  @property
  def ready(self) -> bool:
    return self.matched_miles >= MIN_MATCHED_MILES and self.comma_miles >= MIN_MATCHED_MILES and self.driver_gal > 0

  @property
  def comma_saving(self) -> float | None:
    """Fraction of fuel the comma saves versus the driver (negative: it uses more)."""
    return 1.0 - self.comma_gal / self.driver_gal if self.ready else None

  @property
  def fuel_saving(self) -> float | None:
    """The same on measured fuel alone, battery left out."""
    return 1.0 - self.comma_fuel_gal / self.driver_fuel_gal if self.ready and self.driver_fuel_gal > 0 else None


def _gal_per_mile(t: Totals) -> float | None:
  miles = t.obd_distance / METERS_PER_MILE
  return t.obd_fuel_gal / miles if miles > 0 and t.obd_fuel_gal > 0 else None


def compare(by_context: dict[tuple, Totals], modes: tuple[str, ...] = COMMA_MODES) -> Comparison:
  """by_context: (speed, grade, curves, traffic, who, drive mode) -> Totals, as summarize() or the stored ledger gives."""
  sides: dict[tuple, dict[str, Totals]] = {}
  battery = {'driver': Totals(), 'comma': Totals()}   # everything each side drove, for its battery change per mile
  for key, t in by_context.items():
    side = 'driver' if key[4] == 'driver' else 'comma' if key[4] in modes else None
    if side is not None:
      match = tuple(key[:4]) + (key[5] if len(key) > 5 else 'unknown',)  # older drive files have no drive mode
      sides.setdefault(match, {}).setdefault(side, Totals()).add_totals(t)
      battery[side].add_totals(t)
  matched = driver_fuel = comma_fuel = comma_matched = 0.0
  contexts = 0
  for pair in sides.values():
    driver, comma = pair.get('driver'), pair.get('comma')
    if driver is None or comma is None:
      continue
    d_miles, c_miles = driver.obd_distance / METERS_PER_MILE, comma.obd_distance / METERS_PER_MILE
    d_rate, c_rate = _gal_per_mile(driver), _gal_per_mile(comma)
    if d_miles < MIN_MILES_EACH or c_miles < MIN_MILES_EACH or d_rate is None or c_rate is None:
      continue
    matched += d_miles
    comma_matched += c_miles
    driver_fuel += d_rate * d_miles
    comma_fuel += c_rate * d_miles
    contexts += 1

  def charge_gal(side: str) -> float:
    """The side's battery change per mile over all its driving, as fuel on the matched miles (charged: a credit)."""
    t = battery[side]
    miles = t.obd_distance / METERS_PER_MILE
    return t.obd_soc_delta / miles * matched * GAL_PER_SOC_POINT if miles > 0 else 0.0

  return Comparison(matched, contexts, driver_fuel - charge_gal('driver'), comma_fuel - charge_gal('comma'), comma_matched,
                    driver_fuel, comma_fuel)


def stored_comparison(root=None, modes: tuple[str, ...] = COMMA_MODES) -> Comparison:
  from openpilot.starpilot.efficiency.store import KEY_SEPARATOR, load_totals
  _, contexts, _ = load_totals(root, warm_only=True)
  return compare({tuple(k.split(KEY_SEPARATOR)): v for k, v in contexts.items()}, modes)


def headline(c: Comparison) -> str:
  """Short text for the settings tile."""
  if not c.ready:
    return f"collecting: {c.matched_miles:.1f} of {MIN_MATCHED_MILES:g} matched mi"
  saving = c.comma_saving
  return f"comma {'saves' if saving >= 0 else 'uses'} {abs(saving):.0%} {'fuel' if saving >= 0 else 'more fuel'} · {c.matched_miles:.0f} mi"


def tank_drives(drives: list, tank) -> set[str] | None:
  """The drive ids of the current tank, or None when no fill-up has been seen (then every drive counts)."""
  if tank is None or tank.miles_since_fill is None:
    return None
  if tank.fill_drive:
    return {drive for drive, _, _ in drives if drive >= tank.fill_drive}
  # a fill-up recorded before fill_drive existed: walk back from the newest drive over the miles since it
  ids, miles = set(), 0.0
  for drive, distance, _ in reversed(drives):
    if miles >= tank.miles_since_fill - TANK_MATCH_SLACK_MI:
      break
    ids.add(drive)
    miles += distance / METERS_PER_MILE
  return ids


def tank_comparison(root=None, tank=None, modes: tuple[str, ...] = COMMA_MODES) -> Comparison:
  """Every stored drive for the driver, only the current tank's drives for the comma."""
  from openpilot.starpilot.efficiency.ledger import Totals
  from openpilot.starpilot.efficiency.store import KEY_SEPARATOR, load_drives
  drives = load_drives(root, warm_only=True)
  window = tank_drives(drives, tank)
  contexts: dict[tuple, Totals] = {}
  for drive, _, by_key in drives:
    for key, t in by_key.items():
      parts = tuple(key.split(KEY_SEPARATOR))
      who = parts[4] if len(parts) > 4 else ''
      if who == 'driver' or (who in modes and (window is None or drive in window)):
        contexts.setdefault(parts, Totals()).add_totals(t)
  return compare(contexts, modes)


def _history_path(root) -> Path:
  from openpilot.starpilot.efficiency.store import default_root
  return (default_root() if root is None else Path(root)) / TANK_HISTORY


def load_tank_history(root=None) -> list[dict]:
  try:
    doc = json.loads(_history_path(root).read_text())
    return [x for x in doc if isinstance(x, dict)] if isinstance(doc, list) else []
  except (OSError, ValueError):
    return []


def record_finished_tank(root, tank, now: float | None = None) -> dict | None:
  """At a fill-up: keep the tank that just ended (its comma against the driver), if the comma drove on it."""
  c = tank_comparison(root, tank)
  if c.comma_miles <= 0:
    return None
  entry = {'ended': time.strftime('%Y-%m-%d %H:%M', time.localtime(now)), 'fill_drive': tank.fill_drive if tank else None,
           'tank_miles': round(tank.miles_since_fill, 1) if tank and tank.miles_since_fill is not None else None,
           'comma_miles': round(c.comma_miles, 1), 'matched_miles': round(c.matched_miles, 1), 'ready': c.ready,
           'comma_saving': round(c.comma_saving, 4) if c.comma_saving is not None else None}
  history = (load_tank_history(root) + [entry])[-MAX_TANK_HISTORY:]
  from openpilot.starpilot.efficiency.store import _atomic_write
  path = _history_path(root)
  path.parent.mkdir(parents=True, exist_ok=True)
  _atomic_write(path, history)
  return entry


def tank_headline(root=None, tank=None) -> str:
  """Settings tile: this tank's comma against the driver's usual driving, and how the last finished tank ended."""
  c = tank_comparison(root, tank)
  if c.ready:
    saving = c.comma_saving
    now = f"this tank: comma {'saves' if saving >= 0 else 'uses'} {abs(saving):.0%}{'' if saving >= 0 else ' more'} · {c.comma_miles:.0f} mi"
  else:
    now = f"this tank: collecting {min(c.comma_miles, MIN_MATCHED_MILES):.1f} of {MIN_MATCHED_MILES:g} mi"
  last = next((x for x in reversed(load_tank_history(root)) if x.get('ready') and x.get('comma_saving') is not None), None)
  if last is not None:
    s = last['comma_saving']
    now += f"\nlast tank: {'saved' if s >= 0 else 'used'} {abs(s):.0%}{'' if s >= 0 else ' more'}"
  return now
