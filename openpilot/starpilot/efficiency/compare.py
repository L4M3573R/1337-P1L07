"""The comma's driving against the driver's, in measured fuel, on comparable driving only.

Stretches are grouped by context (speed band, grade, curves, traffic and the car's drive mode, so Sport is
only ever compared with Sport). In every context where both the
driver and the comma have enough measured (BlueDriver) miles, each side's battery-adjusted fuel per mile
is taken, and the comma's rate is applied to the driver's miles there. Summed over those contexts this
answers: "driving the same kind of roads you drove, how much fuel would the comma have used?"
Contexts only one side drove are left out, so highway-versus-city mixes cannot skew the result.
"""
from __future__ import annotations

from dataclasses import dataclass

from openpilot.starpilot.efficiency.ledger import Totals

METERS_PER_MILE = 1609.344
COMMA_MODES = ('chill', 'experimental', 'pulse-glide')
MIN_MILES_EACH = 0.5  # per side, per context
MIN_MATCHED_MILES = 5.0  # before a result is shown


@dataclass(frozen=True)
class Comparison:
  matched_miles: float  # driver miles in contexts both sides drove
  contexts: int
  driver_gal: float  # fuel the driver used on those miles (battery-adjusted)
  comma_gal: float  # fuel the comma would have used on the same miles at its own rates

  @property
  def ready(self) -> bool:
    return self.matched_miles >= MIN_MATCHED_MILES and self.driver_gal > 0

  @property
  def comma_saving(self) -> float | None:
    """Fraction of fuel the comma saves versus the driver (negative: it uses more)."""
    return 1.0 - self.comma_gal / self.driver_gal if self.ready else None


def _adjusted_gal_per_mile(t: Totals) -> float | None:
  miles = t.obd_distance / METERS_PER_MILE
  fuel = t.obd_fuel_adjusted_gal
  return fuel / miles if miles > 0 and fuel > 0 else None


def compare(by_context: dict[tuple, Totals], modes: tuple[str, ...] = COMMA_MODES) -> Comparison:
  """by_context: (speed, grade, curves, traffic, who, drive mode) -> Totals, as summarize() or the stored ledger gives."""
  sides: dict[tuple, dict[str, Totals]] = {}
  for key, t in by_context.items():
    side = 'driver' if key[4] == 'driver' else 'comma' if key[4] in modes else None
    if side is not None:
      match = tuple(key[:4]) + (key[5] if len(key) > 5 else 'unknown',)  # older drive files have no drive mode
      sides.setdefault(match, {}).setdefault(side, Totals()).add_totals(t)
  matched = driver_gal = comma_gal = 0.0
  contexts = 0
  for pair in sides.values():
    driver, comma = pair.get('driver'), pair.get('comma')
    if driver is None or comma is None:
      continue
    d_miles, c_miles = driver.obd_distance / METERS_PER_MILE, comma.obd_distance / METERS_PER_MILE
    d_rate, c_rate = _adjusted_gal_per_mile(driver), _adjusted_gal_per_mile(comma)
    if d_miles < MIN_MILES_EACH or c_miles < MIN_MILES_EACH or d_rate is None or c_rate is None:
      continue
    matched += d_miles
    driver_gal += d_rate * d_miles
    comma_gal += c_rate * d_miles
    contexts += 1
  return Comparison(matched, contexts, driver_gal, comma_gal)


def stored_comparison(root=None, modes: tuple[str, ...] = COMMA_MODES) -> Comparison:
  from openpilot.starpilot.efficiency.store import KEY_SEPARATOR, load_totals
  _, contexts, _ = load_totals(root)
  return compare({tuple(k.split(KEY_SEPARATOR)): v for k, v in contexts.items()}, modes)


def headline(c: Comparison) -> str:
  """Short text for the settings tile."""
  if not c.ready:
    return f"collecting: {c.matched_miles:.1f} of {MIN_MATCHED_MILES:g} matched mi"
  saving = c.comma_saving
  return f"comma {'saves' if saving >= 0 else 'uses'} {abs(saving):.0%} {'fuel' if saving >= 0 else 'more fuel'} · {c.matched_miles:.0f} mi"
