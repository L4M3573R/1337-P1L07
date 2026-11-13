"""Trip reports from the per-stretch records in the stored drive files (PC side; pure functions).

Every number uses measured BlueDriver fuel with the hybrid battery change paid back (battery-adjusted), and only
stretches the BlueDriver fully covered. Breakdowns: who drove, drive mode, tire pressure (compared within the same
outside-temperature band, since cold lowers both pressure and MPG), outside temperature and the car's map speed
limit as the road type. Fill-ups come from the tank level (BlueDriver PID 012F).
"""
from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass

from openpilot.starpilot.efficiency.compare import COMMA_MODES, METERS_PER_MILE
from openpilot.starpilot.efficiency.ledger import BATTERY_KWH, GAL_PER_BATTERY_KWH

TANK_GAL = 11.9  # Elantra Hybrid; pass the real size if it differs
FILL_JUMP_PCT = 12.0  # a tank-level rise this large is a fill-up, not sloshing
MIN_BUCKET_MILES = 1.0


@dataclass
class Tally:
  meters: float = 0.0
  gal: float = 0.0  # battery-adjusted measured fuel
  seconds: float = 0.0
  ev_s: float = 0.0

  def add(self, rec: dict) -> None:
    gal = rec.get('obd_gal')
    if gal is None:
      return
    self.meters += rec.get('m') or 0.0
    self.gal += gal - (rec.get('soc_d') or 0.0) / 100.0 * BATTERY_KWH * GAL_PER_BATTERY_KWH
    self.seconds += rec.get('dur') or 0.0
    self.ev_s += (rec.get('ev') or 0.0) * (rec.get('dur') or 0.0)

  @property
  def miles(self) -> float:
    return self.meters / METERS_PER_MILE

  @property
  def mpg(self) -> float | None:
    return self.miles / self.gal if self.miles >= MIN_BUCKET_MILES and self.gal > 0 else None

  def row(self) -> dict:
    return {'miles': round(self.miles, 2), 'mpg': None if self.mpg is None else round(self.mpg, 1),
            'ev_share': round(self.ev_s / self.seconds, 3) if self.seconds > 0 else None}


def side(rec: dict) -> str | None:
  return 'you' if rec.get('mode') == 'driver' else 'comma' if rec.get('mode') in COMMA_MODES else None


def f_band(celsius: float | None, width: float = 10.0) -> str | None:
  if celsius is None or not math.isfinite(celsius):
    return None
  lo = math.floor((celsius * 9 / 5 + 32) / width) * width
  return f'{lo:.0f}-{lo + width:.0f}F'


def road_type(limit: int | None) -> str:
  if limit is None:
    return 'unknown'
  return 'city (<=30)' if limit <= 30 else 'arterial (35-45)' if limit <= 45 else 'divided (50-55)' if limit <= 55 else 'highway (60+)'


def drive_summary(t: float, records: list[dict], gas_price: float | None = None) -> dict:
  total, by_side, modes = Tally(), defaultdict(Tally), defaultdict(float)
  meters = seconds = 0.0
  tires, temps = [], []
  for r in records:
    meters += r.get('m') or 0.0
    seconds += r.get('dur') or 0.0
    total.add(r)
    if (s := side(r)) is not None:
      by_side[s].add(r)
    if r.get('drive_mode'):
      modes[r['drive_mode']] += (r.get('m') or 0.0) / METERS_PER_MILE
    if r.get('tire') is not None:
      tires.append(r['tire'])
    if r.get('ambient_c') is not None:
      temps.append(r['ambient_c'])
  levels = [r[k] for r in records for k in ('level0', 'level1') if r.get(k) is not None]
  out = {'t': t, 'miles': round(meters / METERS_PER_MILE, 2), 'minutes': round(seconds / 60, 1), **total.row(),
         'you': by_side['you'].row(), 'comma': by_side['comma'].row(), 'drive_mode_miles': {k: round(v, 2) for k, v in modes.items()},
         'tire_psi': round(sum(tires) / len(tires), 1) if tires else None,
         'ambient_f': round(sum(temps) / len(temps) * 9 / 5 + 32) if temps else None,
         'fuel_level': [levels[0], levels[-1]] if levels else None}
  if gas_price is not None and total.gal > 0:
    out['cost'] = round(total.gal * gas_price, 2)
  return out


def breakdowns(records: list[dict]) -> dict:
  """MPG by who drove, drive mode, temperature band, road type, and tire pressure within each temperature band."""
  tables: dict[str, dict] = defaultdict(lambda: defaultdict(Tally))
  for r in records:
    s = side(r)
    temp = f_band(r.get('ambient_c'))
    road = road_type(r.get('limit'))
    if s:
      tables['who'][s].add(r)
      tables['who_by_road'][f'{road} | {s}'].add(r)
    if r.get('drive_mode'):
      tables['drive_mode'][r['drive_mode']].add(r)
    if temp:
      tables['outside_temp'][temp].add(r)
    tables['road_type'][road].add(r)
    if r.get('tire') is not None and temp:
      tables['tire_psi_by_temp'][f"{temp} | {round(r['tire'])} psi"].add(r)
  return {name: {k: v.row() for k, v in sorted(t.items()) if v.mpg is not None} for name, t in tables.items()}


def fill_ups(drives: list[tuple[float, list[dict]]], tank_gal: float = TANK_GAL) -> list[dict]:
  """Tank-level rises between consecutive readings (within or across drives)."""
  events, last = [], None
  for t, records in drives:
    for r in records:
      for key in ('level0', 'level1'):
        level = r.get(key)
        if level is None:
          continue
        if last is not None and level - last[1] >= FILL_JUMP_PCT:
          events.append({'t': t, 'from_pct': round(last[1], 1), 'to_pct': round(level, 1),
                         'gallons_added': round((level - last[1]) / 100 * tank_gal, 2)})
        last = (t, level)
  return events


def tank_check(drives: list[tuple[float, list[dict]]], tank_gal: float = TANK_GAL) -> dict | None:
  """Since the last fill-up: fuel by tank-level drop versus the BlueDriver's measured fuel (not battery-adjusted)."""
  readings, measured = [], 0.0
  for _, records in drives:
    for r in records:
      for key in ('level0', 'level1'):
        if r.get(key) is not None:
          if readings and r[key] - readings[-1] >= FILL_JUMP_PCT:
            readings, measured = [], 0.0  # a fill-up restarts the check
          readings.append(r[key])
      measured += r.get('obd_gal') or 0.0
  if len(readings) < 2 or readings[0] - readings[-1] < 5.0:
    return None
  by_level = (readings[0] - readings[-1]) / 100 * tank_gal
  return {'tank_gal': round(by_level, 3), 'measured_gal': round(measured, 3),
          'measured_vs_tank': round(measured / by_level, 3) if by_level > 0 else None}
