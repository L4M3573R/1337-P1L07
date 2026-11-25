"""The big fuel levers the comma cannot pull: what the driver's own choices cost in fuel, from the stored drives.

The fuel experiments only shape how the comma drives inside the driver's limits. Four things outside them move the
fuel far more, so the PC summary reports them with an estimate of what each costs:

- Speed: the miles driven over SPEED_CAP_MPH against the car's own measured MPG by speed (steady, level, warm-engine
  driving), and what driving those miles at the best measured speed up to SPEED_CAP_MPH (at most SPEED_LOOK_MPH under)
  would have saved, with the minutes it would have cost.
- Heat: MPG and electric share with the cabin heat on against heat off or mild, at the same outside temperature.
- Tire pressure: MPG by the lowest tire's pressure at the same outside temperature, and a flag when the tires ran under
  PLACARD_PSI (the 2024 Elantra Hybrid's cold door placard, 33 psi; not read from the car, so check the door jamb).
- Cold starts: the share of the fuel burned on a cold engine (store.cold_stretches) and the MPG while warming up.

Fuel is always the BlueDriver's measured fuel with the hybrid battery change paid back, as everywhere in the fuel
reports. Every comparison needs MIN_MILES a side (MIN_SPEED_MILES around a speed) or it says it is still collecting.
Read only: nothing here changes the car or the stored files.
"""
from __future__ import annotations

import json
import math
from datetime import date, datetime, timedelta
from pathlib import Path

from openpilot.starpilot.efficiency.ledger import BATTERY_KWH, GAL_PER_BATTERY_KWH

METERS_PER_MILE = 1609.344
MPS_TO_MPH = 2.23694
WINDOW_DAYS = 7
MIN_WINDOW_MILES = 20.0       # fewer miles than this in the window: all stored drives instead
MIN_MILES = 5.0               # per side of a heat / tire / cold-start comparison
MIN_SPEED_MILES = 5.0         # steady level miles within +-1 mph of a speed before its MPG counts...
MIN_OWN_SPEED_MILES = 1.0     # ...with at least this much at the speed itself
SPEED_CAP_MPH = 70
SPEED_LOOK_MPH = 5            # the slower speed is looked for at most this far under the speed driven
STEADY_DV = 2.0               # m/s over one stretch: more is speeding up or slowing, not holding a speed
LEVEL_PITCH = 0.02            # |pitch| (rad, zeroed) under this counts as level for the MPG by speed
PLACARD_PSI = 33.0            # 2024 Elantra Hybrid door placard, cold (Blue 33/33, Limited 33/32 by tire retailer listings); not read from the car
TEMP_BAND_C = 5.0             # outside-temperature bands the heat and tire comparisons stay inside
HEAT_ON = ('heat',)
HEAT_OFF = ('off', 'mild', 'cool')


def adjusted_gal(s: dict) -> float | None:
  """Measured fuel with the battery change paid back, or None when the BlueDriver did not cover the stretch."""
  gal = s.get('obd_gal')
  if not isinstance(gal, (int, float)) or gal < 0:
    return None
  soc_d = s.get('soc_d') if isinstance(s.get('soc_d'), (int, float)) else 0.0
  return gal - soc_d / 100.0 * BATTERY_KWH * GAL_PER_BATTERY_KWH


def _miles(s: dict) -> float:
  m = s.get('m')
  return float(m) / METERS_PER_MILE if isinstance(m, (int, float)) and m > 0 else 0.0


def _mph(s: dict) -> float | None:
  m, dur = s.get('m'), s.get('dur')
  if not isinstance(m, (int, float)) or not isinstance(dur, (int, float)) or dur <= 0:
    return None
  return m / dur * MPS_TO_MPH


class _Sum:
  """Miles, adjusted gallons and electric seconds over the stretches that had measured fuel."""
  def __init__(self):
    self.miles = self.gal = self.ev_s = self.flow_s = 0.0

  def add(self, s: dict) -> None:
    gal = adjusted_gal(s)
    if gal is None:
      return
    self.miles += _miles(s)
    self.gal += gal
    if isinstance(s.get('ev'), (int, float)) and isinstance(s.get('dur'), (int, float)):
      self.ev_s += s['ev'] * s['dur']
      self.flow_s += s['dur']

  @property
  def mpg(self) -> float | None:
    return self.miles / self.gal if self.gal > 0 and self.miles > 0 else None

  @property
  def ev_share(self) -> float | None:
    return self.ev_s / self.flow_s if self.flow_s > 0 else None


def _drive_date(doc: dict) -> date | None:
  try:
    return datetime.strptime(str(doc.get('drive', ''))[:8], '%Y%m%d').date()
  except ValueError:
    return None


def load_docs(root: Path) -> list[dict]:
  """Every stored drive document on today's speed scale (store.rescale), oldest first."""
  from openpilot.starpilot.efficiency.store import VERSION, rescale
  docs = []
  for path in sorted((root / 'drives').glob('*.json')):
    try:
      doc = rescale(json.loads(path.read_text()))
    except (OSError, ValueError, TypeError):
      continue
    if isinstance(doc, dict) and doc.get('version') == VERSION and isinstance(doc.get('stretches'), list):
      docs.append(doc)
  return docs


def window(docs: list[dict], today: date | None = None, days: int = WINDOW_DAYS) -> tuple[list[dict], str]:
  """The drives of the last `days` days, or every drive when those are too few miles to say anything."""
  today = today or date.today()
  recent = [d for d in docs if (dd := _drive_date(d)) is not None and dd > today - timedelta(days=days)]
  miles = sum(_miles(s) for d in recent for s in d['stretches'] if isinstance(s, dict))
  if miles >= MIN_WINDOW_MILES:
    return recent, f'last {days} days'
  return docs, 'all stored drives'


def _stretches(docs: list[dict]) -> list[dict]:
  return [s for d in docs for s in d['stretches'] if isinstance(s, dict)]


def _cold_ids(docs: list[dict]) -> set[int]:
  from openpilot.starpilot.efficiency.store import cold_stretches
  return {id(s) for d in docs for s in cold_stretches(d)}


def _pitch_zero(stretches: list[dict]) -> float:
  """The pitch signal's long-run mean (it reads nose-up all the time; experiment_learn zeroes it the same way)."""
  num = den = 0.0
  for s in stretches:
    p, m = s.get('pitch'), _miles(s)
    if isinstance(p, (int, float)) and math.isfinite(p) and m > 0:
      num, den = num + p * m, den + m
  return num / den if den > 0 else 0.0


# ---------- speed ----------
def mpg_by_speed(stretches: list[dict], cold: set[int]) -> dict[int, _Sum]:
  """Whole mph -> steady, level, warm-engine fuel sums."""
  zero = _pitch_zero(stretches)
  out: dict[int, _Sum] = {}
  for s in stretches:
    mph, p, v0, v1 = _mph(s), s.get('pitch'), s.get('v0'), s.get('v1')
    if id(s) in cold or mph is None or not isinstance(p, (int, float)) or abs(p - zero) > LEVEL_PITCH:
      continue
    if not isinstance(v0, (int, float)) or not isinstance(v1, (int, float)) or abs(v1 - v0) > STEADY_DV:
      continue
    out.setdefault(int(round(mph)), _Sum()).add(s)
  return out


def speed_mpg(cells: dict[int, _Sum], mph: int) -> float | None:
  """MPG at a speed, pooled with its two neighbours, once there is enough driving around it."""
  own = cells.get(mph)
  near = [cells[k] for k in (mph - 1, mph, mph + 1) if k in cells]
  miles, gal = sum(c.miles for c in near), sum(c.gal for c in near)
  if own is None or own.miles < MIN_OWN_SPEED_MILES or miles < MIN_SPEED_MILES or gal <= 0:
    return None
  return miles / gal


def speed_lever(stretches: list[dict], cold: set[int]) -> dict:
  cells = mpg_by_speed(stretches, cold)
  total_miles = sum(_miles(s) for s in stretches)
  fast: dict[int, float] = {}
  for s in stretches:
    mph = _mph(s)
    if mph is not None and mph > SPEED_CAP_MPH + 0.5:
      fast[int(round(mph))] = fast.get(int(round(mph)), 0.0) + _miles(s)
  fast_miles = sum(fast.values())
  saved_gal = minutes = judged_miles = 0.0
  for mph, miles in sorted(fast.items()):
    here = speed_mpg(cells, mph)
    options = [(speed_mpg(cells, t), t) for t in range(mph - SPEED_LOOK_MPH, min(mph - 1, SPEED_CAP_MPH) + 1)]
    options = [(m, t) for m, t in options if m is not None]
    if here is None or not options:
      continue
    best_mpg, best = max(options)
    if best_mpg <= here:
      continue
    judged_miles += miles
    saved_gal += miles * (1.0 / here - 1.0 / best_mpg)
    minutes += miles * (60.0 / best - 60.0 / mph)
  out = {'over_mph': SPEED_CAP_MPH, 'miles': round(total_miles, 1), 'fast_miles': round(fast_miles, 1),
         'fast_share': round(fast_miles / total_miles, 3) if total_miles > 0 else None,
         'curve': {str(k): round(m, 1) for k in sorted(cells) if (m := speed_mpg(cells, k)) is not None},
         'judged_miles': round(judged_miles, 1), 'saved_gal': round(saved_gal, 4), 'minutes': round(minutes, 1),
         'saved_gal_per_100mi': round(100.0 * saved_gal / total_miles, 4) if total_miles > 0 else None}
  if fast_miles < 0.5:
    out['status'], out['line'] = 'none', f'Speed: almost nothing over {SPEED_CAP_MPH} mph ({fast_miles:.1f} mi)'
  elif judged_miles < 0.5:
    out['status'] = 'collecting'
    out['line'] = (f'Speed: {fast_miles:.1f} mi over {SPEED_CAP_MPH} mph ({100 * out["fast_share"]:.0f}% of the miles); ' +
                   'not enough steady level driving at those speeds yet to price it')
  else:
    out['status'] = 'ok'
    out['line'] = (f'Speed: {fast_miles:.1f} mi over {SPEED_CAP_MPH} mph ({100 * out["fast_share"]:.0f}% of the miles); ' +
                   f'at the best measured speed up to {SPEED_CAP_MPH} they would have used {saved_gal:.2f} gal less ' +
                   f'({out["saved_gal_per_100mi"]:.2f} gal per 100 mi of all driving) for {minutes:.1f} min more')
  return out


# ---------- heat and tires: compared inside one outside-temperature band ----------
def _band(s: dict) -> int | None:
  a = s.get('ambient_c')
  return int(math.floor(a / TEMP_BAND_C)) if isinstance(a, (int, float)) and math.isfinite(a) else None


def _band_name(b: int) -> str:
  lo = b * TEMP_BAND_C
  return f'{lo * 9 / 5 + 32:.0f}-{(lo + TEMP_BAND_C) * 9 / 5 + 32:.0f} F'


def heat_lever(stretches: list[dict], cold: set[int]) -> dict:
  bands: dict[int, dict[str, _Sum]] = {}
  for s in stretches:
    b, climate = _band(s), s.get('climate')
    if b is None or id(s) in cold:
      continue
    side = 'heat' if climate in HEAT_ON else 'off' if climate in HEAT_OFF else None
    if side is not None:
      bands.setdefault(b, {'heat': _Sum(), 'off': _Sum()})[side].add(s)
  rows, extra_gal, heat_miles, heat_gal = [], 0.0, 0.0, 0.0
  for b, sides in sorted(bands.items()):
    h, o = sides['heat'], sides['off']
    if h.miles < MIN_MILES or o.miles < MIN_MILES or h.mpg is None or o.mpg is None:
      continue
    rows.append({'band': _band_name(b), 'heat_mpg': round(h.mpg, 1), 'off_mpg': round(o.mpg, 1),
                 'heat_ev': round(h.ev_share, 3) if h.ev_share is not None else None,
                 'off_ev': round(o.ev_share, 3) if o.ev_share is not None else None,
                 'heat_miles': round(h.miles, 1), 'off_miles': round(o.miles, 1)})
    extra_gal += h.miles * (1.0 / h.mpg - 1.0 / o.mpg)
    heat_miles += h.miles
    heat_gal += h.gal
  total_heat = sum(sides['heat'].miles for sides in bands.values())
  out = {'bands': rows, 'heat_miles': round(total_heat, 1), 'extra_gal': round(extra_gal, 4)}
  if not rows:
    out['status'] = 'collecting'
    out['line'] = (f'Heat: {total_heat:.0f} mi with the heat on, but not enough heat-off driving at the same outside ' +
                   'temperature to compare yet')
  else:
    share = extra_gal / heat_gal if heat_gal > 0 else 0.0
    parts = '; '.join(f"{r['band']}: {r['heat_mpg']} mpg heat on vs {r['off_mpg']} off" for r in rows)
    out['status'], out['cost_share'] = 'ok', round(share, 3)
    out['line'] = (f'Heat: costs about {100 * share:.0f}% of the fuel on the {heat_miles:.0f} heated miles compared ' +
                   f'({extra_gal:.2f} gal) - {parts}')
  return out


def tire_lever(stretches: list[dict], cold: set[int]) -> dict:
  bands: dict[int, dict[int, _Sum]] = {}
  low_seen = None
  for s in stretches:
    psi = s.get('tire_low') if isinstance(s.get('tire_low'), (int, float)) else s.get('tire')
    if not isinstance(psi, (int, float)) or not math.isfinite(psi):
      continue
    low_seen = psi if low_seen is None else min(low_seen, psi)
    b = _band(s)
    if b is None or id(s) in cold:
      continue
    bands.setdefault(b, {}).setdefault(int(round(psi)), _Sum()).add(s)
  rows = []
  for b, by_psi in sorted(bands.items()):
    ok = {p: c for p, c in by_psi.items() if c.miles >= MIN_MILES and c.mpg is not None}
    if len(ok) >= 2:
      lo, hi = min(ok), max(ok)
      rows.append({'band': _band_name(b), 'mpg_by_psi': {str(p): round(c.mpg, 1) for p, c in sorted(ok.items())},
                   'low_psi': lo, 'high_psi': hi, 'low_mpg': round(ok[lo].mpg, 1), 'high_mpg': round(ok[hi].mpg, 1)})
  out = {'placard_psi': PLACARD_PSI, 'placard_assumed': True, 'lowest_psi': low_seen, 'bands': rows,
         'under_placard': low_seen is not None and low_seen < PLACARD_PSI}
  # The tire-pressure readings are taken while driving, when warm tires read a few psi above their cold pressure, so a
  # reading under the cold placard means the tires really were low.
  flag = (f'the tires read as low as {low_seen:.0f} psi while driving, under the {PLACARD_PSI:.0f} psi cold door-sticker ' +
          'pressure (warm tires should read above it), so add air; '
          if out['under_placard'] else '')
  if not rows:
    out['status'] = 'collecting'
    out['line'] = f'Tires: {flag}not enough driving at two pressures in the same weather to compare yet'
  else:
    parts = '; '.join(f"{r['band']}: {r['low_psi']} psi {r['low_mpg']} mpg vs {r['high_psi']} psi {r['high_mpg']} mpg"
                      for r in rows)
    out['status'] = 'ok'
    out['line'] = f'Tires: {flag}{parts} (other things changed too: a hint, not proof)'
  return out


# ---------- cold starts ----------
def cold_lever(stretches: list[dict], cold: set[int]) -> dict:
  c, w = _Sum(), _Sum()
  for s in stretches:
    (c if id(s) in cold else w).add(s)
  total = c.gal + w.gal
  out = {'cold_miles': round(c.miles, 1), 'cold_gal': round(c.gal, 4),
         'cold_share': round(c.gal / total, 3) if total > 0 else None,
         'cold_mpg': round(c.mpg, 1) if c.mpg else None, 'warm_mpg': round(w.mpg, 1) if w.mpg else None}
  if c.miles < MIN_MILES or w.miles < MIN_MILES or c.mpg is None or w.mpg is None:
    out['status'] = 'collecting'
    out['line'] = 'Cold starts: not enough measured driving yet'
  else:
    out['status'] = 'ok'
    out['line'] = (f'Cold starts: {100 * out["cold_share"]:.0f}% of the fuel went into warming up ({c.miles:.0f} mi at ' +
                   f'{c.mpg:.1f} mpg vs {w.mpg:.1f} warm); fewer, longer trips and less idling to warm up save it')
  return out


def report(root: Path, today: date | None = None) -> dict:
  docs, span = window(load_docs(root), today)
  stretches = _stretches(docs)
  cold = _cold_ids(docs)
  return {'span': span, 'drives': len(docs), 'speed': speed_lever(stretches, cold), 'heat': heat_lever(stretches, cold),
          'tires': tire_lever(stretches, cold), 'cold_start': cold_lever(stretches, cold)}


def report_lines(doc: dict) -> list[str]:
  lines = [f"Fuel levers the comma cannot pull ({doc['span']}, {doc['drives']} drives):"]
  for key in ('speed', 'heat', 'tires', 'cold_start'):
    lines.append('  ' + doc[key]['line'])
  return lines
