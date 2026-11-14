"""Self-learning fuel efficiency, shadow mode: learns from the energy ledger what Pulse and Glide should do on this
car and reports it. Nothing here commands the car; the PC summary shows the result for review.

Fuel is always the BlueDriver's measured fuel with the hybrid battery change paid back (never the dash average), and
stretches the BlueDriver did not fully cover are left out. Everything stays inside the driver's time budget: at most
TIME_BUDGET_MIN_PER_HR extra minutes per hour of driving.

It learns three things:
1. EV cut-off: the highest speed at which a feather-light glide keeps the hybrid electric, from the power-flow
   histogram (any comma-controlled driving at that light load counts), instead of the fixed FEATHER_MAX_SPEED_MPS.
2. Where Pulse and Glide pays: per speed band, its fuel per mile against steady cruise on matching roads (grade,
   curves, traffic and drive mode), and the deepest glide band the time budget allows at that speed
   (never more than MAX_UNDER_MPH under the set speed).
3. Downhill carry: on open downhill stretches the comma drove, the energy spent regenerating or braking to hold the
   set speed, and what letting the car run up to DOWNHILL_OVER_MPH over the set speed would have kept, plus the time
   that wins back.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from openpilot.starpilot.controllers.coast import FEATHER_MAX_SPEED_MPS
from openpilot.starpilot.efficiency.ledger import BATTERY_KWH, GAL_PER_BATTERY_KWH, MPH

METERS_PER_MILE = 1609.344
TIME_BUDGET_MIN_PER_HR = 2.5  # the driver's limit is 2-3 extra minutes an hour: plan to the middle of it
MAX_UNDER_MPH = 5.0
MIN_BAND_MPH = 1.0
DOWNHILL_OVER_MPH = 2.0
STEADY_MODES = ('chill', 'experimental')
PULSE_GLIDE_MODE = 'pulse-glide'

# EV cut-off: flat-road accelerations (2 mph x 0.05 m/s^2 histogram bins) that a feather glide asks for.
FEATHER_BINS = (-0.10, -0.05)
EV_SHARE = 0.85  # electric at least this share of the time
MIN_BIN_SAMPLES = 200  # 10 s at 20 Hz in one 2 mph bin
EV_SCAN_FROM_MPH = 40.0

# Pulse and Glide against steady cruise.
MIN_MILES_EACH = 1.0  # per side, per matched context
MIN_BAND_MILES = 5.0  # matched miles in a speed band before it gets a recommendation
MIN_SAVING = 0.02  # below this, the extra speed swings are not worth it

# Downhill carry. Estimates: energy put into the battery comes back at about this share, and a gallon of gasoline
# delivers about this much energy at the wheels through the hybrid powertrain (33.7 kWh at ~30%).
REGEN_ROUND_TRIP = 0.6
WHEEL_J_PER_GAL = 33.7 * 3.6e6 * 0.30
DEFAULT_MASS_KG = 1450.0


def band_depth_mph(set_mph: float, budget_min_per_hr: float = TIME_BUDGET_MIN_PER_HR) -> float:
  """Deepest glide band (mph under the set speed) the time budget allows. The car averages about halfway down the
  band, so the extra time is (depth / 2) / (set - depth / 2) of the trip."""
  if not math.isfinite(set_mph) or set_mph <= 0:
    return MIN_BAND_MPH
  f = budget_min_per_hr / 60.0
  return max(MIN_BAND_MPH, min(MAX_UNDER_MPH, 2.0 * set_mph * f / (1.0 + f)))


def band_time_cost(set_mph: float, depth_mph: float) -> float:
  """Extra minutes per hour a glide band this deep costs at this set speed."""
  avg = set_mph - depth_mph / 2.0
  return 60.0 * (set_mph / avg - 1.0) if avg > 0 else math.inf


def learn_ev_cutoff(flow: dict[str, list[float]]) -> dict:
  """flow: 'speed|accel|phase' -> [samples, ev, regen, engine, meter sum], as the ledger stores it."""
  by_speed: dict[float, list[float]] = {}
  for key, values in flow.items():
    try:
      speed, accel, _ = key.split('|')
      speed_bin, accel_bin = float(speed), round(float(accel), 2)
      samples, ev = float(values[0]), float(values[1])
    except (ValueError, IndexError, TypeError):
      continue
    if accel_bin in FEATHER_BINS and samples > 0:
      entry = by_speed.setdefault(speed_bin, [0.0, 0.0])
      entry[0] += samples
      entry[1] += ev
  shares = {b: v[1] / v[0] for b, v in sorted(by_speed.items()) if v[0] >= MIN_BIN_SAMPLES}
  out = {'current_mph': round(FEATHER_MAX_SPEED_MPS / MPH, 1), 'learned_mph': None, 'at_least': False,
         'samples': int(sum(v[0] for v in by_speed.values())), 'ev_share_by_mph': {f'{b:g}': round(s, 3) for b, s in shares.items()}}
  scanned = [(b, s) for b, s in shares.items() if b >= EV_SCAN_FROM_MPH]
  if len(scanned) < 2:
    return out
  good = 0
  for speed_bin, share in scanned:
    if share < EV_SHARE:
      if good >= 2:  # electric below this bin, the engine starts in it
        out['learned_mph'] = speed_bin
      return out
    good += 1
  out['learned_mph'], out['at_least'] = scanned[-1][0] + 2.0, True  # electric all the way up the data so far
  return out


def _adjusted_gal(record: dict) -> float | None:
  gal = record.get('obd_gal')
  if not isinstance(gal, (int, float)):
    return None
  soc_d = record.get('soc_d') if isinstance(record.get('soc_d'), (int, float)) else 0.0
  return gal - soc_d / 100.0 * BATTERY_KWH * GAL_PER_BATTERY_KWH


@dataclass
class _Side:
  meters: float = 0.0
  seconds: float = 0.0
  gal: float = 0.0

  def add(self, record: dict, gal: float) -> None:
    self.meters += float(record['m'])
    self.seconds += float(record['dur'])
    self.gal += gal

  @property
  def miles(self) -> float:
    return self.meters / METERS_PER_MILE


@dataclass
class _Band:
  pg_miles: float = 0.0
  pg_gal: float = 0.0  # Pulse and Glide's fuel on the matched miles
  steady_gal: float = 0.0  # steady cruise's fuel on the same miles, at its own rate
  pg_hours: float = 0.0
  steady_hours: float = 0.0
  contexts: int = 0
  speed_m: list = field(default_factory=lambda: [0.0, 0.0])  # metres, seconds of steady cruise (its mean speed)


def learn_bands(records: list[dict], budget_min_per_hr: float = TIME_BUDGET_MIN_PER_HR) -> dict:
  """Per speed band: Pulse and Glide's battery-adjusted measured fuel against steady cruise on matching roads."""
  sides: dict[tuple, dict[str, _Side]] = {}
  for r in records:
    side = 'pg' if r.get('mode') == PULSE_GLIDE_MODE else 'steady' if r.get('mode') in STEADY_MODES else None
    gal, ctx = _adjusted_gal(r), r.get('ctx')
    if side is None or gal is None or not isinstance(ctx, list) or len(ctx) < 4 or not r.get('m') or not r.get('dur'):
      continue
    key = tuple(ctx[:4]) + (r.get('drive_mode') or 'unknown',)
    sides.setdefault(key, {}).setdefault(side, _Side()).add(r, gal)
  bands: dict[str, _Band] = {}
  for key, pair in sides.items():
    pg, steady = pair.get('pg'), pair.get('steady')
    if pg is None or steady is None or pg.miles < MIN_MILES_EACH or steady.miles < MIN_MILES_EACH:
      continue
    if pg.gal <= 0 or steady.gal <= 0:
      continue
    b = bands.setdefault(key[0], _Band())
    b.pg_miles += pg.miles
    b.pg_gal += pg.gal
    b.steady_gal += steady.gal / steady.miles * pg.miles
    b.pg_hours += pg.seconds / 3600.0
    b.steady_hours += steady.seconds / steady.meters * pg.meters / 3600.0  # steady cruise's time for the same miles
    b.contexts += 1
    b.speed_m[0] += steady.meters
    b.speed_m[1] += steady.seconds
  out = {}
  for name, b in sorted(bands.items()):
    entry = {'matched_miles': round(b.pg_miles, 2), 'contexts': b.contexts, 'saving': None, 'measured_time_cost_min_per_hr': None,
             'recommend': 'collecting', 'depth_mph': None, 'planned_time_cost_min_per_hr': None}
    if b.pg_miles >= MIN_BAND_MILES:
      saving = 1.0 - b.pg_gal / b.steady_gal
      set_mph = b.speed_m[0] / b.speed_m[1] / MPH if b.speed_m[1] > 0 else 0.0
      entry['saving'] = round(saving, 3)
      entry['measured_time_cost_min_per_hr'] = round(60.0 * (b.pg_hours / b.steady_hours - 1.0), 2) if b.steady_hours > 0 else None
      if saving >= MIN_SAVING:
        depth = band_depth_mph(set_mph, budget_min_per_hr)
        entry.update(recommend='pulse-glide', depth_mph=round(depth, 1), planned_time_cost_min_per_hr=round(band_time_cost(set_mph, depth), 2))
      else:
        entry['recommend'] = 'steady cruise'
    out[name] = entry
  return out


def learn_downhill(drives: list[list[dict]], mass_kg: float = DEFAULT_MASS_KG) -> dict:
  """Energy the comma spent holding the set speed on open downhill road, and what running up to DOWNHILL_OVER_MPH
  over it would have kept. Consecutive downhill stretches are one hill: the speed headroom is used once per hill."""
  dv = DOWNHILL_OVER_MPH * MPH
  hills = kept_j = gained_s = comma_m = 0.0
  for records in drives:
    hill: list[dict] = []
    for r in records + [None]:
      if r is not None and r.get('mode') in STEADY_MODES + (PULSE_GLIDE_MODE,):
        comma_m += float(r.get('m') or 0.0)
      ctx = r.get('ctx') if r is not None else None
      downhill = (r is not None and r.get('mode') in STEADY_MODES + (PULSE_GLIDE_MODE,) and isinstance(ctx, list) and len(ctx) >= 4
                  and ctx[1] == 'downhill' and ctx[3] == 'open road' and ctx[2] != 'curvy')
      if downhill:
        hill.append(r)
        continue
      if hill:
        meters = sum(float(h.get('m') or 0.0) for h in hill)
        seconds = sum(float(h.get('dur') or 0.0) for h in hill)
        dumped = sum((1.0 - REGEN_ROUND_TRIP) * float(h.get('regen_j') or 0.0) + float(h.get('fric_j') or 0.0) for h in hill)
        v = meters / seconds if seconds > 0 else 0.0
        if dumped > 0 and v > 0:
          hills += 1
          kept_j += min(dumped, mass_kg * v * dv)
          gained_s += meters / v - meters / (v + dv / 2.0)  # the speed builds up over the hill: about half dv on average
        hill = []
  comma_mi = comma_m / METERS_PER_MILE
  gal = kept_j / WHEEL_J_PER_GAL
  return {'over_mph': DOWNHILL_OVER_MPH, 'hills': int(hills), 'comma_miles': round(comma_mi, 1), 'est_gal_saved': round(gal, 6),
          'gal_per_100mi': round(100.0 * gal / comma_mi, 5) if comma_mi > 0 else None, 'time_won_s': round(gained_s, 1)}


def learn(drives: list[list[dict]], flow: dict[str, list[float]], mass_kg: float = DEFAULT_MASS_KG,
          budget_min_per_hr: float = TIME_BUDGET_MIN_PER_HR) -> dict:
  records = [r for rs in drives for r in rs if isinstance(r, dict)]
  return {'mode': 'shadow', 'time_budget_min_per_hr': budget_min_per_hr, 'max_under_mph': MAX_UNDER_MPH,
          'ev_cutoff': learn_ev_cutoff(flow), 'bands': learn_bands(records, budget_min_per_hr),
          'downhill': learn_downhill([[r for r in rs if isinstance(r, dict)] for rs in drives], mass_kg)}


def learn_stored(root: Path, mass_kg: float = DEFAULT_MASS_KG) -> dict:
  """From a folder of drive files (the comma's efficiency/drives, or the PC copy of them), oldest first."""
  from openpilot.starpilot.efficiency.store import VERSION
  drives, flow = [], {}
  paths = sorted((root / 'drives').glob('*.json'), key=lambda p: p.stat().st_mtime)
  for path in paths:
    try:
      doc = json.loads(path.read_text())
    except (OSError, ValueError):
      continue
    if not isinstance(doc, dict) or doc.get('version') != VERSION:
      continue
    drives.append(list(doc.get('stretches') or []))
    for key, values in (doc.get('flow') or {}).items():
      try:
        entry = flow.setdefault(key, [0.0] * 5)
        for i in range(5):
          entry[i] += float(values[i])
      except (ValueError, IndexError, TypeError):
        continue
  return learn(drives, flow, mass_kg)


def report_lines(result: dict) -> list[str]:
  """Plain-language summary for the terminal."""
  lines = [f"Efficiency learner (shadow, budget {result['time_budget_min_per_hr']:g} min/hr):"]
  ev = result['ev_cutoff']
  if ev['learned_mph'] is None:
    lines.append(f"  EV cut-off: collecting ({ev['samples']} light-glide samples; fixed at {ev['current_mph']:g} mph)")
  else:
    lines.append(f"  EV cut-off: {'at least ' if ev['at_least'] else ''}{ev['learned_mph']:g} mph learned (fixed at {ev['current_mph']:g} mph)")
  if not result['bands']:
    lines.append('  Pulse and Glide vs steady cruise: no matched miles yet')
  for band, b in result['bands'].items():
    if b['recommend'] == 'collecting':
      lines.append(f"  {band}: collecting ({b['matched_miles']:.1f} of {MIN_BAND_MILES:g} matched mi)")
    else:
      line = f"  {band}: Pulse and Glide {'saves' if b['saving'] >= 0 else 'costs'} {abs(b['saving']):.0%} -> {b['recommend']}"
      if b['depth_mph'] is not None:
        line += f", band {b['depth_mph']:g} mph under (about {b['planned_time_cost_min_per_hr']:.1f} min/hr)"
      lines.append(line)
  d = result['downhill']
  if d['hills']:
    kept = f"would keep ~{d['est_gal_saved']:.3f} gal ({d['gal_per_100mi']:.3f} gal/100 mi)"
    lines.append(f"  Downhill +{d['over_mph']:g} mph: {d['hills']} hills, {kept} and win back {d['time_won_s']:.0f} s")
  else:
    lines.append('  Downhill carry: no open downhill stretches held at speed yet')
  return lines
