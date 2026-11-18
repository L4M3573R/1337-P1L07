"""The car's own MPG at each speed, measured, so cruising picks its speed from what this car does instead of assuming
slower is cheaper (the driver, 2026-11-18: "I wanted it to learn my car, not just assume my car").

Every stored stretch with BlueDriver fuel goes into the 1 mph bin of its average speed, whoever drove, with the
battery, the hills and any speed gained or lost taken out (experiment_learn's corrections): a bin is the fuel this car
needs to hold that speed on the level. A speed's estimate pools its own bin with the one on each side (the car's
curve is smooth; single bins are a few miles each), and its range is a bootstrap over drives, so one drive's wind,
traffic or cold engine cannot carry a speed.

The car is measured per condition (the driver, 2026-11-18: "it needs to learn those speeds at different grades,
temperature, use"): a table per grade (uphill / flat / downhill, from the zeroed pitch), each at three levels, most
specific first: the current outside-temperature band and climate use (heat / cool / off), the temperature band alone,
and all driving. Since the same day ("my car's driving modes will need to be tested as well") the car's drive mode (eco / sport) comes
first: mode + temperature + climate, mode + temperature, mode, all driving. A decision is made inside one level only
(65 in the cold with the heater on is never weighed against 63 on a warm day), at the most specific level that has
measured the set speed and another speed in the window. The condition is the newest stored drive's when the judge
runs (as the drive starts): a mode or heater switched mid-drive is recorded right but only decides from the next drive.

The judge stores the tables in experiments.json; the cruise, uphill and downhill episodes (experiment.DialExperimenter)
pick from them inside the driver's window around the set speed: the speed that clearly saves fuel, or, half the time,
a speed in the window that this condition has not driven enough to judge yet. pc_summary prints the same tables.

All speeds are the car's wheel speed (carState.vEgo). Against GPS (2026-11-18, 9,571 steady samples over 53 drives)
the wheel sensors read 1.3 % low and the dash about 1.3 % high: wheel 65.2 is GPS 66.1 and 67 on the dash.
"""
import math
import random
from collections import defaultdict

from openpilot.starpilot.efficiency import experiment_learn as el

MPS_TO_MPH = 2.23694
MIN_STRETCH_S = 2.0
MAX_SPEED_CHANGE = 2.0   # m/s over one stretch: more is speeding up or slowing, not holding a speed
MIN_MILES = 10.0         # a speed is judged with this much driving at it and its two neighbours ...
MIN_OWN_MILES = 2.0      # ... this much at the speed itself (never judged from its neighbours alone) ...
MIN_DRIVES = 3           # ... on at least this many drives
THIN_MILES = 5.0         # a speed in the window is driven on purpose until it has this much of its own
RESAMPLES = 400
UNDER_MAX_MPH = 2        # the driver's limit under the set speed
OVER_MAX_MPH = 2         # the most the config may allow over it (experiment.json cruise_over_mph, 0 unless set)
GRADE_BAND = 0.02        # +-2 % (zeroed pitch) separates uphill / flat / downhill, as in the ledger
GRADES = ('flat', 'uphill', 'downhill')


def grade_of(stretch: dict, pitch_zero: float = 0.0) -> str:
  pitch = stretch.get('pitch')
  if not isinstance(pitch, (int, float)) or not math.isfinite(pitch):
    return 'flat'
  grade = math.tan(pitch - pitch_zero)
  return 'uphill' if grade > GRADE_BAND else 'downhill' if grade < -GRADE_BAND else 'flat'


def current_value(drives: list[list[dict]], field: str) -> str | None:
  """What the newest drive that recorded this field (climate, drive_mode) mostly had, by distance."""
  for stretches in reversed(drives):
    metres: dict[str, float] = defaultdict(float)
    for st in stretches:
      if isinstance(st, dict) and isinstance(st.get(field), str) and st[field] != 'unknown':
        metres[st[field]] += st.get('m') or 0.0
    if metres:
      return max(metres, key=metres.get)
  return None


def current_climate(drives: list[list[dict]]) -> str | None:
  """The climate use (car_signals.climate_band) of the newest drive that recorded one."""
  return current_value(drives, 'climate')


def current_mode(drives: list[list[dict]]) -> str | None:
  """The car's drive mode (eco / sport / ...) of the newest drive that recorded one."""
  return current_value(drives, 'drive_mode')


def _cells(drives: list[list[dict]], corrections: dict, keep=None) -> dict[int, dict[int, list[float]]]:
  """mph -> drive number -> [metres, gallons, engine-off seconds, seconds]."""
  cells: dict[int, dict[int, list[float]]] = defaultdict(lambda: defaultdict(lambda: [0.0, 0.0, 0.0, 0.0]))
  for n, stretches in enumerate(drives):
    for s in stretches:
      if not isinstance(s, dict):
        continue
      dur, m = s.get('dur') or 0.0, s.get('m') or 0.0
      v0, v1 = s.get('v0'), s.get('v1')
      if dur < MIN_STRETCH_S or m <= 0.0 or (keep is not None and not keep(s)):
        continue
      if isinstance(v0, (int, float)) and isinstance(v1, (int, float)) and abs(v1 - v0) > MAX_SPEED_CHANGE:
        continue
      gal = el.corrected_gal(s, corrections)
      if gal is None:
        continue
      cell = cells[round(m / dur * MPS_TO_MPH)][n]   # rounded: a car holding 65 belongs to 65, not half to 64
      cell[0] += m
      cell[1] += gal
      cell[2] += (s.get('ev') or 0.0) * dur
      cell[3] += dur
  return cells


def _mpg(sums) -> float | None:
  metres, gal = sum(x[0] for x in sums), sum(x[1] for x in sums)
  return metres / el.METERS_PER_MILE / gal if gal > 0 and metres > 0 else None


def speed_curve(drives: list[list[dict]], low_mph: int = 25, rng: random.Random | None = None,
                resamples: int = RESAMPLES, corrections: dict | None = None, keep=None) -> dict:
  """{'rows': [{mph, miles, drives, mpg, ev_share, est, low, high, est_miles, est_drives, enough}], ...}: mpg is the
  bin alone, est the pooled estimate with its 90 % range over drives."""
  rng = rng or random.Random(0)
  corrections = corrections or el.fit_corrections(drives)
  cells = _cells(drives, corrections, keep)
  rows = []
  for mph in sorted(k for k in cells if k >= low_mph):
    own = list(cells[mph].values())
    miles = sum(x[0] for x in own) / el.METERS_PER_MILE
    pooled: dict[int, list[float]] = defaultdict(lambda: [0.0, 0.0])
    for near in (mph - 1, mph, mph + 1):
      for n, x in cells.get(near, {}).items():
        pooled[n][0] += x[0]
        pooled[n][1] += x[1]
    names = list(pooled)
    est = _mpg(pooled.values())
    if est is None or miles < 0.2:
      continue
    draws = sorted(v for _ in range(resamples) if (v := _mpg([pooled[n] for n in rng.choices(names, k=len(names))])) is not None)
    est_miles, seconds = sum(x[0] for x in pooled.values()) / el.METERS_PER_MILE, sum(x[3] for x in own)
    raw = _mpg(own)
    rows.append({'mph': mph, 'miles': round(miles, 1), 'drives': len(own), 'mpg': None if raw is None else round(raw, 1),
                 'ev_share': round(sum(x[2] for x in own) / seconds, 3) if seconds > 0 else None,
                 'est': round(est, 1), 'low': round(draws[int(0.05 * len(draws))], 1) if draws else None,
                 'high': round(draws[max(int(0.95 * len(draws)) - 1, 0)], 1) if draws else None,
                 'est_miles': round(est_miles, 1), 'est_drives': len(names),
                 'enough': bool(draws) and est_miles >= MIN_MILES and miles >= MIN_OWN_MILES and len(names) >= MIN_DRIVES})
  return {'rows': rows, 'pitch_zero': corrections.get('pitch_zero'), 'hill_correction': corrections.get('source')}


def table(curve: dict) -> dict[str, dict]:
  """The curve as the experimenter reads it from experiments.json: mph (text) -> its estimate."""
  return {str(r['mph']): {'mi': r['miles'], 'est': r['est'], 'low': r['low'], 'high': r['high'], 'ok': r['enough']}
          for r in curve.get('rows', [])}


def condition_tables(drives: list[list[dict]], corrections: dict | None = None, season: str | None = None,
                     climate: str | None = None, rng: random.Random | None = None, resamples: int = RESAMPLES,
                     mode: str | None = None) -> dict:
  """{'flat': [tables], 'uphill': [...], 'downhill': [...], 'season', 'climate', 'mode', 'levels'}: per grade, the
  car's table at each level, most specific first: this drive mode in this temperature band with this climate use, this
  drive mode in this temperature band, this drive mode, all driving. A condition that is not known leaves its levels
  out (no drive mode: temperature band and climate, temperature band, all driving)."""
  corrections = corrections or el.fit_corrections(drives)
  zero = corrections.get('pitch_zero', 0.0) or 0.0

  def same(st, want_mode, want_season, want_climate):
    return ((want_mode is None or st.get('drive_mode') == want_mode) and
            (want_season is None or el.season_of(st.get('ambient_c')) == want_season) and
            (want_climate is None or st.get('climate') == want_climate))

  wanted = []   # (name, drive mode, temperature band, climate)
  if season is not None and climate is not None:
    wanted.append((', '.join(x for x in (mode, season, f'climate {climate}') if x), mode, season, climate))
  if season is not None:
    wanted.append((', '.join(x for x in (mode, season) if x), mode, season, None))
  if mode is not None:
    wanted.append((mode, mode, None, None))
  wanted.append(('all driving', None, None, None))
  levels = [(name, lambda st, m=m, se=se, c=c: same(st, m, se, c)) for name, m, se, c in wanted]
  out: dict = {'season': season, 'climate': climate, 'mode': mode, 'levels': [name for name, _ in levels]}
  for grade in GRADES:
    out[grade] = [table(speed_curve(drives, rng=rng or random.Random(0), resamples=resamples, corrections=corrections,
                                    keep=lambda st, ok=ok, grade=grade: grade_of(st, zero) == grade and ok(st)))
                  for _, ok in levels]
  return out


def _row(speeds: dict, mph: int) -> dict | None:
  r = speeds.get(str(mph)) if isinstance(speeds, dict) else None
  ok = isinstance(r, dict) and all(isinstance(r.get(k), (int, float)) and r[k] > 0 for k in ('est', 'low', 'high'))
  return r if ok else None


def _window(set_mph: float, over_mph: float, under_mph: float = UNDER_MAX_MPH) -> tuple[int, range]:
  """The set speed (whole mph) and the speeds the caller's limits allow around it."""
  s = round(set_mph)
  return s, range(s - int(max(under_mph, 0)), s + int(max(over_mph, 0)) + 1)


def choose_u(speeds: dict, set_mph: float, over_mph: float = 0.0, allowed=lambda u: True,
             under_mph: float = UNDER_MAX_MPH) -> float:
  """How far under the set speed to cruise (negative: over it), from the measured table: the speed in the window that
  clearly uses less fuel than the set speed itself, else 0. Clearly: at least MIN_GAIN better, with the two ranges
  apart; a slower speed must also be worth its minutes (MIN_GAL_PER_MINUTE)."""
  s, window = _window(set_mph, over_mph, under_mph)
  own = _row(speeds, s)
  if own is None or not own.get('ok'):
    return 0.0   # the set speed itself is not measured yet: nothing to compare with
  best, best_est = s, own['est']
  for c in window:
    r = _row(speeds, c)
    if c == s or r is None or not r.get('ok') or not allowed(float(s - c)):
      continue
    if r['est'] < own['est'] * (1.0 + el.MIN_GAIN) or r['low'] <= own['est'] or r['est'] <= own['high']:
      continue
    if c < s and (1.0 / own['est'] - 1.0 / r['est']) / (60.0 * (1.0 / c - 1.0 / s)) < el.MIN_GAL_PER_MINUTE:
      continue
    if r['est'] > best_est:
      best, best_est = c, r['est']
  return float(s - best)


def judged_level(levels: list, set_mph: float, over_mph: float = 0.0, under_mph: float = UNDER_MAX_MPH) -> dict | None:
  """The most specific table that has measured the set speed and another speed in the window: a decision compares
  speeds inside one condition only."""
  s, window = _window(set_mph, over_mph, under_mph)
  for t in levels if isinstance(levels, list) else []:
    own = _row(t, s)
    if own is not None and own.get('ok') and any((r := _row(t, c)) is not None and r.get('ok') for c in window if c != s):
      return t
  return None


def choose(levels: list, set_mph: float, over_mph: float = 0.0, allowed=lambda u: True,
           under_mph: float = UNDER_MAX_MPH) -> float:
  """choose_u at the most specific condition that can decide; 0 when none can yet."""
  t = judged_level(levels, set_mph, over_mph, under_mph)
  return choose_u(t, set_mph, over_mph, allowed, under_mph) if t is not None else 0.0


def thin(levels: list, set_mph: float, over_mph: float = 0.0, under_mph: float = UNDER_MAX_MPH) -> list[float]:
  """thin_us for the most specific condition (the first table): every condition gets driven, not just the pooled one."""
  first = levels[0] if isinstance(levels, list) and levels else {}
  return thin_us(first, set_mph, over_mph, under_mph)


def thin_us(speeds: dict, set_mph: float, over_mph: float = 0.0, under_mph: float = UNDER_MAX_MPH) -> list[float]:
  """The window's speeds that are not driven enough to judge yet, least driven first (as mph under the set speed)."""
  s, window = _window(set_mph, over_mph, under_mph)
  need = []
  for c in window:
    r = speeds.get(str(c)) if isinstance(speeds, dict) else None
    mi = r.get('mi') if isinstance(r, dict) and isinstance(r.get('mi'), (int, float)) else 0.0
    if c != s and c > 0 and mi < THIN_MILES:
      need.append((mi, abs(c - s), float(s - c)))
  return [u for _, _, u in sorted(need)]


def report_lines(curve: dict, low_mph: int = 55, over_mph: float = 0.0) -> list[str]:
  head = 'MPG holding a speed on the level (hills and battery taken out; pooled with the speed on each side; '
  lines = [head + '* = not enough driving to judge):']
  for r in curve['rows']:
    if r['mph'] >= low_mph:
      own = f"alone {r['mpg']} on {r['miles']} mi / {r['drives']} drives, engine off {r['ev_share']:.0%}"
      lines.append(f"  {r['mph']:>2} mph{' ' if r['enough'] else '*'} {r['est']:5.1f} mpg ({r['low']}-{r['high']}), {own}")
  if not any(r['enough'] for r in curve['rows']):
    lines.append('  no speed has enough driving yet')
  return lines


def condition_lines(doc: dict, low_mph: int = 55, over_mph: float = 0.0) -> list[str]:
  """What each grade's tables hold and what cruising would do at each set speed, in the current condition."""
  now = f"{doc.get('mode') or 'drive mode unknown'}, {doc.get('season') or 'temperature unknown'}, climate {doc.get('climate') or 'unknown'}"
  lines = [f'Speed by condition (now: {now}; wheel speed: GPS is about 1 mph more at 65, the dash about 2):']
  limits = {'flat': (UNDER_MAX_MPH, min(max(over_mph, 0), OVER_MAX_MPH)), 'uphill': (UNDER_MAX_MPH, 0), 'downhill': (0, 3)}
  for grade in GRADES:
    levels = doc.get(grade) or []
    under, over = limits[grade]
    for name, t in zip(doc.get('levels', []), levels, strict=False):
      judged = [int(k) for k, r in t.items() if r.get('ok') and int(k) >= low_mph]
      miles = sum(r.get('mi') or 0.0 for k, r in t.items() if int(k) >= low_mph)
      lines.append(f"  {grade:8s} {name:28s} {miles:6.1f} mi at {low_mph}+ mph, judged speeds: {', '.join(map(str, sorted(judged))) or 'none yet'}")
    picks = []
    for mph in range(low_mph, 76):
      if judged_level(levels, mph, over, under) is not None:
        u = choose(levels, mph, over, under_mph=under)
        picks.append(f'{mph}' + (f'->{mph - u:.0f}' if u else ''))
    lines.append(f"  {grade:8s} set speed -> cruise: {', '.join(picks) or 'nothing decided yet'}")
  return lines
