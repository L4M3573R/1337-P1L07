#!/usr/bin/env python3
"""PC side of the fuel comparison: pull the comma's efficiency ledger and summarize it for a graph.

  python -m openpilot.starpilot.efficiency.pc_summary --host <comma ip> [--host <other ip>] [--out summary.json]

Copies /data/starpilot/efficiency/drives/*.json from the first reachable host into --data (default
./fuel_data), then writes per-drive MPG for the driver and the comma (BlueDriver, battery-adjusted), the
running averages, the matched comma-versus-driver comparison per comma mode, trip reports (per drive, MPG by
drive mode, tire pressure, outside temperature and road type, fill-ups and a tank-level fuel check) and the
latest BlueDriver health check, measured MPG by steady speed (speed_curve.py), and the shadow efficiency learner's
findings (learner.py), as JSON. With
<efficiency>/learner_ledger.json {"start": epoch s} on the comma, also the self-learner's own ledger: its drives from
that moment to the next fill-up, MPG per drive, running MPG and the matched comparison with the driver.
"""
import argparse
import json
import subprocess
import time
from pathlib import Path

from openpilot.starpilot.efficiency import compare, experiment_learn, learner, speed_curve, trend, trips


def pull(hosts: list[str], local: Path, key: str) -> str | None:
  (local / 'drives').mkdir(parents=True, exist_ok=True)
  for host in hosts:
    r = subprocess.run(['rsync', '-a', '--timeout=20', '-e', f'ssh -i {key} -o BatchMode=yes -o ConnectTimeout=6',
                        f'comma@{host}:/data/starpilot/efficiency/drives/', f'{local}/drives/'], capture_output=True, text=True,
                       check=False)
    if r.returncode == 0:
      for remote, name in (('/data/starpilot/bluedriver/health.json', 'health.json'),
                           ('/data/starpilot/efficiency/learner_ledger.json', 'learner_ledger.json'),
                           ('/data/starpilot/efficiency/tank_history.json', 'tank_history.json'),
                           ('/data/starpilot/efficiency/experiment.json', 'experiment.json'),
                           ('/data/starpilot/efficiency/pump_fills.json', 'pump_fills.json')):
        subprocess.run(['rsync', '-a', '--timeout=20', '-e', f'ssh -i {key} -o BatchMode=yes -o ConnectTimeout=6',
                        f'comma@{host}:{remote}', f'{local}/{name}'], capture_output=True, check=False)
      return host
  return None


def stored_drives(local: Path) -> list[tuple[float, list[dict]]]:
  from openpilot.starpilot.efficiency.store import rescale
  drives = []
  for path in (local / 'drives').glob('*.json'):
    try:
      doc = rescale(json.loads(path.read_text()))
      drives.append((path.stat().st_mtime, list(doc.get('stretches', []))))
    except (OSError, ValueError, TypeError):
      continue
  return sorted(drives, key=lambda d: d[0])


def learner_ledger(local: Path, points: list) -> dict | None:
  """The self-learner's own ledger: comma-driven drives from the start mark to the next fill-up after it (the tank
  tracker's finished-tank record, the same fill-up that resets the fuel tile; the gauge-based fill guess is too noisy)."""
  from openpilot.starpilot.efficiency.ledger import Totals
  from openpilot.starpilot.efficiency.store import KEY_SEPARATOR, load_drives
  try:
    start = float(json.loads((local / 'learner_ledger.json').read_text())['start'])
  except (OSError, ValueError, TypeError, KeyError):
    return None
  fills = []
  try:
    for entry in json.loads((local / 'tank_history.json').read_text()):
      fills.append(time.mktime(time.strptime(entry['ended'], '%Y-%m-%d %H:%M')))
  except (OSError, ValueError, TypeError, KeyError):
    pass
  end = min((t for t in fills if t > start), default=None)
  inside = lambda t: t >= start and (end is None or t < end)  # noqa: E731
  rows, miles, gal = [], 0.0, 0.0
  for p in points:
    if inside(p.t) and p.comma_mi > 0 and p.comma_mpg:
      miles += p.comma_mi
      gal += p.comma_mi / p.comma_mpg
      rows.append({'date': time.strftime('%m-%d %H:%M', time.localtime(p.t)), 'comma_mi': round(p.comma_mi, 2),
                   'comma_mpg': round(p.comma_mpg, 1), 'running_mpg': round(miles / gal, 1)})
  window = set()
  for drive, _, _ in load_drives(local):
    try:
      if inside((local / 'drives' / f'{drive}.json').stat().st_mtime):
        window.add(drive)
    except OSError:
      continue
  contexts: dict[tuple, Totals] = {}
  for drive, _, by_key in load_drives(local):
    for key, t in by_key.items():
      parts = tuple(key.split(KEY_SEPARATOR))
      who = parts[4] if len(parts) > 4 else ''
      if who == 'driver' or (who in compare.COMMA_MODES and drive in window):
        contexts.setdefault(parts, Totals()).add_totals(t)
  c = compare.compare(contexts)
  return {'start': time.strftime('%m-%d %H:%M', time.localtime(start)),
          'ended_at_fill': time.strftime('%m-%d %H:%M', time.localtime(end)) if end else None,
          'drives': rows, 'comma_miles': round(miles, 2), 'comma_mpg': round(miles / gal, 1) if gal else None,
          'vs_you': {'matched_miles': round(c.matched_miles, 2), 'ready': c.ready, 'comma_saving': c.comma_saving}}


def pump_tanks(local: Path) -> list[dict]:
  """Real tank MPG from the receipt gallons (tank.py --pump-gal): every mile the comma logged from the fill that began
  the tank up to the fill that ended it, over the pump gallons; who drove it; and how far the BlueDriver's own count
  of the tank's fuel is from the pump (scaled to all the tank's miles; it reads low)."""
  from openpilot.starpilot.efficiency.store import rescale
  from openpilot.starpilot.efficiency.tank import load_pump_log
  docs = []
  for path in sorted((local / 'drives').glob('*.json')):
    try:
      docs.append((path.stem, rescale(json.loads(path.read_text()))))
    except (OSError, ValueError, TypeError):
      continue
  out = []
  for entry in load_pump_log(local):
    since, filled, pump = entry.get('since_drive'), entry.get('filled_drive'), entry.get('pump_gal')
    if not since or not filled or not isinstance(pump, (int, float)) or pump <= 0:
      continue
    meters = obd_gal = obd_m = comma_m = driver_m = 0.0
    for name, doc in docs:
      if since <= name < filled:
        total = doc.get('total') or {}
        meters += float(total.get('distance') or 0.0)
        obd_gal += float(total.get('obd_fuel_gal') or 0.0)
        obd_m += float(total.get('obd_distance') or 0.0)
        for st in doc.get('stretches') or []:
          if st.get('mode') == 'driver':
            driver_m += float(st.get('m') or 0.0)
          else:
            comma_m += float(st.get('m') or 0.0)
    miles = meters / 1609.344
    bd_gal = obd_gal * meters / obd_m if obd_m > 0 else None
    out.append({'since': since, 'filled': filled, 'pump_gal': pump, 'miles': round(miles, 1),
                'mpg': round(miles / pump, 1) if miles > 0 else None,
                'comma_share': round(comma_m / (comma_m + driver_m), 3) if comma_m + driver_m > 0 else None,
                'bluedriver_gal': round(bd_gal, 2) if bd_gal else None,
                'bluedriver_reads_low': round(1.0 - bd_gal / pump, 3) if bd_gal else None})
  return out


def summarize(local: Path, host: str | None, gas_price: float | None = None, tank_gal: float = trips.TANK_GAL) -> dict:
  points = trend.load_points(local)
  you, comma = [p.you_mpg for p in points], [p.comma_mpg for p in points]
  out = {
    'pulled_from': host, 'pulled_at': time.strftime('%Y-%m-%d %H:%M'),
    'drives': [{'t': p.t, 'date': time.strftime('%m-%d %H:%M', time.localtime(p.t)), 'you_mpg': p.you_mpg, 'comma_mpg': p.comma_mpg,
                'you_mi': round(p.you_mi, 2), 'comma_mi': round(p.comma_mi, 2)} for p in points],
    'you_avg': trend.rolling(you), 'comma_avg': trend.rolling(comma),
    'comparison': {},
  }
  for label, modes in (('all', compare.COMMA_MODES), ('chill', ('chill',)), ('experimental', ('experimental',)),
                       ('pulse-glide', ('pulse-glide',))):
    c = compare.stored_comparison(local, modes)
    out['comparison'][label] = {'matched_miles': round(c.matched_miles, 2), 'contexts': c.contexts, 'ready': c.ready,
                                'comma_saving': c.comma_saving}
  drives = stored_drives(local)
  records = [r for _, rs in drives for r in rs]
  out['trips'] = [{'date': time.strftime('%m-%d %H:%M', time.localtime(t)), **trips.drive_summary(t, rs, gas_price)}
                  for t, rs in drives if rs]
  out['breakdowns'] = trips.breakdowns(records)
  out['drive_modes_vs_eco'] = trips.drive_mode_comparison(records)
  out['fill_ups'] = trips.fill_ups(drives, tank_gal)
  out['tank_check'] = trips.tank_check(drives, tank_gal)
  all_drives = [rs for _, rs in drives]
  out['speed_curve'] = speed_curve.speed_curve(all_drives)
  out['speed_conditions'] = speed_curve.condition_tables(all_drives, None, experiment_learn.current_season(all_drives),
                                                         speed_curve.current_climate(all_drives),
                                                         mode=speed_curve.current_mode(all_drives))
  out['speed_by_mode'] = {m: speed_curve.table(speed_curve.speed_curve(all_drives, keep=lambda st, m=m: st.get('drive_mode') == m))
                          for m in sorted({st.get('drive_mode') for d in all_drives for st in d if isinstance(st.get('drive_mode'), str)} - {'unknown'})}
  out['learner'] = learner.learn_stored(local)
  out['learner_ledger'] = learner_ledger(local, points)
  out['pump_tanks'] = pump_tanks(local)
  try:
    out['health'] = json.loads((local / 'health.json').read_text())
  except (OSError, ValueError):
    out['health'] = None
  return out


def main():
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
  parser.add_argument('--host', action='append', default=[], help='comma address to try (repeatable, in order)')
  parser.add_argument('--key', default=str(Path.home() / '.ssh' / 'id_ed25519'))
  parser.add_argument('--data', type=Path, default=Path('fuel_data'))
  parser.add_argument('--out', type=Path, default=Path('fuel_summary.json'))
  parser.add_argument('--gas-price', type=float, default=None, help='dollars per gallon, for trip cost')
  parser.add_argument('--tank-gal', type=float, default=trips.TANK_GAL)
  args = parser.parse_args()
  host = pull(args.host, args.data, args.key) if args.host else None
  out = summarize(args.data, host, args.gas_price, args.tank_gal)
  args.out.write_text(json.dumps(out, indent=1))
  n = len(out['drives'])
  print(f"pulled from {host or 'nowhere (using the last copy)'}: {n} drives with measured fuel")
  print('comparison:', out['comparison']['all'])
  for tk in out['pump_tanks']:
    print(f"tank {tk['since']} .. {tk['filled']}: {tk['miles']} mi / {tk['pump_gal']} gal = {tk['mpg']} mpg real, comma drove " +
          f"{tk['comma_share']}, BlueDriver counted {tk['bluedriver_gal']} gal (reads low {tk['bluedriver_reads_low']})")
  print('current averages: you', out['you_avg'][-1] if n else None, '| comma', out['comma_avg'][-1] if n else None)
  print('\n'.join(learner.report_lines(out['learner'])))
  print('\n'.join(speed_curve.report_lines(out['speed_curve'])))
  print('\n'.join(speed_curve.condition_lines(out['speed_conditions'])))


if __name__ == '__main__':
  main()
