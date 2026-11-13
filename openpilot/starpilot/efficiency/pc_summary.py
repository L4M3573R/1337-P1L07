#!/usr/bin/env python3
"""PC side of the fuel comparison: pull the comma's efficiency ledger and summarize it for a graph.

  python -m openpilot.starpilot.efficiency.pc_summary --host <comma ip> [--host <other ip>] [--out summary.json]

Copies /data/starpilot/efficiency/drives/*.json from the first reachable host into --data (default
./fuel_data), then writes per-drive MPG for the driver and the comma (BlueDriver, battery-adjusted), the
running averages, the matched comma-versus-driver comparison per comma mode, trip reports (per drive, MPG by
drive mode, tire pressure, outside temperature and road type, fill-ups and a tank-level fuel check) and the
latest BlueDriver health check, as JSON.
"""
import argparse
import json
import subprocess
import time
from pathlib import Path

from openpilot.starpilot.efficiency import compare, trend, trips


def pull(hosts: list[str], local: Path, key: str) -> str | None:
  (local / 'drives').mkdir(parents=True, exist_ok=True)
  for host in hosts:
    r = subprocess.run(['rsync', '-a', '--timeout=20', '-e', f'ssh -i {key} -o BatchMode=yes -o ConnectTimeout=6',
                        f'comma@{host}:/data/starpilot/efficiency/drives/', f'{local}/drives/'], capture_output=True, text=True,
                       check=False)
    if r.returncode == 0:
      subprocess.run(['rsync', '-a', '--timeout=20', '-e', f'ssh -i {key} -o BatchMode=yes -o ConnectTimeout=6',
                      f'comma@{host}:/data/starpilot/bluedriver/health.json', f'{local}/health.json'], capture_output=True, check=False)
      return host
  return None


def stored_drives(local: Path) -> list[tuple[float, list[dict]]]:
  drives = []
  for path in (local / 'drives').glob('*.json'):
    try:
      doc = json.loads(path.read_text())
      drives.append((path.stat().st_mtime, list(doc.get('stretches', []))))
    except (OSError, ValueError, TypeError):
      continue
  return sorted(drives, key=lambda d: d[0])


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
  print('current averages: you', out['you_avg'][-1] if n else None, '| comma', out['comma_avg'][-1] if n else None)


if __name__ == '__main__':
  main()
