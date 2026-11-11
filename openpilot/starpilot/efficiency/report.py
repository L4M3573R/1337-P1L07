#!/usr/bin/env python3
"""Energy ledger report for recorded drives.

  python -m openpilot.starpilot.efficiency.report <rlog file or folder> [...]

Prints where the wheel energy went overall, per control mode, and in the driving contexts that
waste the most (speed changes beyond steady cruising, and braking beyond estimated regen).
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

from openpilot.starpilot.efficiency.ledger import Ledger, Totals, make_sample, summarize

SAMPLE_PERIOD_S = 0.05
BATTERY_ADDR = 0x595  # Elantra Hybrid: byte 2 (signed) tracks hybrid battery current; uncalibrated


def log_files(paths: list[str]) -> list[Path]:
  files: list[Path] = []
  for p in map(Path, paths):
    if p.is_dir():
      files += sorted((f for f in p.iterdir() if f.suffix in ('.zst', '.bz2') or f.name.startswith('rlog')),
                      key=lambda f: int(''.join(ch for ch in f.stem.split('.')[0] if ch.isdigit()) or 0))
    else:
      files.append(p)
  return files


def samples(files: list[Path]):
  from openpilot.tools.lib.logreader import LogReader
  mass = cs = cc = radar = sd = pose = None
  battery = None
  last_t = -math.inf
  for f in files:
    for e in LogReader(str(f), sort_by_time=True):
      w = e.which()
      if w == 'carParams':
        mass = e.carParams.mass
      elif w == 'carControl':
        cc = e.carControl
      elif w == 'radarState':
        radar = e.radarState
      elif w == 'selfdriveState':
        sd = e.selfdriveState
      elif w == 'deviceMotion':  # this fork's pose message (angular velocity in the device frame)
        pose = e.deviceMotion
      elif w == 'can':
        for c in e.can:
          if c.address == BATTERY_ADDR and c.src == 0 and len(c.dat) > 2:
            raw = c.dat[2]
            battery = raw - 256 if raw > 127 else raw
      elif w == 'carState':
        cs = e.carState
        t = e.logMonoTime / 1e9
        # Each drive's clock restarts at boot: a backwards jump is a new drive, not a sample to skip.
        if mass is None or 0.0 <= t - last_t < SAMPLE_PERIOD_S:
          continue
        last_t = t
        yield make_sample(t, mass, cs, cc, radar, sd, pose, battery)


def line(name: str, t: Totals) -> str:
  excess = 100.0 * t.excess / t.propulsion if t.propulsion > 0 else 0.0
  drive = f"propulsion {t.per_km(t.propulsion):6.1f} Wh/km (steady {t.per_km(t.steady):5.1f}, speed changes +{excess:3.0f}%)"
  braking = f"regen {t.per_km(t.regen):5.1f}  friction {t.per_km(t.friction):5.1f} Wh/km"
  return f"{name:58s} {t.distance / 1000:6.1f} km  {drive}  {braking}"


def report(paths: list[str], min_km: float = 1.0) -> str:
  ledger = Ledger()
  for sample in samples(log_files(paths)):
    ledger.add(sample)
  total, by_context = summarize(ledger.finish())
  out = [line("ALL DRIVING", total), "", "By control mode:"]
  for mode in ("driver", "chill", "experimental"):
    t = Totals()
    for key, v in by_context.items():
      if key[4] == mode:
        t.distance += v.distance
        t.duration += v.duration
        t.propulsion += v.propulsion
        t.regen += v.regen
        t.friction += v.friction
        t.steady += v.steady
    if t.distance > 0:
      out.append(line(f"  {mode}", t))
  ranked = sorted(((k, v) for k, v in by_context.items() if v.distance >= min_km * 1000),
                  key=lambda kv: -(kv[1].excess + kv[1].friction) / kv[1].distance)
  out += ["", f"Contexts with the most waste per km (speed changes + friction braking), at least {min_km:g} km:"]
  for key, v in ranked[:12]:
    out.append(line("  " + " / ".join(key), v))
  return "\n".join(out)


def stored_report(root: Path | None = None, min_km: float = 1.0) -> str:
  """Totals the on-device ledger (efficiencyd) has accumulated across drives."""
  from openpilot.starpilot.efficiency.store import KEY_SEPARATOR, load_totals
  total, contexts, drives = load_totals(root)
  out = [f"{drives} drives recorded", line("ALL DRIVING", total), "",
         f"Contexts with the most waste per km (speed changes + friction braking), at least {min_km:g} km:"]
  ranked = sorted(((k, v) for k, v in contexts.items() if v.distance >= min_km * 1000),
                  key=lambda kv: -(kv[1].excess + kv[1].friction) / kv[1].distance)
  out += [line("  " + " / ".join(k.split(KEY_SEPARATOR)), v) for k, v in ranked[:12]]
  return "\n".join(out)


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument('paths', nargs='*', help='rlog files or folders')
  parser.add_argument('--stored', action='store_true', help='report the drives efficiencyd recorded on this device')
  parser.add_argument('--min-km', type=float, default=1.0)
  args = parser.parse_args()
  if args.stored or not args.paths:
    print(stored_report(min_km=args.min_km))
  else:
    print(report(args.paths, args.min_km))


if __name__ == '__main__':
  main()
