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

from openpilot.starpilot.efficiency.hybrid import CLUSTER_ADDR, POWER_FLOW_ADDR, decode_cluster, decode_power_flow
from openpilot.starpilot.efficiency.ledger import Ledger, Totals, make_sample, summarize
from openpilot.starpilot.efficiency.tank import TankState

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


def samples(files: list[Path], seed_miles: float | None = None):
  """Ledger samples from logs. seed_miles: miles since the fill-up when the first log starts (for real fuel)."""
  from openpilot.tools.lib.logreader import LogReader
  mass = cs = cc = radar = sd = pose = None
  battery = None
  flow = None
  fuel = None
  pulse_glide = None
  tank = TankState(miles_since_fill=seed_miles)
  last_t = -math.inf
  last_cs_t = None
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
      elif w == 'starpilotLongitudinalPlan':
        plan = e.starpilotLongitudinalPlan
        pulse_glide = str(plan.pulseGlidePhase) if plan.pulseGlideEnabled else None
      elif w == 'can':
        for c in e.can:
          if c.src != 0:
            continue
          if c.address == BATTERY_ADDR and len(c.dat) > 2:
            raw = c.dat[2]
            battery = raw - 256 if raw > 127 else raw
          if c.address == POWER_FLOW_ADDR:
            flow = decode_power_flow(bytes(c.dat)) or flow
          elif c.address == CLUSTER_ADDR:
            cluster = decode_cluster(bytes(c.dat))
            if cluster is not None:
              tank.observe_range(cluster.range_mi)
              fuel = tank.gallons_used(cluster.avg_mpg)
      elif w == 'carState':
        cs = e.carState
        t = e.logMonoTime / 1e9
        if last_cs_t is not None and 0.0 < t - last_cs_t < 1.0:
          tank.drive(max(cs.vEgo, 0.0) * (t - last_cs_t) / 1609.344)
        last_cs_t = t
        # Each drive's clock restarts at boot: a backwards jump is a new drive, not a sample to skip.
        if mass is None or 0.0 <= t - last_t < SAMPLE_PERIOD_S:
          continue
        last_t = t
        yield make_sample(t, mass, cs, cc, radar, sd, pose, battery, fuel_gal=fuel, power_flow=flow, pulse_glide=pulse_glide)


def line(name: str, t: Totals) -> str:
  excess = 100.0 * t.excess / t.propulsion if t.propulsion > 0 else 0.0
  drive = f"propulsion {t.per_km(t.propulsion):6.1f} Wh/km (steady {t.per_km(t.steady):5.1f}, speed changes +{excess:3.0f}%)"
  braking = f"regen {t.per_km(t.regen):5.1f}  friction {t.per_km(t.friction):5.1f} Wh/km"
  return f"{name:58s} {t.distance / 1000:6.1f} km  {drive}  {braking}"


def mpg_line(name: str, t: Totals) -> str:
  mpg = t.mpg
  flow_s = t.ev_s + t.regen_s + t.engine_s
  flow = (f"EV {t.ev_s / flow_s:4.0%}  regen {t.regen_s / flow_s:4.0%}  engine {t.engine_s / flow_s:4.0%}" if flow_s > 0 else "")
  shown = f"{mpg:5.1f} mpg" if mpg else "   -- mpg"
  return f"{name:34s} {t.distance / 1609.344:6.1f} mi  {shown}  {flow}"


def real_fuel_section(by_context: dict) -> list[str]:
  """Real MPG (dash fuel) by control mode and, for cruise modes, by speed band."""
  out = ["", "Real fuel (dash average since the fill-up):"]
  modes = sorted({k[4] for k in by_context})
  for mode in modes:
    t = Totals()
    for key, v in by_context.items():
      if key[4] == mode:
        t.add_totals(v)
    out.append(mpg_line(f"  {mode}", t))
  for band in sorted({k[0] for k in by_context}):
    parts = []
    for mode in ("pulse-glide", "chill", "experimental", "driver"):
      t = Totals()
      for key, v in by_context.items():
        if key[0] == band and key[4] == mode:
          t.add_totals(v)
      if t.mpg:
        parts.append(f"{mode} {t.mpg:.1f}")
    if parts:
      out.append(f"  {band:10s} " + "  |  ".join(parts))
  return out


def ev_cutoff_section(flow: dict) -> list[str]:
  """Where the hybrid stays electric in light-load glides (feather zone -0.15..+0.05 m/s^2), by speed."""
  rows = {}
  for key, (n, ev, regen, engine, _meter) in flow.items():
    speed, accel, phase = key.split('|')
    if phase == 'glide' and -0.15 <= float(accel) < 0.05:
      r = rows.setdefault(float(speed), [0.0, 0.0, 0.0])
      r[0] += n
      r[1] += ev + regen
      r[2] += engine
  out = ["", "Pulse and Glide feather zone: share electric by speed (where the glide can stay in EV):"]
  for speed in sorted(rows):
    n, electric, engine = rows[speed]
    if n >= 20:
      out.append(f"  {speed:3.0f}-{speed + 2:.0f} mph  {n / 20:6.1f} s  electric {electric / n:4.0%}  engine {engine / n:4.0%}")
  return out if len(out) > 2 else []


def report(paths: list[str], min_km: float = 1.0, seed_miles: float | None = None) -> str:
  ledger = Ledger()
  for sample in samples(log_files(paths), seed_miles):
    ledger.add(sample)
  total, by_context = summarize(ledger.finish())
  out = [line("ALL DRIVING", total), "", "By control mode:"]
  for mode in ("driver", "chill", "experimental", "pulse-glide"):
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
  out += real_fuel_section(by_context) + ev_cutoff_section(ledger.flow_hist)
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
  from openpilot.starpilot.efficiency.store import load_flow
  out += real_fuel_section({tuple(k.split(KEY_SEPARATOR)): v for k, v in contexts.items()}) + ev_cutoff_section(load_flow(root))
  return "\n".join(out)


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument('paths', nargs='*', help='rlog files or folders')
  parser.add_argument('--stored', action='store_true', help='report the drives efficiencyd recorded on this device')
  parser.add_argument('--min-km', type=float, default=1.0)
  parser.add_argument('--seed-miles', type=float, default=None, help='miles since the fill-up when the first log starts')
  args = parser.parse_args()
  if args.stored or not args.paths:
    print(stored_report(min_km=args.min_km))
  else:
    print(report(args.paths, args.min_km, args.seed_miles))


if __name__ == '__main__':
  main()
