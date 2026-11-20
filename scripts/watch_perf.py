#!/usr/bin/env python3
"""Watch the comma's CPU load, clocks and heat during a drive (read-only), from the PC.

  scripts/watch_perf.py [--host comma@100.117.14.62] [--minutes 30] [--every 5] [--log watch.jsonl] [--flip-boost 5]

Every few seconds it prints one row: engaged / speed, CPU boost mode and the clocks the two clusters actually run at,
max temperature, thermal status, fan, and the CPU use of the UI main thread and of card, controlsd, selfdrived,
plannerd, radard and modeld (percent of one core, from /proc). At the start it checks that the UI's native
model-path projection is in the installed build. Ctrl-C (or --minutes) ends it with averages grouped by
engaged/parked and boost on/off, which is how one combined update can still be read change by change:

  - boost: compare boost-on with boost-off windows of the same drive. --flip-boost N alternates
    /data/starpilot/cpu_boost.json between on and off every N minutes (atomic write, every flip logged, the file put
    back as it was at the end); without it, flip "mode" by hand. Boost off also restores the stock fan curve;
  - UI and process fixes: compare boost-off windows with the stock-clock baseline of 2026-11-20
    (UI main 44 % engaged / 39 % parked; card 68, selfdrived 58, controlsd 45-55, plannerd 22-31, radard 6-12).

Without --flip-boost nothing on the comma is changed: the comma side only reads /proc, /sys, the boost settings file
and messages, at nice 19.
"""
import argparse
import json
import subprocess
import sys
import time
from collections import defaultdict

REMOTE = r'''
import glob, json, os, sys, time
every = float(sys.argv[1])
NAMES = {"ui": "openpilot.selfdrive.ui.ui", "card": "openpilot.selfdrive.car.card",
         "controlsd": "openpilot.selfdrive.controls.controlsd", "selfdrived": "openpilot.selfdrive.selfdrived.selfdrived",
         "plannerd": "openpilot.selfdrive.controls.plannerd", "radard": "openpilot.selfdrive.controls.radard",
         "modeld": "openpilot.selfdrive.modeld.modeld"}
HZ = os.sysconf("SC_CLK_TCK")

def pids():
  found = {}
  for d in glob.glob("/proc/[0-9]*"):
    try:
      cmd = open(d + "/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="replace")
    except OSError:
      continue
    for name, needle in NAMES.items():
      if needle in cmd and "watch" not in cmd:
        found.setdefault(name, int(d[6:]))
  return found

def ticks(path):
  try:
    s = open(path).read()
    f = s[s.rindex(")") + 2:].split()
    return int(f[11]) + int(f[12])
  except (OSError, ValueError, IndexError):
    return None

def read(path):
  try:
    return open(path).read().strip()
  except OSError:
    return None

def boost_mode():
  try:
    from openpilot.starpilot.storage import starpilot_storage_root
    path = starpilot_storage_root() / "cpu_boost.json"
  except Exception:
    return "n/a"
  if not os.path.exists("/data/openpilot/openpilot/starpilot/power/cpu_boost.py"):
    return "n/a"
  try:
    return "off" if json.loads(path.read_bytes()[:4096]).get("mode", "on") == "off" else "on"
  except FileNotFoundError:
    return "on"
  except (OSError, ValueError, AttributeError):
    return "on?"

try:
  from openpilot.starpilot.ui.native import _raylib_extra
  native = hasattr(_raylib_extra, "project_polygons")
except Exception as e:
  native = f"import failed: {e!r}"
print(json.dumps({"start": True, "native_projection": native,
                  "commit": os.popen("cd /data/openpilot && git log -1 --format=%h 2>/dev/null").read().strip()}), flush=True)

import cereal.messaging as messaging
sm = messaging.SubMaster(["selfdriveState", "carState", "deviceState"])
procs = pids()
def snap():
  out = {name: ticks(f"/proc/{pid}/stat") for name, pid in procs.items()}
  if "ui" in procs:
    out["ui_main"] = ticks(f"/proc/{procs['ui']}/task/{procs['ui']}/stat")
  return out
prev, pt = snap(), time.monotonic()
while True:
  end = time.monotonic() + every
  while time.monotonic() < end:
    sm.update(200)
  cur, ct = snap(), time.monotonic()
  if any(v is None for v in cur.values()):
    procs = pids(); prev, pt = snap(), time.monotonic()
    continue
  cpu = {k: round((cur[k] - prev[k]) / HZ / (ct - pt) * 100, 1) for k in cur if prev.get(k) is not None}
  prev, pt = cur, ct
  ds = sm["deviceState"]
  print(json.dumps({"t": round(time.time(), 1), "en": bool(sm["selfdriveState"].enabled),
                    "mph": round(sm["carState"].vEgo * 2.237, 1), "boost": boost_mode(),
                    "little_mhz": int(read("/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq") or 0) // 1000,
                    "big_mhz": int(read("/sys/devices/system/cpu/cpufreq/policy4/scaling_cur_freq") or 0) // 1000,
                    "temp": round(float(ds.maxTempC), 1), "thermal": str(ds.thermalStatus),
                    "fan": int(ds.fanSpeedPercentDesired), "cpu": cpu}), flush=True)
'''

BOOST_FILE = "/data/starpilot/cpu_boost.json"


def ssh(host: str, command: str, data: str | None = None) -> subprocess.CompletedProcess:
  return subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", host, command], input=data, capture_output=True, text=True, timeout=30)


def set_boost(host: str, mode: str) -> bool:
  """Atomic write: a torn file would read as "on" and blur the window."""
  doc = json.dumps({"mode": mode}) + "\n"
  r = ssh(host, f"cat > {BOOST_FILE}.tmp && mv -f {BOOST_FILE}.tmp {BOOST_FILE}", doc)
  return r.returncode == 0


COLUMNS = ("ui_main", "card", "controlsd", "selfdrived", "plannerd", "radard", "modeld")


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--host", default="comma@100.117.14.62")
  ap.add_argument("--minutes", type=float, default=30)
  ap.add_argument("--every", type=float, default=5)
  ap.add_argument("--log", default=f"watch_perf_{time.strftime('%Y%m%d_%H%M')}.jsonl")
  ap.add_argument("--flip-boost", type=float, metavar="MINUTES",
                  help="alternate cpu_boost.json on/off every MINUTES (changes the comma; restored at the end)")
  args = ap.parse_args()

  original = None
  if args.flip_boost:
    r = ssh(args.host, f"cat {BOOST_FILE} 2>/dev/null || echo __MISSING__")
    if r.returncode != 0:
      sys.exit(f"cannot reach {args.host}: {r.stderr.strip()}")
    original = r.stdout
    print(f"boost flipping every {args.flip_boost:g} min; {BOOST_FILE} is restored at the end")

  cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "-o", "ServerAliveInterval=10", args.host,
         "cd /data/openpilot && PYTHONPATH=/data/openpilot:/data/openpilot/openpilot "
         f"exec nice -n 19 /usr/local/venv/bin/python3 - {args.every}"]
  proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
  proc.stdin.write(REMOTE)
  proc.stdin.close()
  groups, deadline = defaultdict(list), time.monotonic() + args.minutes * 60
  next_flip, flip_mode = time.monotonic(), "on"
  print(f"logging to {args.log}; Ctrl-C to stop")
  header = (f"{'time':8s} {'state':8s} {'mph':>5s} {'boost':>5s} {'lit/big MHz':>11s} {'temp':>5s} {'fan':>4s}  "
            + " ".join(f"{c[:10]:>10s}" for c in COLUMNS))
  try:
    with open(args.log, "a") as log:
      for n, line in enumerate(proc.stdout):
        row = json.loads(line)
        log.write(line)
        log.flush()
        if row.get("start"):
          ok = row["native_projection"] is True
          print(f"comma build {row['commit']}: native model-path projection "
                f"{'LOADED' if ok else 'NOT loaded (UI uses the numpy fallback): ' + str(row['native_projection'])}")
          print(header)
          continue
        state = "engaged" if row["en"] else ("parked" if row["mph"] < 0.5 else "manual")
        groups[(state, row["boost"])].append(row)
        print(f"{time.strftime('%H:%M:%S', time.localtime(row['t']))} {state:8s} {row['mph']:5.1f} {row['boost']:>5s} "
              f"{row['little_mhz']:>5d}/{row['big_mhz']:<5d} {row['temp']:5.1f} {row['fan']:3d}%  "
              + " ".join(f"{row['cpu'].get(c, float('nan')):9.1f}%" for c in COLUMNS), flush=True)
        if n and n % 20 == 0:
          print(header)
        if args.flip_boost and time.monotonic() >= next_flip:
          ok = set_boost(args.host, flip_mode)
          event = {"t": round(time.time(), 1), "boost_set": flip_mode, "ok": ok}
          log.write(json.dumps(event) + "\n")
          log.flush()
          print(f"--- {time.strftime('%H:%M:%S')} boost set {flip_mode}{'' if ok else ' FAILED'} (takes effect within 5 s)")
          next_flip, flip_mode = time.monotonic() + args.flip_boost * 60, "off" if flip_mode == "on" else "on"
        if time.monotonic() > deadline:
          break
  except KeyboardInterrupt:
    pass
  finally:
    proc.terminate()
    if original is not None:
      if original.strip() == "__MISSING__":
        r = ssh(args.host, f"rm -f {BOOST_FILE}")
      else:
        r = ssh(args.host, f"cat > {BOOST_FILE}.tmp && mv -f {BOOST_FILE}.tmp {BOOST_FILE}", original)
      print(f"boost file restored: {'ok' if r.returncode == 0 else 'FAILED, check ' + BOOST_FILE}")

  print("\naverages (percent of one core)")
  print(f"{'group':20s} {'min':>5s} {'lit/big MHz':>11s} {'temp max':>8s} {'fan':>4s}  " + " ".join(f"{c[:10]:>10s}" for c in COLUMNS))
  for (state, boost), rows in sorted(groups.items()):
    mean = lambda f: sum(f(r) for r in rows) / len(rows)
    print(f"{state + ' / boost ' + boost:20s} {len(rows) * args.every / 60:5.1f} "
          f"{mean(lambda r: r['little_mhz']):5.0f}/{mean(lambda r: r['big_mhz']):<5.0f} "
          f"{max(r['temp'] for r in rows):8.1f} {mean(lambda r: r['fan']):3.0f}%  "
          + " ".join(f"{mean(lambda r, c=c: r['cpu'].get(c, 0.0)):9.1f}%" for c in COLUMNS))
  print("baseline 2026-11-20, stock clocks: ui_main 44 engaged / 39 parked; card 68, selfdrived 58, controlsd 45-55, "
        "plannerd 22-31, radard 6-12")
  return 0


if __name__ == "__main__":
  sys.exit(main())
