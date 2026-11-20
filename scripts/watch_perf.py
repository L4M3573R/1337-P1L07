#!/usr/bin/env python3
"""Watch the comma's CPU load, clocks, heat and model timing during a drive, from the PC.

  scripts/watch_perf.py [--host comma@100.117.14.62] [--minutes 30] [--every 5] [--log watch.jsonl]
                        [--rotate boost|irq|ui] [--window 5]

Every few seconds it prints one row: engaged / speed; the three test switches in /data/starpilot/cpu_boost.json
(boost mode, chestnut_irq_core, ui_cores); the clocks both clusters actually run at; max temperature and fan;
model execution time (mean / max over the row); the Chestnut USB interrupt rate and the core taking most of it;
the UI main thread's CPU use and its time spent waiting for a free core; and the CPU use of card, controlsd,
selfdrived, plannerd, radard and modeld (percent of one core). At the start it checks that the UI's native
model-path projection is in the installed build. Ctrl-C (or --minutes) ends it with averages per switch setting.

--rotate SWITCH cycles ONE switch every --window minutes, editing only that key (atomic write, every change
logged, the file restored byte for byte at the end), so one update can still be read change by change:
  boost: mode on <-> off (off = stock clocks and stock fan)
  irq:   chestnut_irq_core unset (stock) <-> 6
  ui:    ui_cores unset (stock 0,1,2,7) -> [0,1,2,6] -> [4,5,6]
Rotate one switch per stretch of driving; city and highway load differ a lot, so compare windows of similar driving.
Baseline 2026-11-20 (stock clocks, before the UI changes): UI main 44 % engaged / 39 % parked, waiting for a core
31 %; card 68, selfdrived 58, controlsd 45-55, plannerd 22-31, radard 6-12; model 35.5 ms p50.

Without --rotate nothing on the comma is changed: the comma side only reads /proc, /sys, that file and messages,
at nice 19.
"""
import argparse
import json
import subprocess
import sys
import time
from collections import defaultdict

BOOST_FILE = "/data/starpilot/cpu_boost.json"

REMOTE = r'''
import glob, json, os, sys, time
every = float(sys.argv[1])
NAMES = {"ui": "openpilot.selfdrive.ui.ui", "card": "openpilot.selfdrive.car.card",
         "controlsd": "openpilot.selfdrive.controls.controlsd", "selfdrived": "openpilot.selfdrive.selfdrived.selfdrived",
         "plannerd": "openpilot.selfdrive.controls.plannerd", "radard": "openpilot.selfdrive.controls.radard",
         "modeld": "openpilot.selfdrive.modeld.modeld"}
HZ = os.sysconf("SC_CLK_TCK")
BOOST_FILE = "/data/starpilot/cpu_boost.json"

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

def schedstat(path):
  try:
    run, wait, _ = open(path).read().split()
    return int(run), int(wait)
  except (OSError, ValueError):
    return None

def read(path):
  try:
    return open(path).read().strip()
  except OSError:
    return None

def usb_irq():
  try:
    for line in open("/proc/interrupts"):
      if "xhci-hcd:usb3" in line:
        parts = line.split()
        return [int(x) for x in parts[1:1 + (os.cpu_count() or 8)]]
  except (OSError, ValueError):
    pass
  return None

def switches():
  if not os.path.exists("/data/openpilot/openpilot/starpilot/power/cpu_boost.py"):
    return {"boost": "n/a", "irq": "n/a", "ui": "n/a"}
  try:
    doc = json.loads(open(BOOST_FILE, "rb").read()[:4096])
    doc = doc if isinstance(doc, dict) else {}
  except FileNotFoundError:
    doc = {}
  except (OSError, ValueError):
    return {"boost": "on?", "irq": "?", "ui": "?"}
  ui = doc.get("ui_cores")
  irq = doc.get("chestnut_irq_core")
  return {"boost": "off" if doc.get("mode", "on") == "off" else "on",
          "irq": "stock" if irq is None else str(irq),
          "ui": "stock" if not isinstance(ui, list) else "".join(str(c) for c in ui)}

try:
  from openpilot.starpilot.ui.native import _raylib_extra
  native = hasattr(_raylib_extra, "project_polygons")
except Exception as e:
  native = f"import failed: {e!r}"
print(json.dumps({"start": True, "native_projection": native,
                  "commit": os.popen("cd /data/openpilot && git log -1 --format=%h 2>/dev/null").read().strip()}), flush=True)

import cereal.messaging as messaging
sm = messaging.SubMaster(["selfdriveState", "carState", "deviceState", "modelV2"])
procs = pids()
def snap():
  out = {name: ticks(f"/proc/{pid}/stat") for name, pid in procs.items()}
  if "ui" in procs:
    out["ui_main"] = ticks(f"/proc/{procs['ui']}/task/{procs['ui']}/stat")
  return out
def ui_sched():
  return schedstat(f"/proc/{procs['ui']}/task/{procs['ui']}/schedstat") if "ui" in procs else None
prev, pt, prev_sched, prev_irq = snap(), time.monotonic(), ui_sched(), usb_irq()
while True:
  end = time.monotonic() + every
  model_ms = []
  while time.monotonic() < end:
    sm.update(200)
    if sm.updated["modelV2"]:
      model_ms.append(sm["modelV2"].modelExecutionTime * 1000)
  cur, ct, cur_sched, cur_irq = snap(), time.monotonic(), ui_sched(), usb_irq()
  if any(v is None for v in cur.values()):
    procs = pids(); prev, pt, prev_sched, prev_irq = snap(), time.monotonic(), ui_sched(), usb_irq()
    continue
  dt = ct - pt
  cpu = {k: round((cur[k] - prev[k]) / HZ / dt * 100, 1) for k in cur if prev.get(k) is not None}
  ui_wait = round((cur_sched[1] - prev_sched[1]) / 1e9 / dt * 100, 1) if cur_sched and prev_sched else None
  irq_rate, irq_cpu = None, None
  if cur_irq and prev_irq and len(cur_irq) == len(prev_irq):
    delta = [b - a for a, b in zip(prev_irq, cur_irq)]
    irq_rate, irq_cpu = round(sum(delta) / dt), max(range(len(delta)), key=lambda i: delta[i])
  prev, pt, prev_sched, prev_irq = cur, ct, cur_sched, cur_irq
  ds = sm["deviceState"]
  print(json.dumps({"t": round(time.time(), 1), "en": bool(sm["selfdriveState"].enabled),
                    "mph": round(sm["carState"].vEgo * 2.237, 1), **switches(),
                    "little_mhz": int(read("/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq") or 0) // 1000,
                    "big_mhz": int(read("/sys/devices/system/cpu/cpufreq/policy4/scaling_cur_freq") or 0) // 1000,
                    "temp": round(float(ds.maxTempC), 1), "fan": int(ds.fanSpeedPercentDesired),
                    "model_mean": round(sum(model_ms) / len(model_ms), 2) if model_ms else None,
                    "model_max": round(max(model_ms), 2) if model_ms else None,
                    "ui_wait": ui_wait, "usb_irq_rate": irq_rate, "usb_irq_cpu": irq_cpu, "cpu": cpu}), flush=True)
'''

SET_KEY = r'''
import json, os, sys
path, key, raw = sys.argv[1], sys.argv[2], sys.argv[3]
try:
  doc = json.loads(open(path, "rb").read()[:4096])
  doc = doc if isinstance(doc, dict) else {}
except (OSError, ValueError):
  doc = {}
value = json.loads(raw)
if value is None:
  doc.pop(key, None)
else:
  doc[key] = value
tmp = path + ".tmp"
with open(tmp, "w") as f:
  f.write(json.dumps(doc) + "\n")
os.replace(tmp, path)
print(json.dumps(doc))
'''

ROTATIONS = {
  "boost": ("mode", ["on", "off"]),
  "irq": ("chestnut_irq_core", [None, 6]),
  "ui": ("ui_cores", [None, [0, 1, 2, 6], [4, 5, 6]]),
}
COLUMNS = ("ui_main", "card", "controlsd", "selfdrived", "plannerd", "radard", "modeld")


def ssh(host: str, command: list[str] | str, data: str | None = None) -> subprocess.CompletedProcess:
  cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", host]
  cmd += command if isinstance(command, list) else [command]
  return subprocess.run(cmd, input=data, capture_output=True, text=True, timeout=30)


def set_key(host: str, key: str, value) -> tuple[bool, str]:
  """Change one key, keep the rest of the file; atomic replace so a reader never sees a torn file."""
  r = ssh(host, ["python3", "-", BOOST_FILE, key, f"'{json.dumps(value)}'"], SET_KEY)
  return r.returncode == 0, (r.stdout or r.stderr).strip()


def fmt(v, spec: str, width: int) -> str:
  return format(v, spec) if isinstance(v, (int, float)) else str(v if v is not None else "-").rjust(width)


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--host", default="comma@100.117.14.62")
  ap.add_argument("--minutes", type=float, default=30)
  ap.add_argument("--every", type=float, default=5)
  ap.add_argument("--log", default=f"watch_perf_{time.strftime('%Y%m%d_%H%M')}.jsonl")
  ap.add_argument("--rotate", choices=sorted(ROTATIONS), help="cycle ONE switch (changes the comma; restored at the end)")
  ap.add_argument("--window", type=float, default=5, help="minutes per setting with --rotate")
  args = ap.parse_args()

  original = None
  if args.rotate:
    r = ssh(args.host, f"cat {BOOST_FILE} 2>/dev/null || echo __MISSING__")
    if r.returncode != 0:
      sys.exit(f"cannot reach {args.host}: {r.stderr.strip()}")
    original = r.stdout
    key, values = ROTATIONS[args.rotate]
    print(f"rotating {key} through {values} every {args.window:g} min; {BOOST_FILE} is restored at the end")

  cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "-o", "ServerAliveInterval=10", args.host,
         "cd /data/openpilot && PYTHONPATH=/data/openpilot:/data/openpilot/openpilot "
         f"exec nice -n 19 /usr/local/venv/bin/python3 - {args.every}"]
  proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
  proc.stdin.write(REMOTE)
  proc.stdin.close()
  groups, deadline = defaultdict(list), time.monotonic() + args.minutes * 60
  next_change, step = time.monotonic(), 0
  print(f"logging to {args.log}; Ctrl-C to stop")
  header = (f"{'time':8s} {'state':8s} {'mph':>5s} {'boost':>5s} {'irq':>5s} {'ui':>6s} {'lit/big MHz':>11s} {'temp':>5s} "
            f"{'fan':>4s} {'model ms':>11s} {'usbirq/s@cpu':>12s} {'ui wait':>7s}  " + " ".join(f"{c[:10]:>10s}" for c in COLUMNS))
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
        groups[(state, row["boost"], row["irq"], row["ui"])].append(row)
        model = f"{fmt(row['model_mean'], '5.1f', 5)}/{fmt(row['model_max'], '<5.1f', 5)}"
        irq = f"{fmt(row['usb_irq_rate'], '6d', 6)}@{fmt(row['usb_irq_cpu'], 'd', 1)}"
        print(f"{time.strftime('%H:%M:%S', time.localtime(row['t']))} {state:8s} {row['mph']:5.1f} {row['boost']:>5s} "
              f"{row['irq']:>5s} {row['ui']:>6s} {row['little_mhz']:>5d}/{row['big_mhz']:<5d} {row['temp']:5.1f} "
              f"{row['fan']:3d}% {model:>11s} {irq:>12s} {fmt(row['ui_wait'], '6.1f', 6)}%  "
              + " ".join(f"{row['cpu'].get(c, float('nan')):9.1f}%" for c in COLUMNS), flush=True)
        if n and n % 20 == 0:
          print(header)
        if args.rotate and time.monotonic() >= next_change:
          key, values = ROTATIONS[args.rotate]
          value = values[step % len(values)]
          ok, doc = set_key(args.host, key, value)
          log.write(json.dumps({"t": round(time.time(), 1), "set": key, "value": value, "ok": ok, "doc": doc}) + "\n")
          log.flush()
          print(f"--- {time.strftime('%H:%M:%S')} {key} = {json.dumps(value)}{'' if ok else ' FAILED: ' + doc} "
                f"(takes effect within 5 s)")
          next_change, step = time.monotonic() + args.window * 60, step + 1
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
      print(f"switch file restored: {'ok' if r.returncode == 0 else 'FAILED, check ' + BOOST_FILE}")

  print("\naverages per setting (CPU in percent of one core)")
  print(f"{'state / boost / irq / ui':32s} {'min':>5s} {'temp max':>8s} {'model ms':>11s} {'usbirq/s':>8s} {'ui wait':>7s}  "
        + " ".join(f"{c[:10]:>10s}" for c in COLUMNS))
  for key, rows in sorted(groups.items()):
    def mean(field, rows=rows):
      vals = [r[field] for r in rows if r.get(field) is not None]
      return sum(vals) / len(vals) if vals else float('nan')
    model_max = max((r["model_max"] for r in rows if r.get("model_max") is not None), default=float('nan'))
    print(f"{' / '.join(key):32s} {len(rows) * args.every / 60:5.1f} {max(r['temp'] for r in rows):8.1f} "
          f"{mean('model_mean'):5.1f}/{model_max:<5.1f} {mean('usb_irq_rate'):8.0f} {mean('ui_wait'):6.1f}%  "
          + " ".join(f"{sum(r['cpu'].get(c, 0.0) for r in rows) / len(rows):9.1f}%" for c in COLUMNS))
  print("baseline 2026-11-20 (stock clocks, before the UI changes): ui_main 44 engaged / 39 parked, ui wait 31 %; "
        "card 68, selfdrived 58, controlsd 45-55, plannerd 22-31, radard 6-12; model 35.5 ms p50")
  return 0


if __name__ == "__main__":
  sys.exit(main())
