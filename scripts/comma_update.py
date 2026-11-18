#!/usr/bin/env python3
"""Install a branch on the comma over SSH, by hand, one run at a time (no watcher, no automation).

  scripts/comma_update.py                      # update the comma's current target branch
  scripts/comma_update.py --branch eps-test    # switch the target branch, then update to it
  scripts/comma_update.py --status             # only show what the comma runs and whether it is parked

Steps, each printed: find the comma (home Wi-Fi first, then Tailscale); refuse unless the car is off or in Park, stopped
and disengaged; set UpdaterTargetBranch (block=True: a plain put is lost); force Offroad if onroad; ask the running
updater for its fast update (StarPilot's updater: only changed files, falls back to a full update when the OS
changes; it reboots the comma itself); wait for the comma to come back on the branch tip; then check the AOL safety
link (aolSafetyWire) and the Chestnut. Exit code 0 only when the comma runs the tip with AOL up.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time

HOSTS = ('192.168.1.157', '100.117.14.62')   # home Wi-Fi, Tailscale
SSH = ['ssh', '-o', 'ConnectTimeout=6', '-o', 'BatchMode=yes', '-o', 'ServerAliveInterval=10']
REMOTE_PY = 'cd /data/openpilot && PYTHONPATH=/data/openpilot:/data/openpilot/openpilot /usr/local/venv/bin/python -'

STATUS = r'''
import json, os, time
from openpilot.cereal import messaging as m
from openpilot.common.params import Params
p = Params()
sm = m.SubMaster(['carState', 'selfdriveState', 'deviceState', 'aolSafetyWire'])
t = time.time()
while time.time() - t < 3: sm.update(100)
cs = sm['carState']
def text(k):
  v = p.get(k)
  return v.decode() if isinstance(v, bytes) else v
print(json.dumps({
  'commit': os.popen('git -C /data/openpilot rev-parse HEAD').read().strip(),
  'branch': os.popen('git -C /data/openpilot branch --show-current').read().strip(),
  'target': text('UpdaterTargetBranch'), 'agnos': open('/VERSION').read().strip(),
  'onroad': bool(sm['deviceState'].started), 'offroad_param': open('/data/params/d/IsOffroad').read().strip() == '1',
  'car_seen': bool(sm.seen['carState']), 'gear': str(cs.gearShifter), 'speed': float(cs.vEgo),
  'engaged': bool(sm['selfdriveState'].enabled), 'aol_link': bool(sm.seen['aolSafetyWire']),
  'chestnut': (open('/data/params/d/ChestnutActive').read().strip() == '1') if os.path.exists('/data/params/d/ChestnutActive') else False,
  'updater_state': text('UpdaterState'), 'uptime': float(open('/proc/uptime').read().split()[0]),
}))
'''

PREPARE = r'''
import json, os, sys
from openpilot.common.params import Params
from openpilot.starpilot.drive_state.owner import DriveStateOwner
from openpilot.starpilot.storage import starpilot_storage_root
args = json.loads(os.environ['COMMA_UPDATE'])
p = Params()
if args['branch']:
  p.put('UpdaterTargetBranch', args['branch'], block=True)
target = p.get('UpdaterTargetBranch')
target = target.decode() if isinstance(target, bytes) else target
if args['force_offroad']:
  o = DriveStateOwner(p, starpilot_storage_root() / 'drive-state')
  s = o.snapshot()
  o.request('offroad', expected_revision=s.revision, authorized=lambda: True, override_allowed=lambda: True)
print(json.dumps({'target': target}))
'''

REQUEST = r'''
import json, os, subprocess, time
from openpilot.starpilot.software import update_control
args = json.loads(os.environ['COMMA_UPDATE'])
deadline = time.time() + 30
pid = None
while time.time() < deadline:
  out = subprocess.run(['pgrep', '-f', '[s]ystem\\.updated\\.updated'], capture_output=True, text=True).stdout.split()
  if out and open('/data/params/d/IsOffroad').read().strip() == '1':
    pid = int(out[0]); break
  time.sleep(1)
if pid is None:
  raise SystemExit('updater is not running offroad')
commit = subprocess.run(['git', '-C', '/data/openpilot', 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
update_control.send(pid, update_control.process_start(pid), 'update', branch=args['target'], commit=commit)
print(json.dumps({'pid': pid, 'from_commit': commit}))
'''


def say(msg: str) -> None:
  print(f'[{time.strftime("%H:%M:%S")}] {msg}', flush=True)


def remote(host: str, code: str, env: dict | None = None, timeout: float = 60) -> dict:
  prefix = f"export COMMA_UPDATE='{json.dumps(env)}'; " if env is not None else ''   # export: REMOTE_PY starts with cd
  r = subprocess.run(SSH + [f'comma@{host}', prefix + REMOTE_PY], input=code, capture_output=True, text=True, timeout=timeout)
  lines = [l for l in r.stdout.splitlines() if l.startswith('{')]
  if r.returncode != 0 or not lines:
    raise RuntimeError((r.stderr or r.stdout).strip().splitlines()[-1] if (r.stderr or r.stdout).strip() else f'exit {r.returncode}')
  return json.loads(lines[-1])


def reachable(host: str) -> bool:
  return subprocess.run(SSH + [f'comma@{host}', 'true'], capture_output=True, timeout=12).returncode == 0


def find_host() -> str:
  for host in HOSTS:
    if reachable(host):
      return host
  raise SystemExit('the comma is not reachable on home Wi-Fi or Tailscale')


def branch_tip(branch: str) -> str | None:
  out = subprocess.run(['git', 'ls-remote', 'origin', f'refs/heads/{branch}'], capture_output=True, text=True).stdout.split()
  return out[0] if out else None


def show(st: dict) -> str:
  where = 'off' if not st['car_seen'] else f"{st['gear']}, {st['speed'] * 2.237:.0f} mph, {'engaged' if st['engaged'] else 'disengaged'}"
  parts = [f"{st['commit'][:8]} on {st['branch']} (target {st['target']})", f"AGNOS {st['agnos']}", f"car {where}",
           'onroad' if st['onroad'] else 'offroad', f"AOL link {'up' if st['aol_link'] else 'DOWN'}",
           f"Chestnut {'on' if st['chestnut'] else 'off'}"]
  return ', '.join(parts)


def push_stop_line() -> None:
  """A stop-line model the PC staged goes along (shadow only, the comma just logs with it). Never blocks the install."""
  try:
    import datetime as dt
    import fcntl
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'openpilot' / 'starpilot' / 'stop_line'))
    import nightly
    if not (nightly.RUNS / 'staged' / 'stop_line.onnx').exists():
      return
    with open(nightly.RUNS / 'nightly.lock', 'w') as lock:
      try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
      except BlockingIOError:
        say('stop-line model: the nightly run holds it, not pushed this time')
        return
      sent = nightly.deploy(nightly.Night(f'{dt.date.today():%Y-%m-%d}'), parked=True)
    say('stop-line model: pushed to the comma' if sent else 'stop-line model: NOT pushed (see the nightly log)')
  except Exception as e:
    say(f'stop-line model: not pushed ({e})')


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  ap.add_argument('--branch', help='switch UpdaterTargetBranch to this branch first')
  ap.add_argument('--status', action='store_true', help='only show the comma state')
  ap.add_argument('--host', help='skip discovery and use this address')
  ap.add_argument('--timeout', type=float, default=1200, help='seconds to wait for the install (default 1200)')
  a = ap.parse_args()

  host = a.host or find_host()
  st = remote(host, STATUS)
  say(f'comma at {host}: {show(st)}')
  if a.status:
    return 0

  parked = not st['car_seen'] or (st['gear'] == 'park' and abs(st['speed']) < 0.1 and not st['engaged'])
  if not parked:
    say('refusing: put the car in Park (stopped, disengaged) or turn it off first')
    return 2
  push_stop_line()
  target = a.branch or st['target']
  tip = branch_tip(target)
  if tip is None:
    say(f'refusing: branch {target!r} is not on origin')
    return 2
  if tip == st['commit']:
    say(f'already on the tip of {target} ({tip[:8]}); nothing to install')
    return 0 if st['aol_link'] or not st['car_seen'] else 1

  prep = remote(host, PREPARE, {'branch': a.branch, 'force_offroad': not st['offroad_param']})
  say(f"target branch {prep['target']}; {'forced Offroad' if not st['offroad_param'] else 'already offroad'}")
  req = remote(host, REQUEST, {'target': prep['target']})
  say(f"fast update requested from the updater (pid {req['pid']}): {req['from_commit'][:8]} -> {tip[:8]}")

  start, last_state, rebooting = time.monotonic(), None, False
  while time.monotonic() - start < a.timeout:
    time.sleep(8)
    try:
      now = remote(host, STATUS, timeout=40)
    except (RuntimeError, subprocess.TimeoutExpired):
      if not rebooting:
        say('comma went down: installing / rebooting')
        rebooting = True
      continue
    if now['updater_state'] != last_state:
      say(f"updater: {now['updater_state']}")
      last_state = now['updater_state']
    # The fast updater puts the new files in place first and reboots after, so the tip alone is not done: it counts
    # only once the comma has rebooted since the request (uptime shorter than the time since then).
    if now['commit'] == tip and now['uptime'] < time.monotonic() - start:
      if now['uptime'] < 60:
        time.sleep(30)   # let pandad, the model and AOL come up
        now = remote(host, STATUS)
      say(f'installed: {show(now)}')
      if now['car_seen'] and not now['aol_link']:
        say('WARNING: AOL safety link is down; do not drive engaged until this is checked')
        return 1
      return 0
    if rebooting and now['uptime'] > 300 and now['commit'] != tip:
      say(f"comma came back on {now['commit'][:8]}, not {tip[:8]}: the install did not take")
      return 1
  say('timed out waiting for the install')
  return 1


if __name__ == '__main__':
  sys.exit(main())
