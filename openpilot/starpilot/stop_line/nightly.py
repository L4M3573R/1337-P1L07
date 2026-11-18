#!/usr/bin/env python3
"""The stop-line finder's 4 AM run on the PC: new drives in, a better model out to the comma (PC side).

  .venv/bin/python openpilot/starpilot/stop_line/nightly.py            # the full run (systemd timer, 04:00)
  .venv/bin/python openpilot/starpilot/stop_line/nightly.py --deploy   # only push a staged model (every 5 min)

1. Waits for the PC to be free (no game, MIN_FREE_GB of RAM, GPU idle), up to WAIT_FREE_S, else skips the night.
2. extract.py, lines.py, autolabel.py, frames.py on what the puller brought to DR4L (all incremental).
3. Cross-validates the corrector by drive and scores it with report.gate_metrics.
4. Gate: chestnut-first-with-ours placement must not drop and ours' false alarms must not rise (beyond TOLERANCE)
   against the deployed model's scores. Only then is the final model trained, checked in cv2 and staged.
5. A staged model goes to the comma when it is reachable and offroad: copied beside the live one, run once there
   with the comma's own cv2, then swapped in (stop_lined reloads it within 10 s); the previous model is kept.
Each night writes <runs>/nightly/<date>/ (log, cv, final, gate.json); <runs>/deployed/ holds the live model's scores.
Driving is never touched: stop_lined is shadow only.
"""
from __future__ import annotations

import argparse
import datetime as dt
import fcntl
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
DATA = Path('/mnt/Projects/stopline-data')
RUNS = Path('/mnt/Projects/stopline-train')
REPO_PY = REPO / '.venv' / 'bin' / 'python'
TRAIN_PY = RUNS / '.venv' / 'bin' / 'python'
PYTHONPATH = ':'.join(str(REPO / p) for p in ('', 'openpilot', 'opendbc_repo', 'msgq_repo'))
COMMA_HOSTS = ('comma@192.168.1.157', 'comma@100.117.14.62')  # home Wi-Fi first, then Tailscale
COMMA_MODEL = '/data/starpilot/stop_line/stop_line.onnx'
COMMA_PY = 'cd /data/openpilot && PYTHONPATH=/data/openpilot:/data/openpilot/openpilot nice -n 19 /usr/local/venv/bin/python3'
MIN_FREE_GB = 12.0
MAX_GPU_BUSY = 30
GAME_NAMES = frozenset({'reaper', 'gamescope', 'gamescope-wl', 'steamlaunch'})  # Steam runs each game under reaper
WAIT_FREE_S = 2 * 3600
TOLERANCE = 0.01
STEP = Path(__file__).parent


class Night:
  def __init__(self, day: str):
    self.dir = RUNS / 'nightly' / day
    self.dir.mkdir(parents=True, exist_ok=True)
    self.log_file = open(self.dir / 'log.txt', 'a')

  def log(self, msg: str) -> None:
    line = f"{dt.datetime.now():%H:%M:%S} {msg}"
    print(line, flush=True)
    self.log_file.write(line + '\n')
    self.log_file.flush()

  def run(self, name: str, args: list[str], py: Path = REPO_PY) -> None:
    self.log(f'{name}: start')
    t0 = time.monotonic()
    with open(self.dir / f'{name}.out', 'w') as out:
      done = subprocess.run(['nice', '-n', '10', str(py), *args], cwd=REPO, stdout=out, stderr=subprocess.STDOUT,
                            env={'PYTHONPATH': PYTHONPATH, 'PATH': '/usr/bin:/bin:/usr/local/bin', 'HOME': str(Path.home())})
    if done.returncode != 0:
      raise RuntimeError(f'{name} failed (exit {done.returncode}), see {self.dir / f"{name}.out"}')
    self.log(f'{name}: done in {(time.monotonic() - t0) / 60:.1f} min')


def pc_busy() -> str | None:
  """Why the PC is not free for training, or None."""
  mem = dict(line.split(':', 1) for line in open('/proc/meminfo'))
  free_gb = int(mem['MemAvailable'].split()[0]) / 1e6
  if free_gb < MIN_FREE_GB:
    return f'only {free_gb:.0f} GB RAM free'
  try:
    gpu = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits'],
                         capture_output=True, text=True, timeout=20).stdout.split()
    if gpu and int(gpu[0]) > MAX_GPU_BUSY:
      return f'GPU {gpu[0]}% busy'
  except (OSError, ValueError, subprocess.TimeoutExpired):
    return 'GPU not answering'
  procs = subprocess.run(['ps', '-eo', 'comm='], capture_output=True, text=True).stdout.lower().split()
  # not every .exe: long-lived Wine tools (Vinegar's Roblox Studio helper) are not games; other games show in RAM/GPU
  game = next((p for p in procs if p in GAME_NAMES), None)
  return f'{game} is running' if game else None


def ssh(host: str, cmd: str, timeout: int = 60, stdin: str | None = None) -> subprocess.CompletedProcess:
  try:
    return subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', host, cmd], input=stdin,
                          capture_output=True, text=True, timeout=timeout)
  except subprocess.TimeoutExpired:
    return subprocess.CompletedProcess([], 255, '', 'timed out')


VERIFY = """
import cv2, numpy as np, sys
net = cv2.dnn.readNetFromONNX(sys.argv[1])
net.setInput(np.zeros((1, 3, 256, 448), np.float32), 'image')
try:
  net.setInput(np.array([[1.0, 0.25]], np.float32), 'chestnut')
except Exception:
  pass
out = net.forward().reshape(-1)
assert out.size >= 3 and np.all(np.isfinite(out[:3])), out
print('ok', cv2.__version__, out[:3].tolist())
"""


def deploy(night: Night, quiet: bool = False, parked: bool = False) -> bool:
  """Push a staged model. `parked`: the caller (scripts/comma_update.py) already checked the car is off or in Park.

  The comma has no power with the car off and is onroad whenever it is on, so the offroad wait alone never came true
  (2026-11-15..17: three models staged, none sent)."""
  staged = RUNS / 'staged'
  if not (staged / 'stop_line.onnx').exists():
    return False
  host = next((h for h in COMMA_HOSTS if ssh(h, 'true', 20).returncode == 0), None)
  if host is None:
    if not quiet:  # the 5-minute retry only logs when something happens
      night.log('deploy: comma not reachable, will retry')
    return False
  offroad = ssh(host, 'cat /data/params/d/IsOffroad 2>/dev/null').stdout.strip()
  if offroad != '1' and not parked:
    night.log('deploy: comma is onroad, waiting until it is parked')
    return False
  new = COMMA_MODEL + '.new'
  if ssh(host, f'mkdir -p {Path(COMMA_MODEL).parent}').returncode != 0 or subprocess.run(
      ['scp', '-q', '-o', 'BatchMode=yes', str(staged / 'stop_line.onnx'), f'{host}:{new}'], timeout=300).returncode != 0:
    night.log('deploy: copy failed, will retry')
    return False
  check = ssh(host, f'{COMMA_PY} - {new}', 180, stdin=VERIFY)
  if check.returncode != 0 or not check.stdout.startswith('ok'):
    night.log(f'deploy: the comma could not run it, not swapped in: {(check.stdout + check.stderr).strip()[-300:]}')
    ssh(host, f'rm -f {new}')
    shutil.move(str(staged), str(RUNS / f'rejected-{dt.datetime.now():%Y%m%d-%H%M}'))
    return False
  ssh(host, f'[ -f {COMMA_MODEL} ] && cp {COMMA_MODEL} {COMMA_MODEL}.previous; mv {new} {COMMA_MODEL}')
  deployed = RUNS / 'deployed'
  if deployed.exists():
    shutil.rmtree(RUNS / 'deployed.previous', ignore_errors=True)
    deployed.rename(RUNS / 'deployed.previous')
  staged.rename(deployed)
  night.log(f'deploy: live on the comma ({host}), its cv2 says {check.stdout.strip()}')
  return True


def nightly(night: Night) -> None:
  waited = 0
  while (why := pc_busy()) is not None:
    if waited >= WAIT_FREE_S:
      night.log(f'skipped: PC not free for {WAIT_FREE_S // 3600} h ({why})')
      return
    night.log(f'waiting: {why}')
    time.sleep(600)
    waited += 600
  deploy(night)  # a model staged earlier goes first, if the comma is home
  night.run('extract', ['-W', 'ignore', '-m', 'openpilot.starpilot.stop_line.extract', str(DATA), '--jobs', '10'])
  night.run('lines', [str(STEP / 'lines.py'), str(DATA), '--jobs', '8'], TRAIN_PY)
  night.run('autolabel', ['-m', 'openpilot.starpilot.stop_line.autolabel', str(DATA)])
  night.run('frames', ['-m', 'openpilot.starpilot.stop_line.frames', str(DATA), '--jobs', '10'])
  cv = night.dir / 'cv'
  shutil.rmtree(cv, ignore_errors=True)
  night.run('cv', [str(STEP / 'train.py'), str(DATA), str(cv), '--folds', '4', '--epochs', '6', '--workers', '6',
                   '--split', 'drive', '--corrector'], TRAIN_PY)
  sys.path.insert(0, str(STEP))
  from report import gate_metrics
  new = gate_metrics(DATA, cv)
  live_path = RUNS / 'deployed' / 'metrics.json'
  live = json.loads(live_path.read_text()) if live_path.exists() else None
  passed = live is None or (new['hybrid_within5'] >= live['hybrid_within5'] - TOLERANCE and
                            new['false_alarms'] <= live['false_alarms'] + TOLERANCE)
  (night.dir / 'gate.json').write_text(json.dumps({'new': new, 'live': live, 'passed': passed}, indent=1))
  night.log(f"gate: placed {new['hybrid_within5']:.0%} (live {live['hybrid_within5']:.0%}), false alarms "
            f"{new['false_alarms']:.1%} (live {live['false_alarms']:.1%})" if live else
            f"gate: placed {new['hybrid_within5']:.0%}, false alarms {new['false_alarms']:.1%} (nothing live yet)")
  if not passed:
    night.log('gate: not better, the live model stays')
    return
  final = night.dir / 'final'
  shutil.rmtree(final, ignore_errors=True)
  night.run('final', [str(STEP / 'train.py'), str(DATA), str(final), '--epochs', '6', '--workers', '6', '--corrector',
                      '--final'], TRAIN_PY)
  check = subprocess.run([str(TRAIN_PY), '-c', VERIFY, str(final / 'stop_line.onnx')], capture_output=True, text=True)
  if check.returncode != 0:
    night.log(f'final: cv2 could not run the export, not staged: {check.stderr[-300:]}')
    return
  staged = RUNS / 'staged'
  shutil.rmtree(staged, ignore_errors=True)
  staged.mkdir()
  shutil.copy(final / 'stop_line.onnx', staged / 'stop_line.onnx')
  (staged / 'metrics.json').write_text(json.dumps(dict(new, trained=night.dir.name), indent=1))
  night.log('staged a better model')
  deploy(night)


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument('--deploy', action='store_true', help='only push a staged model to the comma')
  args = ap.parse_args()
  night = Night(f'{dt.date.today():%Y-%m-%d}')
  lock = open(RUNS / 'nightly.lock', 'w')
  try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)  # released by the OS even if this run dies
  except BlockingIOError:
    if not args.deploy:
      night.log('another run is going, skipped')
    return
  try:
    deploy(night, quiet=True) if args.deploy else nightly(night)
  except Exception as e:
    night.log(f'failed: {e}')
    raise


if __name__ == '__main__':
  main()
