#!/usr/bin/env python3
"""Parking-lot EPS limit test for the eps-lab branch: how hard, how fast, how far and how long this car's steering goes.

On the comma (eps-lab installed, car engaged in an empty lot):
  python3 scripts/eps_lab.py plan --list           # the steps
  python3 scripts/eps_lab.py plan                  # run every step in order (resumes where it stopped)
  python3 scripts/eps_lab.py plan --from 11        # start at step 11;  --only 5,6,7  just those;  --redo  ignore saved state
  (after the torque climb and the time-between-sags climb, the plan narrows the gap between the highest clean value
  and the first faulting one, then repeats the winner the other way: the true limit, not just the nearest step)
  python3 scripts/eps_lab.py push 1.0 6 --steer-max 512 --delta-up 12     # one ad-hoc run (also: hold, reverse)
  python3 scripts/eps_lab.py reset                 # stock values, scripted steering off
On the PC:
  python3 scripts/eps_lab.py report --pull         # copy the runs from the comma and print the results

Every run is armed by the driver's blinker (left blinker = steer left), starts only with steering active, the wheel
within 30 deg of center and under 12 mph, and stops at once (zero torque) on a fault, brake / disengage, over 12 mph or
the driver holding the wheel (> 200 for 0.25 s). The torque, ramp and >90 deg cut-out come from
/data/starpilot/eps_test.json, which opendbc's Hyundai controller re-reads live (no restart between runs) and clamps to
the eps-lab firmware ceiling 768 / 12 / 15; it is put back to stock when the script ends. Each run is saved, with its
100 Hz samples, to /data/eps_lab/runs/. After the session, reinstall 1337-8374 so pandad reflashes the normal firmware.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

CFG_PATH = Path('/data/starpilot/eps_test.json')
LAB = Path('/data/eps_lab')
RUNS = LAB / 'runs'
STATE = LAB / 'state.json'
STOCK = {'steer_max': 384, 'delta_up': 3, 'delta_down': 7, 'max_angle': 85, 'max_angle_frames': 89, 'cut_frames': 2,
         'cut_boost': 0, 'cut_boost_max': 0, 'cut_boost_frames': 0, 'cut_preload_frames': 0, 'cut_early_pct': 0}
CEILING = {'steer_max': 768, 'delta_up': 12, 'delta_down': 15}
MAX_MPH = 12.0
ARM_TIMEOUT_S = 240.0
HOSTS = ('192.168.1.157', '100.117.14.62')

# The plan. cfg 'best' = the highest torque / ramp that ran without a fault earlier in this session.
# run: (kind, args); push (level, seconds), hold (deg, seconds), reverse (first s, second s).
PLAN = [
  # name, what it answers, cfg, run, driving note
  ('base', 'stock baseline (384 / ramp 3)', {}, ('push', (1.0, 6.0)), 'roll ~3 mph'),
  ('ramp-6', 'ramp rate 6/frame', {'delta_up': 6, 'delta_down': 9}, ('push', (1.0, 4.0)), 'roll ~3 mph'),
  ('ramp-9', 'ramp rate 9/frame', {'delta_up': 9, 'delta_down': 12}, ('push', (1.0, 4.0)), 'roll ~3 mph'),
  ('ramp-12', 'ramp rate 12/frame (ceiling)', {'delta_up': 12, 'delta_down': 15}, ('push', (1.0, 4.0)), 'roll ~3 mph'),
  ('tq-448', 'torque 448 (3.5 Nm)', {'steer_max': 448, 'ramp': 'best'}, ('push', (1.0, 6.0)), 'roll ~3 mph'),
  ('tq-512', 'torque 512 (4.0 Nm)', {'steer_max': 512, 'ramp': 'best'}, ('push', (1.0, 6.0)), 'roll ~3 mph'),
  ('tq-576', 'torque 576 (4.5 Nm)', {'steer_max': 576, 'ramp': 'best'}, ('push', (1.0, 6.0)), 'roll ~3 mph'),
  ('tq-640', 'torque 640 (5.0 Nm)', {'steer_max': 640, 'ramp': 'best'}, ('push', (1.0, 6.0)), 'roll ~3 mph'),
  ('tq-704', 'torque 704 (5.5 Nm)', {'steer_max': 704, 'ramp': 'best'}, ('push', (1.0, 6.0)), 'roll ~3 mph'),
  ('tq-768', 'torque 768 (6.0 Nm, ceiling)', {'steer_max': 768, 'ramp': 'best'}, ('push', (1.0, 6.0)), 'roll ~3 mph'),
  ('hold-45', 'hold 45 deg: torque needed, steadiness', {'best': True}, ('hold', (45.0, 8.0)), 'roll ~3 mph'),
  ('hold-90', 'hold 90 deg (the EPS limit angle)', {'best': True}, ('hold', (90.0, 8.0)), 'roll ~3 mph'),
  ('hold-180', 'hold 180 deg (half a turn)', {'best': True}, ('hold', (180.0, 8.0)), 'roll ~3 mph'),
  ('hold-270', 'hold 270 deg', {'best': True}, ('hold', (270.0, 8.0)), 'roll ~3 mph'),
  ('sag-base', 'the sag at each stock cut-out (2 frames dropped every 0.89 s past 85 deg)', {'best': True},
   ('hold', (135.0, 10.0)), 'roll ~3 mph'),
  ('sag-1frame', 'drop 1 frame instead of 2: half the sag, if the EPS still resets its timer', {'best': True, 'cut_frames': 1},
   ('hold', (135.0, 10.0)), 'roll ~3 mph'),
  ('cut-angle-90', 'start counting at 90 deg instead of 85', {'best': True, 'max_angle': 90}, ('hold', (135.0, 10.0)), 'roll ~3 mph'),
  ('cut-every-1.0s', 'longest time between sags: 1.0 s', {'best': True, 'max_angle_frames': 100}, ('hold', (135.0, 10.0)), 'roll ~3 mph'),
  ('cut-every-1.2s', 'longest time between sags: 1.2 s', {'best': True, 'max_angle_frames': 120}, ('hold', (135.0, 10.0)), 'roll ~3 mph'),
  ('cut-every-1.5s', 'longest time between sags: 1.5 s', {'best': True, 'max_angle_frames': 150}, ('hold', (135.0, 10.0)), 'roll ~3 mph'),
  # Sag shaping, compared on the same turn (in to 180 deg over 1.2 s, hold 3 s, back out): tracking error and sag.
  ('turn-stock', 'turn profile with plain sags', {'best': True}, ('turn', (180.0, 3.0)), 'roll ~3 mph'),
  ('turn-boost', 'sag boost: +128 for 0.15 s after each cut', {'best': True, 'cut_boost': 128, 'cut_boost_frames': 15},
   ('turn', (180.0, 3.0)), 'roll ~3 mph'),
  ('turn-preload', 'preload: +96 for 0.1 s before each cut, 0.05 s after', {'best': True, 'cut_boost': 96, 'cut_preload_frames': 10,
   'cut_boost_frames': 5}, ('turn', (180.0, 3.0)), 'roll ~3 mph'),
  ('turn-early', 'early sag: cut when demand is under 60% of max', {'best': True, 'cut_early_pct': 60}, ('turn', (180.0, 3.0)),
   'roll ~3 mph'),
  ('turn-all', 'early sag + boost together', {'best': True, 'cut_early_pct': 60, 'cut_boost': 128, 'cut_boost_frames': 15},
   ('turn', (180.0, 3.0)), 'roll ~3 mph'),
  ('turn-boost-over', 'boost may briefly pass the proven torque (to 768): does a short burst fault?',
   {'best': True, 'cut_boost': 128, 'cut_boost_frames': 15, 'cut_boost_max': 768}, ('turn', (180.0, 3.0)), 'roll ~3 mph'),
  ('reverse', 'fast reversal: full one way, then full the other', {'best': True}, ('reverse', (2.0, 4.0)), 'roll ~3 mph'),
  ('lock-rolling', 'how far it can turn the wheel, rolling', {'best': True}, ('push', (1.0, 10.0)), 'roll ~3 mph'),
  ('lock-crawl', 'how far it can turn the wheel, barely moving', {'best': True}, ('push', (1.0, 8.0)), 'creep, as slow as possible'),
  ('stopped-push', 'does the EPS take torque at a standstill?', {'best': True}, ('push', (1.0, 5.0)),
   'STOPPED in Drive, foot on the brake'),
  ('stopped-hold-90', 'hold 90 deg at a standstill', {'best': True}, ('hold', (90.0, 6.0)), 'STOPPED in Drive, foot on the brake'),
  ('speed-8', 'hold 90 deg at ~8 mph: torque needed with speed', {'best': True}, ('hold', (90.0, 5.0)), 'roll ~8 mph, room to circle'),
]


# ---------------------------------------------------------------- config file
def write_cfg(values: dict | None) -> None:
  CFG_PATH.parent.mkdir(parents=True, exist_ok=True)
  doc = {'steer_from_test': True, **(values or {})} if values is not None else {}
  tmp = CFG_PATH.with_suffix('.tmp')
  tmp.write_text(json.dumps(doc))
  os.replace(tmp, CFG_PATH)


def resolve_cfg(cfg: dict, best: dict) -> dict:
  out = dict(STOCK)
  if cfg.get('best') or cfg.get('ramp') == 'best':
    out.update({k: best[k] for k in ('delta_up', 'delta_down') if k in best})
  if cfg.get('best'):
    out.update({k: best[k] for k in ('steer_max', 'cut_frames', 'max_angle', 'max_angle_frames') if k in best})
  out.update({k: v for k, v in cfg.items() if k in STOCK})
  for k, high in CEILING.items():
    out[k] = min(out[k], high)
  return out


def load_state() -> dict:
  try:
    return json.loads(STATE.read_text())
  except (OSError, ValueError):
    return {'done': [], 'best': {}}


def save_state(state: dict) -> None:
  LAB.mkdir(parents=True, exist_ok=True)
  STATE.write_text(json.dumps(state, indent=1))


# ---------------------------------------------------------------- live car
class Car:
  def __init__(self):
    sys.path[:0] = ['/data/openpilot', '/data/openpilot/openpilot']
    from openpilot.cereal import messaging
    self.messaging = messaging
    self.pm = messaging.PubMaster(['testJoystick'])
    self.sm = messaging.SubMaster(['carState', 'carControl', 'carOutput'])
    self.can = messaging.sub_sock('sendcan', conflate=False, timeout=0)
    self.lkas_raw, self.lkas_req = 0, True

  def send(self, steer: float) -> None:
    msg = self.messaging.new_message('testJoystick')
    msg.testJoystick.axes = [0.0, float(max(-1.0, min(1.0, steer)))]
    self.pm.send('testJoystick', msg)

  def update(self, timeout_ms: int = 0) -> None:
    self.sm.update(timeout_ms)
    reqs = []
    for evt in self.messaging.drain_sock(self.can):
      for c in evt.sendcan:
        if c.address == 0x340 and len(c.dat) >= 4:   # LKAS11: what actually goes to the EPS
          word = int.from_bytes(bytes(c.dat[:4]), 'little')
          self.lkas_raw = ((word >> 16) & 0x7FF) - 1024
          reqs.append(bool((word >> 27) & 1))
    if reqs:
      self.lkas_req = all(reqs)   # a 1-frame cut-out between two samples still shows

  def sample(self, t0: float, cmd: float) -> dict:
    cs, cc = self.sm['carState'], self.sm['carControl']
    return {'t': round(time.monotonic() - t0, 3), 'cmd': round(cmd, 3), 'angle': round(cs.steeringAngleDeg, 1),
            'rate': round(cs.steeringRateDeg, 1), 'driver': round(cs.steeringTorque), 'eps': round(cs.steeringTorqueEps),
            'sent': self.lkas_raw, 'req': int(self.lkas_req), 'out': round(self.sm['carOutput'].actuatorsOutput.torque, 3),
            'mph': round(cs.vEgo * 2.237, 1), 'lat': int(cc.latActive),
            'fault': 2 if cs.steerFaultPermanent else 1 if cs.steerFaultTemporary else 0}

  def zero(self, seconds: float = 0.4) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
      self.send(0.0)
      time.sleep(0.01)


def say(msg: str) -> None:
  print(f'[{time.strftime("%H:%M:%S")}] {msg}', flush=True)


def arm(car: Car, note: str) -> int | None:
  """Wait for one blinker with steering active, wheel near center, slow. Returns +1 (left) / -1 (right)."""
  say(f'ARMED ({note}): straighten the wheel, then put on the blinker for the side to test')
  deadline, last_why = time.monotonic() + ARM_TIMEOUT_S, ''
  while time.monotonic() < deadline:
    car.send(0.0)
    car.update(20)
    cs, cc = car.sm['carState'], car.sm['carControl']
    why = ('steering not active (engage)' if not cc.latActive else
           'wheel not centered' if abs(cs.steeringAngleDeg) > 30 else
           f'too fast ({cs.vEgo * 2.237:.0f} mph)' if cs.vEgo * 2.237 > MAX_MPH else
           'steering fault' if cs.steerFaultTemporary or cs.steerFaultPermanent else '')
    if not why and cs.leftBlinker != cs.rightBlinker:
      side = 1 if cs.leftBlinker else -1
      say(f"GO {'LEFT' if side > 0 else 'RIGHT'}: hands off the wheel")
      return side
    if why and why != last_why and (cs.leftBlinker or cs.rightBlinker):
      say(f'waiting: {why}')
    last_why = why if (cs.leftBlinker or cs.rightBlinker) else ''
  say('no blinker within 4 minutes: skipped')
  return None


def execute(car: Car, kind: str, args: tuple, side: int) -> dict:
  """Run one steering pattern at 100 Hz. Returns samples, the stop reason and timing marks."""
  samples, marks = [], {}
  t0 = time.monotonic()
  if kind == 'push':
    level, secs = args
    segments = [(secs, lambda s: side * level)]
  elif kind == 'reverse':
    first, second = args
    segments = [(first, lambda s: side * 1.0), (second, lambda s: -side * 1.0)]
  elif kind == 'hold':
    target, secs = args
    target *= side

    def hold_cmd(s):   # PD on the wheel angle: full torque at 40 deg error, damped by the wheel speed
      return (target - s['angle']) / 40.0 - s['rate'] / 300.0
    segments = [(secs, hold_cmd)]
  elif kind == 'turn':
    peak, hold_s = args
    peak *= side

    def turn_target(t):
      return peak * (min(1.0, t / 1.2) if t < 1.2 + hold_s else max(0.0, 1.0 - (t - 1.2 - hold_s) / 1.2))

    def turn_cmd(s):
      s['target'] = round(turn_target(s['t']), 1)
      return (s['target'] - s['angle']) / 40.0 - s['rate'] / 300.0
    segments = [(2.4 + hold_s, turn_cmd)]
  else:
    raise ValueError(kind)

  stop, driver_since, last_print, cmd = 'done', None, 0.0, 0.0
  seg_end = t0
  for index, (secs, fn) in enumerate(segments):
    seg_start, seg_end = time.monotonic(), time.monotonic() + secs
    if index == 1:
      marks['reverse_at'] = round(seg_start - t0, 3)
    while time.monotonic() < seg_end:
      car.update(0)
      s = car.sample(t0, cmd)
      cmd = max(-1.0, min(1.0, fn(s)))
      s['cmd'] = round(cmd, 3)
      samples.append(s)
      a = abs(s['angle'])
      for mark in (90, 180, 270, 360, 450):
        if a >= mark and f't{mark}' not in marks and index == 0:
          marks[f't{mark}'] = s['t']
      if s['fault']:
        stop = 'PERMANENT FAULT' if s['fault'] == 2 else 'fault'
        marks['fault_t'], marks['fault_angle'] = s['t'], s['angle']
      elif not s['lat']:
        stop = 'steering went inactive (brake / disengage)'
      elif s['mph'] > MAX_MPH:
        stop = f"over {MAX_MPH:.0f} mph"
      elif abs(s['driver']) > 200:
        driver_since = driver_since if driver_since is not None else time.monotonic()
        if time.monotonic() - driver_since > 0.25:
          stop = 'driver held the wheel'
      else:
        driver_since = None
      if stop != 'done':
        break
      car.send(cmd)
      if s['t'] - last_print >= 0.5:
        last_print = s['t']
        print(f"  t {s['t']:5.2f}  cmd {cmd:+.2f}  sent {s['sent']:+5d}  angle {s['angle']:+7.1f}  rate {s['rate']:+6.0f}/s"
              + f"  eps {s['eps']:+5d}  driver {s['driver']:+5d}  {s['mph']:4.1f} mph", flush=True)
      time.sleep(max(0.0, 0.01 - (time.monotonic() - t0 - s['t'])))
    if stop != 'done':
      break
  car.zero()
  # Let go for 2 s (how the wheel comes back); after a temporary fault, up to 60 s for steering to return.
  release_end = time.monotonic() + (60.0 if stop == 'fault' else 2.0)
  while time.monotonic() < release_end:
    car.send(0.0)
    car.update(10)
    s = car.sample(t0, 0.0)
    s['phase'] = 'release'
    samples.append(s)
    if stop == 'fault' and not s['fault'] and s['lat']:
      marks['recovered_s'] = round(s['t'] - marks['fault_t'], 2)
      break
  if 'fault_t' in marks:
    marks.setdefault('recovered_s', None)
  return {'stop': stop, 'marks': marks, 'samples': samples}


def summarize(kind: str, args: tuple, run: dict) -> dict:
  act = [s for s in run['samples'] if 'phase' not in s]
  if not act:
    return {'stop': run['stop']}
  peak = max(abs(s['angle']) for s in act)
  out = {'stop': run['stop'], 'peak_angle': round(peak), 'turns': round(peak / 360, 2),
         'peak_rate': round(max(abs(s['rate']) for s in act)), 'peak_sent': max(abs(s['sent']) for s in act),
         'peak_eps': max(abs(s['eps']) for s in act), 'mph': round(sum(s['mph'] for s in act) / len(act), 1),
         'cutouts': sum(1 for a, b in zip(act, act[1:], strict=False) if a['req'] and not b['req']), **run['marks']}
  # Stall: the wheel stops (< 5 deg/s for 0.5 s) while still pushed at >= 90% command.
  still = 0.0
  for a, b in zip(act, act[1:], strict=False):
    still = still + (b['t'] - a['t']) if abs(b['cmd']) >= 0.9 and abs(b['rate']) < 5 else 0.0
    if still >= 0.5 and 'stall_angle' not in out:
      out['stall_angle'] = round(abs(b['angle']))
  # Ramp: time from the start to full commanded torque on the bus.
  full = next((s['t'] for s in act if abs(s['sent']) >= 0.98 * max(1, out['peak_sent'])), None)
  out['t_full_torque'] = full
  out.update(sag_stats(act))
  if kind == 'hold':
    target = args[0]
    reached = next((s['t'] for s in act if abs(abs(s['angle']) - target) <= 5), None)
    out['t_reach'] = reached
    if reached is not None:
      settled = [s for s in act if s['t'] >= reached + 1.0]
      if settled:
        out['hold_err'] = round(sum(abs(abs(s['angle']) - target) for s in settled) / len(settled), 1)
        out['hold_cmd'] = round(sum(abs(s['cmd']) for s in settled) / len(settled), 2)
        out['hold_sent'] = round(sum(abs(s['sent']) for s in settled) / len(settled))
        out['held_s'] = round(settled[-1]['t'] - reached, 1)
  if kind == 'turn':
    tracked = [s for s in act if 'target' in s]
    if tracked:
      out['track_err'] = round(sum(abs(s['target'] - s['angle']) for s in tracked) / len(tracked), 1)
      held = [s for s in tracked if 1.5 <= s['t'] <= 1.2 + args[1]]
      if held:
        out['track_err_hold'] = round(sum(abs(s['target'] - s['angle']) for s in held) / len(held), 1)
  if kind == 'reverse':
    at = run['marks'].get('reverse_at')
    after = [s for s in act if at is not None and s['t'] >= at]
    if after:
      start_sign = math.copysign(1, after[0]['angle']) if after[0]['angle'] else 1
      cross = next((s['t'] for s in after if math.copysign(1, s['angle']) != start_sign), None)
      out['reverse_to_center_s'] = round(cross - at, 2) if cross is not None else None
      out['reverse_peak_rate'] = round(max(abs(s['rate']) for s in after))
  return out


def sag_stats(act: list) -> dict:
  """Per cut-out (request bit dropped): how far the wheel falls back, how long until it is back, and how far and for
  how long the motor torque drops. Averages and worst case over the run."""
  sags, backs, dips, eps_backs, gaps = [], [], [], [], []
  cuts = [k for k in range(1, len(act)) if act[k - 1]['req'] and not act[k]['req']]
  for n, k in enumerate(cuts):
    t0, a0, e0 = act[k]['t'], abs(act[k]['angle']), abs(act[k - 1]['eps'])
    if n:
      gaps.append(t0 - act[cuts[n - 1]]['t'])
    win = [x for x in act[k:] if x['t'] - t0 <= 0.6]
    if not win or win[-1]['t'] - t0 < 0.5:
      continue
    low = min(win, key=lambda x: abs(x['angle']))
    sags.append(a0 - abs(low['angle']))
    back = next((x['t'] for x in win if x['t'] >= low['t'] and abs(x['angle']) >= a0 - 0.5), None)
    backs.append((back if back is not None else win[-1]['t']) - t0)
    if e0 >= 20:
      e_low = min(win[:40], key=lambda x: abs(x['eps']))
      dips.append(1 - abs(e_low['eps']) / e0)
      e_back = next((x['t'] for x in win if x['t'] >= e_low['t'] and abs(x['eps']) >= 0.9 * e0), None)
      eps_backs.append((e_back if e_back is not None else win[-1]['t']) - t0)
  out = {}
  if sags:
    out.update({'sag_deg': round(sum(sags) / len(sags), 1), 'sag_deg_max': round(max(sags), 1),
                'sag_back_s': round(sum(backs) / len(backs), 2)})
  if dips:
    out.update({'eps_drop_pct': round(100 * sum(dips) / len(dips)), 'eps_back_s': round(sum(eps_backs) / len(eps_backs), 2)})
  if gaps:
    out['s_between_sags'] = round(sum(gaps) / len(gaps), 2)
  return out


def line(name: str, summary: dict) -> str:
  keys = ('stop', 'peak_angle', 'turns', 'peak_rate', 't90', 't180', 't270', 't_full_torque', 'stall_angle', 'peak_sent',
          'peak_eps', 'cutouts', 's_between_sags', 'sag_deg', 'sag_deg_max', 'sag_back_s', 'eps_drop_pct', 'eps_back_s',
          't_reach', 'hold_err', 'hold_sent', 'held_s', 'track_err', 'track_err_hold', 'reverse_to_center_s', 'fault_angle',
          'recovered_s', 'mph')
  return f'{name}: ' + ', '.join(f'{k} {summary[k]}' for k in keys if summary.get(k) is not None)


def save(name: str, cfg: dict, kind: str, args: tuple, side: int, run: dict, summary: dict) -> Path:
  RUNS.mkdir(parents=True, exist_ok=True)
  path = RUNS / f"{time.strftime('%Y%m%d-%H%M%S')}-{name}.json"
  path.write_text(json.dumps({'name': name, 'cfg': cfg, 'kind': kind, 'args': list(args), 'side': side,
                              'summary': summary, 'samples': run['samples']}))
  return path


def one(car: Car, name: str, cfg: dict, kind: str, args: tuple, note: str) -> dict | None:
  write_cfg(cfg)
  say(f"{name}: torque {cfg['steer_max']}, ramp {cfg['delta_up']}/{cfg['delta_down']}, cut-out past {cfg['max_angle']} deg every "
      + f"{cfg['max_angle_frames'] / 100:.2f} s for {cfg['cut_frames']} frame(s); {kind} {args}")
  time.sleep(1.3)   # the controller re-reads the file each second
  side = arm(car, note)
  if side is None:
    return None
  run = execute(car, kind, args, side)
  summary = summarize(kind, args, run)
  path = save(name, cfg, kind, args, side, run, summary)
  say(line(name, summary))
  say(f'saved {path.name}')
  return summary


# ---------------------------------------------------------------- narrowing a limit
# After a climb faults, halve the gap between the highest clean value and the lowest faulting one, then repeat the
# winner the other way (one clean run can be luck; a fault on the repeat steps it down once).
SEARCHES = {
  'tq-': {'what': 'torque', 'key': 'steer_max', 'fault': 'tq_fault', 'done': 'refined', 'step': 16, 'floor': 384,
          'label': 'tq-refine', 'cfg': lambda v: {'steer_max': v, 'ramp': 'best'}, 'run': ('push', (1.0, 6.0)),
          'unit': lambda v: str(v)},
  'cut-every-': {'what': 'time between sags', 'key': 'max_angle_frames', 'fault': 'interval_fault', 'done': 'interval_refined',
                 'step': 3, 'floor': 89, 'label': 'cut-interval', 'cfg': lambda v: {'best': True, 'max_angle_frames': v},
                 'run': ('hold', (135.0, 10.0)), 'unit': lambda v: f'{v / 100:.2f} s'},
}
NARROW_MAX_RUNS = 4
for _prefix, _search in SEARCHES.items():
  _search['last_step'] = max(i for i, step in enumerate(PLAN, 1) if step[0].startswith(_prefix))


def fault_kind(summary: dict) -> str:
  angle = summary.get('fault_angle')
  return 'past the cut-out angle (angle / time rule)' if angle is not None and abs(angle) >= STOCK['max_angle'] else \
         'below the cut-out angle (the torque or ramp itself)'


def narrow(car, state: dict, search: dict) -> str:
  """Returns 'done', 'skipped' (no blinker / stopped early) or 'perm'."""
  key, unit = search['key'], search['unit']
  good, bad = state['best'].get(key, STOCK[key]), state[search['fault']]
  say(f"--- narrowing the {search['what']}: {unit(good)} ran clean, {unit(bad)} faulted; to within {unit(search['step'])}")

  def attempt(value, label, note='roll ~3 mph'):
    summary = one(car, label, resolve_cfg(search['cfg'](value), state['best']), *search['run'], note)
    return summary if summary is not None and summary['stop'] in ('done', 'fault', 'PERMANENT FAULT') else None

  def record(new_good, new_bad):
    state['best'][key], state[search['fault']] = new_good, new_bad
    save_state(state)

  runs = 0
  while bad - good > search['step'] and runs < NARROW_MAX_RUNS:
    runs += 1
    mid = good + max(search['step'], (bad - good) // 2 // search['step'] * search['step'])
    summary = attempt(mid, f"{search['label']}-{mid}")
    if summary is None:
      return 'skipped'
    if summary['stop'] == 'done':
      good = mid
    else:
      bad = mid
      say(f'{unit(mid)} faulted {fault_kind(summary)}')
    record(good, bad)
    if summary['stop'] == 'PERMANENT FAULT':
      return 'perm'
  for _repeat in range(2):
    summary = attempt(good, f"{search['label']}-confirm-{good}", 'roll ~3 mph, signal the OTHER side from last run')
    if summary is None:
      return 'skipped'
    if summary['stop'] == 'done':
      break
    say(f'{unit(good)} faulted on the repeat {fault_kind(summary)}')
    good, bad = max(search['floor'], good - search['step']), good
    record(good, bad)
    if summary['stop'] == 'PERMANENT FAULT':
      return 'perm'
  state[search['done']] = True
  save_state(state)
  say(f"{search['what']} limit: {unit(good)} works (twice), {unit(bad)} faults")
  return 'done'


def pending_narrowing(state: dict, step_index: int) -> list:
  return [x for x in SEARCHES.values() if step_index > x['last_step'] and state.get(x['fault']) and not state.get(x['done'])]


# ---------------------------------------------------------------- commands
def cmd_plan(a) -> int:
  if a.list:
    for i, (name, what, _cfg, (kind, args), note) in enumerate(PLAN, 1):
      print(f'{i:2}. {name:15} {what}  [{kind} {args}; {note}]')
    return 0
  state = {'done': [], 'best': {}} if a.redo else load_state()
  only = {int(x) for x in a.only.split(',')} if a.only else None
  car = Car()
  try:
    for i, (name, what, cfg, (kind, args), note) in enumerate(PLAN, 1):
      if i < a.from_ or (only and i not in only) or (not only and not a.from_ > 1 and name in state['done']):
        continue
      search = next((x for p, x in SEARCHES.items() if name.startswith(p)), None)
      if search and state.get(search['fault']):
        say(f"--- step {i}: {name} skipped ({search['what']} {search['unit'](state[search['fault']])} already faulted)")
        continue
      for pending in pending_narrowing(state, i):
        result = narrow(car, state, pending)
        if result == 'perm':
          say(f'PERMANENT steering fault: turn the car off and on, then resume with: plan --from {i}')
          return 2
        if result == 'skipped':
          say(f'narrowing stopped; resume with: plan --from {i}')
          return 1
      say(f'--- step {i}/{len(PLAN)}: {name} - {what}')
      resolved = resolve_cfg(cfg, state['best'])
      summary = one(car, name, resolved, kind, args, note)
      if summary is None:
        say(f'stopping the plan here; resume with: plan --from {i}')
        return 1
      state['done'].append(name)
      clean = summary['stop'] == 'done'
      # 'best' = every value that ran clean (no fault, not stopped early): fastest ramp, highest torque, the shorter
      # cut, the later cut angle, the longest time between sags. Later 'best' steps run with all of them together.
      if clean and name.startswith('ramp-'):
        state['best'].update({k: resolved[k] for k in ('delta_up', 'delta_down')})
      if clean and (name.startswith('tq-') or name == 'base'):
        state['best']['steer_max'] = max(state['best'].get('steer_max', 0), resolved['steer_max'])
      if clean and name == 'sag-1frame':
        state['best']['cut_frames'] = 1
      if clean and name == 'cut-angle-90':
        state['best']['max_angle'] = 90
      if clean and name.startswith('cut-every-'):
        state['best']['max_angle_frames'] = max(state['best'].get('max_angle_frames', 0), resolved['max_angle_frames'])
      if search and not clean and summary['stop'] in ('fault', 'PERMANENT FAULT'):
        state[search['fault']] = resolved[search['key']]   # higher steps of this climb are skipped from here
      save_state(state)
      if summary['stop'] == 'PERMANENT FAULT':
        say(f'PERMANENT steering fault: turn the car off and on, then resume with: plan --from {i + 1}')
        return 2
      if not clean and summary['stop'] != 'fault':
        say(f"run stopped early ({summary['stop']}); resume with: plan --from {i}")
        return 1
    for pending in pending_narrowing(state, len(PLAN) + 1):
      if narrow(car, state, pending) != 'done':
        say('narrowing stopped; run the plan again to finish it')
        return 1
    say(f"plan finished. best: {state['best']}. Pull the results to the PC with: scripts/eps_lab.py report --pull")
    return 0
  finally:
    write_cfg({})   # stock values, scripted steering still allowed
    say('eps_test.json back to stock values')


def cmd_single(a) -> int:
  cfg = resolve_cfg({k: v for k, v in (('steer_max', a.steer_max), ('delta_up', a.delta_up), ('delta_down', a.delta_down),
                                       ('max_angle', a.max_angle), ('max_angle_frames', a.max_angle_frames),
                                       ('cut_frames', a.cut_frames), ('cut_boost', a.cut_boost), ('cut_boost_max', a.cut_boost_max),
                                       ('cut_boost_frames', a.cut_boost_frames),
                                       ('cut_preload_frames', a.cut_preload_frames), ('cut_early_pct', a.cut_early_pct)) if v is not None}, {})
  car = Car()
  try:
    return 0 if one(car, f'{a.cmd}-adhoc', cfg, a.cmd, (a.x, a.seconds), 'ad-hoc') else 1
  finally:
    write_cfg({})


def cmd_reset(_a) -> int:
  write_cfg(None)
  say('eps_test.json emptied: stock torque / ramp / cut-out, scripted steering off')
  return 0


def cmd_report(a) -> int:
  root = Path(a.dir)
  if a.pull:
    root.mkdir(parents=True, exist_ok=True)
    for host in HOSTS:
      r = subprocess.run(['rsync', '-a', '-e', 'ssh -o ConnectTimeout=6', f'comma@{host}:{RUNS}/', f'{root}/'], capture_output=True)
      if r.returncode == 0:
        say(f'pulled runs from {host} into {root}')
        break
    else:
      say('could not reach the comma; reporting what is already here')
  runs = []
  for p in sorted(root.glob('*.json')):
    try:
      runs.append(json.loads(p.read_text()))
    except ValueError:
      continue
  if not runs:
    say(f'no runs in {root}')
    return 1
  for r in runs:
    c = r['cfg']
    side = 'L' if r['side'] > 0 else 'R'
    print(f"{side} tq {c['steer_max']:3} ramp {c['delta_up']:2}/{c['delta_down']:2} cut {c['max_angle']:.0f}/{c['max_angle_frames']}  "
          + line(r['name'], r['summary']))
  clean = [r for r in runs if r['summary'].get('stop') == 'done' and r['kind'] == 'push']
  if clean:
    fast = max(clean, key=lambda r: r['summary'].get('peak_rate', 0))
    far = max(clean, key=lambda r: r['summary'].get('peak_angle', 0))
    quick = min((r for r in clean if r['summary'].get('t90')), key=lambda r: r['summary']['t90'], default=None)
    print(f"\nfastest wheel: {fast['summary']['peak_rate']} deg/s ({fast['name']}); farthest: {far['summary']['peak_angle']} deg = "
          + f"{far['summary']['turns']} turns ({far['name']})" + (f"; quickest to 90 deg: {quick['summary']['t90']} s ({quick['name']})" if quick else ''))
  # More command only helps while the motor gives more back: show what each clean torque level delivered.
  tiers = {}
  for r in clean:
    if r['name'].startswith(('base', 'tq-', 'ramp-12', 'lock')):
      tiers.setdefault(r['cfg']['steer_max'], []).append(r['summary'])
  prev = None
  for tq in sorted(tiers):
    eps = max(x.get('peak_eps', 0) for x in tiers[tq])
    rate = max(x.get('peak_rate', 0) for x in tiers[tq])
    far = max(x.get('peak_angle', 0) for x in tiers[tq])
    gain = '' if prev is None or not prev[0] else f'  (+{(eps - prev[0]) / prev[0] * 100:.0f}% motor torque vs {prev[1]})'
    # Maxed out: less than a third of the extra torque we asked for came back (motor torque, wheel speed).
    flat = ''
    if prev and prev[0] and prev[2]:
      asked = (tq - prev[1]) / prev[1]
      if (eps - prev[0]) / prev[0] < 0.33 * asked:
        flat += '  <- motor torque maxed out'
      if (rate - prev[2]) / prev[2] < 0.33 * asked:
        flat += '  <- wheel no faster'
    print(f'torque {tq}: motor gave up to {eps}, wheel {rate} deg/s, reached {far} deg{gain}{flat}')
    prev = (eps, tq, rate)
  turns = [r for r in runs if r['kind'] == 'turn' and r['summary'].get('stop') == 'done']
  if turns:
    print('\nsag shaping on the same turn (lower is better):')
    for r in sorted(turns, key=lambda r: r['summary'].get('track_err_hold', 99)):
      x = r['summary']
      print(f"  {r['name']:14} off target {x.get('track_err_hold')} deg while held ({x.get('track_err')} whole turn), "
            + f"sag {x.get('sag_deg')} deg (worst {x.get('sag_deg_max')}), back in {x.get('sag_back_s')} s")
  faults = [r for r in runs if r['summary'].get('stop', '').endswith('FAULT') or r['summary'].get('stop') == 'fault']
  for r in faults:
    s = r['summary']
    print(f"fault in {r['name']}: at {s.get('fault_angle')} deg, t {s.get('fault_t')} s, recovered in {s.get('recovered_s')} s")
  return 0


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__.splitlines()[0], formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
  sub = ap.add_subparsers(dest='cmd', required=True)
  p = sub.add_parser('plan')
  p.add_argument('--list', action='store_true')
  p.add_argument('--from', dest='from_', type=int, default=1)
  p.add_argument('--only')
  p.add_argument('--redo', action='store_true')
  for kind, x_help in (('push', 'command level 0..1'), ('hold', 'angle in degrees'), ('reverse', 'seconds the first way'),
                       ('turn', 'peak angle in degrees')):
    p = sub.add_parser(kind)
    p.add_argument('x', type=float, help=x_help)
    p.add_argument('seconds', type=float)
    for opt in ('steer-max', 'delta-up', 'delta-down', 'max-angle-frames', 'cut-frames', 'cut-boost', 'cut-boost-max', 'cut-boost-frames',
                'cut-preload-frames', 'cut-early-pct'):
      p.add_argument(f'--{opt}', type=int)
    p.add_argument('--max-angle', type=float)
  sub.add_parser('reset')
  p = sub.add_parser('report')
  p.add_argument('--pull', action='store_true')
  p.add_argument('--dir', default=str(Path.home() / 'eps_lab_runs'))
  a = ap.parse_args()
  return {'plan': cmd_plan, 'push': cmd_single, 'hold': cmd_single, 'reverse': cmd_single, 'turn': cmd_single,
          'reset': cmd_reset, 'report': cmd_report}[a.cmd](a)


if __name__ == '__main__':
  sys.exit(main())
