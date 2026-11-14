#!/usr/bin/env python3
"""1337 trace: one readable daily file on the comma for tracing what happened on a drive.

<starpilot storage>/trace/YYYY-MM-DD.jsonl collects, one JSON line each:
  - every decision-log event ("starpilot_feature": Pulse and Glide, lead loss, AOL, turns, curve guard, NNLC, ...),
    copied from the app log, which otherwise only reaches the full rlogs;
  - "alert": an onroad event that disengages or warns about lag, with a CPU snapshot taken right then (every core,
    temperature, memory, and the busiest processes over the last few seconds);
  - "load": once a minute, the average of every core and the busiest processes over that minute;
  - "process": a program that died while it should have been running (crash or exit, with its exit code), one that
    restarted (new pid), and one that stayed down;
  - "error": every error-level app-log line from any program (an uncaught crash with the last lines of its
    traceback, or a caught error a feature logged), at most once per program and message per 30 s.

Observe only and cheap: app-log messages are filtered on their raw bytes before any decoding, and only the 2 Hz
deviceState, the 0.5 Hz procLog and onroadEvents are decoded. Runs at the lowest priority on the little cores.
Files older than KEEP_DAYS are deleted.
"""
from __future__ import annotations

import json
import os
import time
from collections import deque
from pathlib import Path

KEEP_DAYS = 14
SUMMARY_S = 60.0
LOOP_S = 0.1
CPU_HISTORY = 20      # deviceState samples kept (10 s at 2 Hz)
TOP_N = 10
ALERT_REPEAT_S = 10.0  # an alert that flickers (on/off every few frames) is snapshotted once per this long
ERROR_REPEAT_S = 30.0  # the same error from the same program is written once per this long (with a repeat count)
DOWN_REPORT_S = 5.0    # a program that should run but has been down this long is reported once
FEATURE_MARK = b'starpilot_feature'
# Warnings worth a snapshot even though they do not disengage by themselves.
WATCH_EVENTS = frozenset({'selfdrivedLagging', 'modeldLagging', 'commIssue', 'commIssueAvgFreq', 'processNotRunning',
                          'controlsMismatch', 'cameraMalfunction', 'cameraFrameRate', 'posenetInvalid', 'radarFault',
                          'steerTempUnavailable', 'steerSaturated', 'fcw', 'stockAeb', 'overheat', 'lowMemory'})
PYTHON_PREFIXES = ('/usr/local/venv/bin/python3 -m ', '/usr/local/venv/bin/python -m ', 'python3 -m ', 'python -m ')


def trace_root() -> Path:
  from openpilot.starpilot.storage import starpilot_storage_root
  return starpilot_storage_root() / 'trace'


def feature_record(raw_text: str) -> dict | None:
  """A decision-log event from one app-log message (swaglog JSON), or None."""
  try:
    doc = json.loads(raw_text)
  except (TypeError, ValueError):
    return None
  msg = doc.get('msg') if isinstance(doc, dict) else None
  if not isinstance(msg, dict) or msg.get('event') != 'starpilot_feature':
    return None
  ctx = doc.get('ctx') if isinstance(doc.get('ctx'), dict) else {}
  record = {k: v for k, v in msg.items() if k != 'event'}
  record['kind'] = 'feature'
  record['daemon'] = ctx.get('daemon') or doc.get('filename') or ''
  return record


def error_record(raw_text: str) -> dict | None:
  """An error-level app-log line (swaglog JSON) as a trace record, or None. Decision-log events are skipped here:
  they already reach the trace through the app log."""
  try:
    doc = json.loads(raw_text)
  except (TypeError, ValueError):
    return None
  if not isinstance(doc, dict):
    return None
  msg = doc.get('msg')
  if isinstance(msg, dict) and msg.get('event') == 'starpilot_feature':
    return None
  ctx = doc.get('ctx') if isinstance(doc.get('ctx'), dict) else {}
  text = json.dumps(msg, default=str) if isinstance(msg, dict) else str(msg)
  record = {'kind': 'error', 'daemon': ctx.get('daemon') or '', 'msg': text[:300],
            'where': f"{doc.get('filename', '')}:{doc.get('lineno', '')}"}
  exc = doc.get('exc_info')
  if exc:
    record['exception'] = [line for line in str(exc).strip().splitlines() if line.strip()][-6:]
  return record


def process_name(cmdline, fallback: str = '') -> str:
  name = ' '.join(str(part) for part in cmdline) if cmdline else fallback
  for prefix in PYTHON_PREFIXES:
    if name.startswith(prefix):
      name = name[len(prefix):]
  return name[:60]


def busiest(before: dict[int, tuple[float, str]], after: dict[int, tuple[float, str]], seconds: float, n: int = TOP_N):
  """[(name, % of one core)] busiest first, from two {pid: (cpu seconds, name)} readings `seconds` apart."""
  if seconds <= 0:
    return []
  use = [(after[pid][1], (after[pid][0] - before[pid][0]) / seconds * 100.0) for pid in after if pid in before]
  return [(name, round(pct, 1)) for name, pct in sorted(use, key=lambda x: -x[1])[:n] if pct > 0.5]


def cpu_reading(proc_log) -> dict[int, tuple[float, str]]:
  return {int(p.pid): (float(p.cpuUser + p.cpuSystem), process_name(list(p.cmdline), str(p.name))) for p in proc_log.procs}


def alert_names(onroad_events) -> set[str]:
  """Events worth a snapshot: anything that disengages, plus the lag and fault warnings in WATCH_EVENTS."""
  names = set()
  for event in onroad_events:
    name = str(event.name)
    if event.immediateDisable or event.softDisable or name in WATCH_EVENTS:
      names.add(name)
  return names


class Trace:
  """Pure core: fed decoded messages, returns the lines to write."""

  def __init__(self):
    self.cores: deque[list[float]] = deque(maxlen=CPU_HISTORY)
    self.temp = self.memory = None
    self.procs: deque[tuple[float, dict]] = deque(maxlen=6)   # (t, reading), about 12 s at 0.5 Hz
    self.minute_start: tuple[float, dict] | None = None
    self.minute_cores: list[list[float]] = []
    self.active_alerts: set[str] = set()
    self.alert_logged: dict[str, float] = {}
    self.process_seen: dict[str, tuple[bool, bool, int]] = {}   # name -> (running, should run, pid)
    self.down_since: dict[str, float] = {}
    self.down_reported: set[str] = set()
    self.error_logged: dict[tuple[str, str], list] = {}         # (daemon, message) -> [last written, repeats since]

  def device(self, t: float, cores: list[float], temp: float | None, memory: float | None) -> None:
    if cores:
      self.cores.append([float(c) for c in cores])
      self.minute_cores.append([float(c) for c in cores])
    self.temp, self.memory = temp, memory

  def proc(self, t: float, reading: dict) -> dict | None:
    self.procs.append((t, reading))
    if self.minute_start is None:
      self.minute_start = (t, reading)
      return None
    if t - self.minute_start[0] < SUMMARY_S:
      return None
    t0, r0 = self.minute_start
    line = {'kind': 'load', 'seconds': round(t - t0, 1), 'cores_avg': _avg_cores(self.minute_cores),
            'busiest': busiest(r0, reading, t - t0, 6), 'temp_c': self.temp, 'memory_pct': self.memory}
    self.minute_start, self.minute_cores = (t, reading), []
    return line

  def events(self, t: float, names: set[str]) -> dict | None:
    new = {n for n in names - self.active_alerts if t - self.alert_logged.get(n, -1e9) >= ALERT_REPEAT_S}
    self.active_alerts = names
    if not new:
      return None
    for n in new:
      self.alert_logged[n] = t
    return {'kind': 'alert', 'events': sorted(new), 'all_events': sorted(names), **self.snapshot()}

  def processes(self, t: float, states) -> list[dict]:
    """states: [(name, running, should run, pid, exit code)] from managerState. Lines for programs that died while
    they should have been running, came back under a new pid, or stayed down. Normal stops (the manager shutting a
    program down, e.g. when the car turns off) are not reported."""
    lines = []
    for name, running, should, pid, code in states:
      before = self.process_seen.get(name)
      self.process_seen[name] = (running, should, pid)
      if before is not None:
        was_running, _, old_pid = before
        if was_running and not running and should:
          lines.append({'kind': 'process', 'name': name, 'what': 'died', 'exit_code': code,
                        'meaning': exit_meaning(code), **self.snapshot()})
        elif running and was_running and pid and old_pid and pid != old_pid:
          lines.append({'kind': 'process', 'name': name, 'what': 'restarted', 'old_pid': old_pid, 'pid': pid})
        elif running and not was_running and name in self.down_reported:
          lines.append({'kind': 'process', 'name': name, 'what': 'back up',
                        'down_s': round(t - self.down_since.get(name, t), 1)})
      if should and not running:
        self.down_since.setdefault(name, t)
        if name not in self.down_reported and t - self.down_since[name] >= DOWN_REPORT_S:
          self.down_reported.add(name)
          lines.append({'kind': 'process', 'name': name, 'what': 'still down', 'exit_code': code,
                        'meaning': exit_meaning(code), 'down_s': round(t - self.down_since[name], 1)})
      else:
        self.down_since.pop(name, None)
        self.down_reported.discard(name)
    return lines

  def error(self, t: float, record: dict | None) -> dict | None:
    if record is None:
      return None
    key = (record.get('daemon', ''), record.get('msg', '')[:80])
    slot = self.error_logged.setdefault(key, [-1e18, 0])
    if t - slot[0] < ERROR_REPEAT_S:
      slot[1] += 1
      return None
    repeats, slot[0], slot[1] = slot[1], t, 0
    if repeats:
      record = {**record, 'repeats_since_last': repeats}
    return record

  def snapshot(self) -> dict:
    cores_now = self.cores[-1] if self.cores else []
    cores_max = [max(c[i] for c in self.cores) for i in range(len(cores_now))] if self.cores else []
    top = []
    if len(self.procs) >= 2:
      (t0, r0), (t1, r1) = self.procs[-3] if len(self.procs) >= 3 else self.procs[0], self.procs[-1]
      top = busiest(r0, r1, t1 - t0)
    return {'cores_now': cores_now, 'cores_max_10s': cores_max, 'temp_c': self.temp, 'memory_pct': self.memory,
            'busiest_last_4s': top}


def exit_meaning(code: int) -> str:
  """What a process exit code says: negative is the signal that killed it (multiprocessing convention)."""
  try:
    code = int(code)
  except (TypeError, ValueError):
    return ''
  if code < 0:
    import signal
    try:
      return f'killed by {signal.Signals(-code).name}' + (' (out of memory or watchdog?)' if -code == 9 else '')
    except ValueError:
      return f'killed by signal {-code}'
  if code > 128:
    return exit_meaning(128 - code)
  return 'clean exit' if code == 0 else f'exited with error {code}'


def _avg_cores(samples: list[list[float]]) -> list[float]:
  if not samples:
    return []
  n = min(len(s) for s in samples)
  return [round(sum(s[i] for s in samples) / len(samples), 1) for i in range(n)]


class Writer:
  def __init__(self, root: Path):
    self.root = root
    self.day = None

  def write(self, line: dict, t_mono: float) -> None:
    day = time.strftime('%Y-%m-%d')
    if day != self.day:
      self.day = day
      self.prune()
    line = {'time': time.strftime('%H:%M:%S'), 'mono': round(t_mono, 2), **line}
    self.root.mkdir(parents=True, exist_ok=True)
    with open(self.root / f'{day}.jsonl', 'a') as f:
      f.write(json.dumps(line, separators=(',', ':'), default=str) + '\n')

  def prune(self, keep_days: int = KEEP_DAYS) -> None:
    """Keep the newest keep_days daily files (named YYYY-MM-DD, so name order is date order)."""
    files = sorted(self.root.glob('*.jsonl')) if self.root.exists() else []
    for path in files[:-keep_days] if keep_days > 0 else files:
      try:
        path.unlink()
      except OSError:
        pass


def main() -> None:
  import openpilot.cereal.messaging as messaging
  from openpilot.starpilot.vision_cpu import LITTLE_CORES

  try:
    os.nice(19)
    os.sched_setaffinity(0, list(LITTLE_CORES))
  except (AttributeError, OSError):
    pass
  writer = Writer(trace_root())
  trace = Trace()
  logs = messaging.sub_sock('logMessage', conflate=False)
  device = messaging.sub_sock('deviceState', conflate=True)
  procs = messaging.sub_sock('procLog', conflate=True)
  events = messaging.sub_sock('onroadEvents', conflate=False)
  errors = messaging.sub_sock('errorLogMessage', conflate=False)
  manager = messaging.sub_sock('managerState', conflate=True)
  while True:
    now = time.monotonic()
    for raw in messaging.drain_sock_raw(logs):
      if FEATURE_MARK not in raw:
        continue
      try:
        record = feature_record(messaging.log_from_bytes(raw).logMessage)
        if record is not None:
          writer.write(record, now)
      except Exception:
        pass
    try:
      raw = device.receive(non_blocking=True)
      if raw is not None:
        d = messaging.log_from_bytes(raw).deviceState
        trace.device(now, list(d.cpuUsagePercent), max(d.cpuTempC) if len(d.cpuTempC) else None, d.memoryUsagePercent)
      raw = procs.receive(non_blocking=True)
      if raw is not None:
        line = trace.proc(now, cpu_reading(messaging.log_from_bytes(raw).procLog))
        if line is not None:
          writer.write(line, now)
      for raw in messaging.drain_sock_raw(events):
        line = trace.events(now, alert_names(messaging.log_from_bytes(raw).onroadEvents))
        if line is not None:
          writer.write(line, now)
    except Exception:
      pass
    try:
      for raw in messaging.drain_sock_raw(errors):
        line = trace.error(now, error_record(messaging.log_from_bytes(raw).errorLogMessage))
        if line is not None:
          writer.write(line, now)
      raw = manager.receive(non_blocking=True)
      if raw is not None:
        states = [(str(p.name), bool(p.running), bool(p.shouldBeRunning), int(p.pid), int(p.exitCode))
                  for p in messaging.log_from_bytes(raw).managerState.processes]
        for line in trace.processes(now, states):
          writer.write(line, now)
    except Exception:
      pass
    time.sleep(LOOP_S)


if __name__ == '__main__':
  main()
