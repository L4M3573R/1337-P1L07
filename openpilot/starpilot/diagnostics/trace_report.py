#!/usr/bin/env python3
"""Readable timeline of a drive from its rlogs, for tracing what happened and why.

  python -m openpilot.starpilot.diagnostics.trace_report <folder> <route> [--from M:SS] [--to M:SS] [--all]

<folder> holds segment folders (<route>--<n>/rlog.zst), for example the PC footage copy or the DR4L rlogs;
<route> is the route name (e.g. 00000072--1447e3e7bd). Times are counted from the first segment found.
Only changes are printed, each with the time into the drive and the speed:
  engagement, cruise, alerts and disengaging events, bookmarks, brake/gas presses, steering-wheel button actions
  (and what they did), Traffic Mode, experimental mode, Pulse and Glide, the CEM stop detector, every decision-log
  event (lead loss, Pulse and Glide reasons, turns, curve guard, NNLC, slow lookups, ...), and seconds where the
  100 Hz safety loop ran slow, with the busiest processes from the nearest CPU sample.
--all also prints the frequent radar fusion decisions.
"""
from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path

MPH = 2.237
WHEEL_ACTIONS = {2: 'Force Coast toggle', 5: 'cycle conditional mode', 6: 'Traffic Mode toggle', 7: 'Switchback Mode',
                 14: 'Pulse and Glide toggle'}
NOISY_FEATURES = frozenset({'tandem_fusion'})
SLOW_LOOP_HZ = 95


def clock(seconds: float) -> str:
  sign = '-' if seconds < 0 else ''
  seconds = abs(seconds)
  return f'{sign}{int(seconds // 60)}:{seconds % 60:04.1f}'


def parse_clock(text: str | None) -> float | None:
  if text is None:
    return None
  if ':' in text:
    minutes, seconds = text.split(':', 1)
    return int(minutes) * 60 + float(seconds)
  return float(text)


def feature_text(msg: dict) -> str:
  fields = ', '.join(f'{k}={v}' for k, v in msg.items() if k not in ('event', 'feature'))
  return f"[{msg.get('feature')}] {fields}"


def segments(folder: Path, route: str) -> list[Path]:
  paths = [Path(p) for p in glob.glob(str(folder / f'{route}--*' / 'rlog.zst'))]
  return sorted(paths, key=lambda p: int(p.parent.name.rsplit('--', 1)[1]))


class Timeline:
  def __init__(self, show_all: bool = False):
    self.show_all = show_all
    self.start = None
    self.state: dict = {}
    self.speed = 0.0
    self.lines: list[tuple[float, str]] = []
    self.counts: dict[int, int] = {}
    self.second_speed: dict[int, float] = {}
    self.procs: list[tuple[float, dict]] = []

  def emit(self, t: float, text: str, speed: float | None = None) -> None:
    speed = self.speed if speed is None else speed
    self.lines.append((t - self.start, f'{speed * MPH:5.1f} mph  {text}'))

  def change(self, t: float, key: str, value, text: str) -> None:
    if self.state.get(key, object()) != value:
      if key in self.state:
        self.emit(t, text)
      self.state[key] = value

  def message(self, m) -> None:
    t = m.logMonoTime / 1e9
    if self.start is None:
      self.start = t
    w = m.which()
    if w == 'carState':
      cs = m.carState
      self.speed = float(cs.vEgo)
      self.change(t, 'brake', bool(cs.brakePressed), 'brake pressed' if cs.brakePressed else 'brake released')
      self.change(t, 'gas', bool(cs.gasPressed), 'gas pressed' if cs.gasPressed else 'gas released')
      self.change(t, 'cruise', bool(cs.cruiseState.enabled), f"cruise {'engaged' if cs.cruiseState.enabled else 'off'}")
    elif w == 'carControl':
      cc = m.carControl
      self.change(t, 'long', bool(cc.longActive), f"comma speed control {'ON' if cc.longActive else 'off'}")
      self.change(t, 'lat', bool(cc.latActive), f"comma steering {'ON' if cc.latActive else 'off'}")
    elif w == 'selfdriveState':
      ss = m.selfdriveState
      self.change(t, 'alert', (ss.alertText1, ss.alertText2),
                  f'ALERT "{ss.alertText1}" {ss.alertText2!r}' if ss.alertText1 else 'alert cleared')
      self.change(t, 'experimental', bool(ss.experimentalMode), f"experimental mode {'on' if ss.experimentalMode else 'off (chill)'}")
      self._rate(t)
    elif w == 'onroadEvents':
      names = sorted(str(e.name) for e in m.onroadEvents if e.immediateDisable or e.softDisable)
      self.change(t, 'disable_events', names, f"disengaging events: {', '.join(names) or 'none'}")
    elif w == 'bookmarkButton':
      self.emit(t, 'BOOKMARK')
    elif w == 'slcCruiseEvent':
      action = m.slcCruiseEvent.wheelAction
      if action.actionCode:
        self.emit(t, f'wheel button {action.buttonKey or "?"} -> {WHEEL_ACTIONS.get(action.actionCode, f"action {action.actionCode}")}')
    elif w == 'slcState':
      tm = m.slcState.trafficMode
      self.change(t, 'traffic', (bool(tm.effective), str(tm.reason)), f'Traffic Mode effective={tm.effective} ({tm.reason})')
      cm = m.slcState.conditionalMode
      self.change(t, 'cem_reason', str(cm.reason), f'CEM decision: {cm.reason} (experimental={cm.experimental})')
    elif w == 'starpilotLongitudinalPlan':
      p = m.starpilotLongitudinalPlan
      self.change(t, 'pg', bool(p.pulseGlideEnabled), f"Pulse and Glide {'ON' if p.pulseGlideEnabled else 'OFF'}")
      self.change(t, 'cem_light', bool(p.cemStopLightDetected), f"CEM stop light {'detected' if p.cemStopLightDetected else 'cleared'}")
      self.change(t, 'cem_model', bool(p.cemStopModelStopping), f"CEM: model {'plans a stop' if p.cemStopModelStopping else 'no longer stopping'}")
    elif w == 'logMessage':
      text = m.logMessage
      if 'starpilot_feature' not in text:
        return
      try:
        msg = json.loads(text).get('msg')
      except ValueError:
        return
      if isinstance(msg, dict) and msg.get('event') == 'starpilot_feature' and (self.show_all or msg.get('feature') not in NOISY_FEATURES):
        self.emit(t, feature_text(msg))
    elif w == 'procLog':
      self.procs.append((t, {int(p.pid): (float(p.cpuUser + p.cpuSystem), ' '.join(p.cmdline)[-48:] or p.name) for p in m.procLog.procs}))

  def _rate(self, t: float) -> None:
    if self.state.get('lat'):
      second = int(t)
      self.counts[second] = self.counts.get(second, 0) + 1
      self.second_speed[second] = self.speed

  def finish(self) -> None:
    """Seconds where the safety loop ran slow, counting only whole seconds (data on both sides): the first and last
    partial second of a segment, or around a missing one, are not slow seconds."""
    for second, count in sorted(self.counts.items()):
      if count < SLOW_LOOP_HZ and second - 1 in self.counts and second + 1 in self.counts:
        self.emit(second + 1.0, f'safety loop slow: {count}/100 per second{self._busiest(second + 1.0)}',
                  self.second_speed.get(second))

  def _busiest(self, t: float) -> str:
    before = [p for p in self.procs if p[0] <= t]
    if len(before) < 2:
      return ''
    (t0, r0), (t1, r1) = before[-2], before[-1]
    use = sorted(((name, (r1[pid][0] - r0[pid][0]) / (t1 - t0) * 100) for pid, (_, name) in r1.items() if pid in r0),
                 key=lambda x: -x[1])[:4]
    return '; busiest: ' + ', '.join(f'{name.split()[-1].rsplit(".", 1)[-1]} {pct:.0f}%' for name, pct in use)


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('folder', type=Path)
  parser.add_argument('route')
  parser.add_argument('--from', dest='start', default=None, help='M:SS into the drive')
  parser.add_argument('--to', dest='end', default=None, help='M:SS into the drive')
  parser.add_argument('--all', action='store_true', help='include frequent radar fusion decisions')
  args = parser.parse_args()
  from openpilot.tools.lib.logreader import LogReader
  paths = segments(args.folder, args.route)
  if not paths:
    sys.exit(f'no {args.route}--<n>/rlog.zst under {args.folder}')
  timeline = Timeline(args.all)
  for path in paths:
    try:
      for m in LogReader(str(path)):
        timeline.message(m)
    except Exception as e:  # a cut-off last segment still yields what it has
      print(f'(segment {path.parent.name}: {e})', file=sys.stderr)
  timeline.finish()
  start, end = parse_clock(args.start), parse_clock(args.end)
  for t, line in sorted(timeline.lines, key=lambda x: x[0]):
    if (start is None or t >= start) and (end is None or t <= end):
      print(f'{clock(t):>8}  {line}')


if __name__ == '__main__':
  main()
