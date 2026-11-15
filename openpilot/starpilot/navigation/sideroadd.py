#!/usr/bin/env python3
"""Blinker-turn road lookups in their own process, so the tile reads and road search never run inside modeld.

modeld (MapTurnFinder(remote=True)) writes a small request to shared memory at most every LOOKUP_PERIOD_S while a
blinker is on; this process answers each new request with the side road ahead and writes the result back.
Observe only: it publishes no control. Ancillary: it cannot block engagement.

It also runs the no-blinker straight guard in shadow (straight_guard): a log of where the model turned onto a mapped side
road without a blinker while the road went on straight. That never delays answering modeld.
"""
from __future__ import annotations

import json
import os
import signal
import time

from openpilot.starpilot.navigation.blinker_turns import (
  REQUEST_PATH, RESULT_PATH, Lookup, MapTurnFinder, _write_json, lookup_to_doc,
)

POLL_S = 0.05


SLOW_LOOKUP_S = 0.3  # a blinker-turn road search slower than this is written to the decision log


def answer(finder: MapTurnFinder, request: dict) -> Lookup:
  t, side = float(request['t']), str(request['side'])
  started = time.monotonic()
  try:
    return finder.lookup_now(t, float(request['latitude']), float(request['longitude']), float(request['bearing']), side)
  except Exception:  # a bad tile or store must never take this process down
    return Lookup(t, side, None, False)
  finally:
    from openpilot.starpilot.diagnostics.features import log_if_slow
    log_if_slow('side_road_lookup', time.monotonic() - started, SLOW_LOOKUP_S, side=side)


def step(finder: MapTurnFinder, last_seen: float, request_path=REQUEST_PATH, result_path=RESULT_PATH) -> float:
  """Answer the request if it is new; returns the request time last answered."""
  try:
    request = json.loads(request_path.read_text())
    t = float(request['t'])
  except (OSError, ValueError, KeyError, TypeError):
    return last_seen
  if t <= last_seen:
    return last_seen
  _write_json(result_path, lookup_to_doc(answer(finder, request)))
  return t


def guard_step(guard, sm, finder: MapTurnFinder) -> None:
  """One shadow straight-guard step on the latest messages; any failure is logged, never raised into the loop."""
  from openpilot.starpilot.diagnostics.features import log_error
  from openpilot.starpilot.navigation.straight_guard import sample_from
  try:
    sm.update(0)
    if not sm.seen['carState'] or not sm.seen['modelV2']:
      return
    guard.step(sample_from(sm, time.monotonic_ns()), finder.crossing_now)
  except Exception as e:  # the shadow log must never stop blinker-turn answers
    log_error('straight_guard', e, where='sideroadd')


def main() -> None:
  os.nice(10)
  finder = MapTurnFinder()
  from openpilot.cereal import messaging
  from openpilot.starpilot.gps.source import GPS_SERVICES
  from openpilot.starpilot.navigation.straight_guard import StraightGuardShadow
  sm = messaging.SubMaster(['carState', 'carControl', 'modelV2', *GPS_SERVICES],
                           ignore_alive=list(GPS_SERVICES), ignore_valid=list(GPS_SERVICES), ignore_avg_freq=list(GPS_SERVICES))
  guard = StraightGuardShadow()

  def stop(*_):
    raise SystemExit(0)
  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  last = 0.0
  while True:
    last = step(finder, last)
    guard_step(guard, sm, finder)
    time.sleep(POLL_S)


if __name__ == '__main__':
  main()
