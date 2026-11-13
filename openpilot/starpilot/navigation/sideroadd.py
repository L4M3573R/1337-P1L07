#!/usr/bin/env python3
"""Blinker-turn road lookups in their own process, so the tile reads and road search never run inside modeld.

modeld (MapTurnFinder(remote=True)) writes a small request to shared memory at most every LOOKUP_PERIOD_S while a
blinker is on; this process answers each new request with the side road ahead and writes the result back.
Observe only: it publishes no control. Ancillary: it cannot block engagement.
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


def answer(finder: MapTurnFinder, request: dict) -> Lookup:
  t, side = float(request['t']), str(request['side'])
  try:
    return finder.lookup_now(t, float(request['latitude']), float(request['longitude']), float(request['bearing']), side)
  except Exception:  # a bad tile or store must never take this process down
    return Lookup(t, side, None, False)


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


def main() -> None:
  os.nice(10)
  finder = MapTurnFinder()

  def stop(*_):
    raise SystemExit(0)
  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  last = 0.0
  while True:
    last = step(finder, last)
    time.sleep(POLL_S)


if __name__ == '__main__':
  main()
