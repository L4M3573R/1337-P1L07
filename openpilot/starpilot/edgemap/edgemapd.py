#!/usr/bin/env python3
"""Onroad edge map recorder: observe only, never publishes control. Ancillary: it cannot block engagement.

Samples road edges, car motion and GPS at 5 Hz on the model cadence; checkpoints the drive every minute and on shutdown.
"""
from __future__ import annotations

import os
import signal
import time

from openpilot.starpilot.edgemap.recorder import DriveLog, sample_row

SERVICES = ['modelV2', 'carState', 'carControl', 'deviceMotion', 'gpsLocationExternal']
CHECKPOINT_S = 60.0


def drive_id() -> str:
  return f"{time.strftime('%Y%m%d-%H%M%S')}-{os.getpid()}"


def main() -> None:
  import openpilot.cereal.messaging as messaging

  os.nice(10)
  sm = messaging.SubMaster(SERVICES, poll='modelV2')
  log = DriveLog(drive_id())
  last_checkpoint = time.monotonic()

  def stop(*_):
    raise SystemExit(0)
  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  try:
    while True:
      sm.update()
      if sm.updated['modelV2'] and sm.seen['carState']:
        t = sm.logMonoTime['modelV2'] / 1e9
        gps_seen = sm.seen['gpsLocationExternal']
        gps_age = t - sm.logMonoTime['gpsLocationExternal'] / 1e9 if gps_seen else float('inf')
        log.add(sample_row(t, sm['modelV2'], sm['carState'],
                           sm['carControl'] if sm.seen['carControl'] else None,
                           sm['deviceMotion'] if sm.seen['deviceMotion'] else None,
                           sm['gpsLocationExternal'] if gps_seen else None, gps_age))
      now = time.monotonic()
      if now - last_checkpoint >= CHECKPOINT_S:
        last_checkpoint = now
        log.save()
  finally:
    log.save()


if __name__ == "__main__":
  main()
