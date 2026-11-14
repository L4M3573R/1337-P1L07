#!/usr/bin/env python3
"""Onroad edge map recorder: observe only, never publishes control. Ancillary: it cannot block engagement.

Samples road edges, car motion and GPS at 5 Hz on the model cadence; checkpoints the drive every minute and on shutdown.
"""
from __future__ import annotations

import os
import signal
import time

from openpilot.starpilot.diagnostics.features import EVENT, log_error
from openpilot.starpilot.edgemap.recorder import DriveLog, sample_row

SERVICES = ['modelV2', 'carState', 'carControl', 'deviceMotion', 'gpsLocationExternal']
CHECKPOINT_S = 60.0
ANNOUNCE_AFTER_S = 10.0   # the start event waits until the trace daemon (started alongside) is listening


def drive_id() -> str:
  return f"{time.strftime('%Y%m%d-%H%M%S')}-{os.getpid()}"


def main() -> None:
  import openpilot.cereal.messaging as messaging

  os.nice(10)
  sm = messaging.SubMaster(SERVICES, poll='modelV2')
  log = DriveLog(drive_id())
  last_checkpoint = time.monotonic()

  from openpilot.common.swaglog import cloudlog
  started = time.monotonic()
  stats = {'samples_failed': 0, 'saves_failed': 0, 'no_gps_samples': 0}
  announced = False

  def save() -> None:
    try:
      log.save()
    except Exception as e:
      stats['saves_failed'] += 1
      log_error('edge_map', e, where='save')

  def stop(*_):
    raise SystemExit(0)
  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  try:
    while True:
      sm.update()
      if sm.updated['modelV2'] and sm.seen['carState']:
        try:
          t = sm.logMonoTime['modelV2'] / 1e9
          gps_seen = sm.seen['gpsLocationExternal']
          gps_age = t - sm.logMonoTime['gpsLocationExternal'] / 1e9 if gps_seen else float('inf')
          added = log.add(sample_row(t, sm['modelV2'], sm['carState'],
                                     sm['carControl'] if sm.seen['carControl'] else None,
                                     sm['deviceMotion'] if sm.seen['deviceMotion'] else None,
                                     sm['gpsLocationExternal'] if gps_seen else None, gps_age))
          if added and gps_age > 2.0:
            stats['no_gps_samples'] += 1
        except Exception as e:   # one bad frame must not end the drive's recording
          stats['samples_failed'] += 1
          log_error('edge_map', e, where='sample')
      now = time.monotonic()
      if not announced and now - started >= ANNOUNCE_AFTER_S:
        announced = True
        cloudlog.event(EVENT, feature='edge_map', what='recording', drive=log.drive_id, rows=len(log.rows), **stats)
      if now - last_checkpoint >= CHECKPOINT_S:
        last_checkpoint = now
        save()
  finally:
    save()
    try:
      cloudlog.event(EVENT, feature='edge_map', what='stopped', drive=log.drive_id, rows=len(log.rows),
                     minutes=round((time.monotonic() - started) / 60, 1), **stats)
    except Exception:
      pass


if __name__ == "__main__":
  main()
