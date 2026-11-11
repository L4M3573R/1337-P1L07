#!/usr/bin/env python3
"""Onroad energy ledger: observe only, never publishes control. Ancillary: it cannot block engagement.

Samples at the 20 Hz radarState cadence, checkpoints the drive file every minute and on shutdown.
"""
from __future__ import annotations

import os
import signal
import time
from pathlib import Path

from openpilot.starpilot.efficiency import store
from openpilot.starpilot.efficiency.ledger import Ledger, make_sample

SERVICES = ['radarState', 'carState', 'carControl', 'selfdriveState', 'livePose']
CHECKPOINT_S = 60.0


class Recorder:
  def __init__(self, mass: float, drive_id: str, root: Path | None = None, checkpoint_s: float = CHECKPOINT_S):
    self.mass, self.drive_id, self.root, self.checkpoint_s = mass, drive_id, root, checkpoint_s
    self.ledger = Ledger()
    self.last_checkpoint = None
    self.last_t = None

  def observe(self, t: float, car_state, car_control=None, radar_state=None, selfdrive_state=None, live_pose=None) -> None:
    if self.last_t is not None and t <= self.last_t:
      return  # the same carState seen again on the next radarState wake-up
    self.last_t = t
    self.ledger.add(make_sample(t, self.mass, car_state, car_control, radar_state, selfdrive_state, live_pose))

  def maybe_checkpoint(self, now: float) -> bool:
    if self.last_checkpoint is None:
      self.last_checkpoint = now
    if now - self.last_checkpoint < self.checkpoint_s or not self.ledger.stretches:
      return False
    self.last_checkpoint = now
    store.save_drive(self.drive_id, list(self.ledger.stretches), self.root)
    return True

  def finish(self) -> None:
    stretches = self.ledger.finish()
    if stretches:
      store.save_drive(self.drive_id, stretches, self.root)


def drive_id(now: float | None = None) -> str:
  # Wall-clock name for humans; uniqueness comes from the pid if the clock is not yet set.
  stamp = time.strftime("%Y%m%d-%H%M%S") if now is None else time.strftime("%Y%m%d-%H%M%S", time.localtime(now))
  return f"{stamp}-{os.getpid()}"


def main() -> None:
  import openpilot.cereal.messaging as messaging
  from opendbc.car.structs import car
  from openpilot.common.params import Params

  os.nice(10)
  params = Params()
  cp = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  sm = messaging.SubMaster(SERVICES, poll='radarState')
  recorder = Recorder(float(cp.mass), drive_id())

  def stop(*_):
    raise SystemExit(0)
  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  try:
    while True:
      sm.update()
      if sm.seen['carState']:
        optional = {name: (sm[name] if sm.seen[name] else None) for name in ('carControl', 'radarState', 'selfdriveState', 'livePose')}
        recorder.observe(sm.logMonoTime['carState'] / 1e9, sm['carState'], optional['carControl'], optional['radarState'],
                         optional['selfdriveState'], optional['livePose'])
      recorder.maybe_checkpoint(time.monotonic())
  finally:
    recorder.finish()


if __name__ == "__main__":
  main()
