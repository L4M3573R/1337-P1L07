#!/usr/bin/env python3
"""Onroad energy ledger: observe only, never publishes control. Ancillary: it cannot block engagement.

Samples at the 20 Hz radarState cadence, checkpoints the drive file every minute and on shutdown.
Real fuel comes from the dash average MPG since the last fill-up (tank.py); the hybrid power flow and
the cluster readings come from the newest CAN batch only (conflated), which keeps this process light.
"""
from __future__ import annotations

import math
import os
import signal
import time
from pathlib import Path

from openpilot.starpilot.efficiency import store, tank as tank_store
from openpilot.starpilot.efficiency.hybrid import Cluster, PowerFlow, scan
from openpilot.starpilot.efficiency.ledger import Ledger, make_sample

SERVICES = ['radarState', 'carState', 'carControl', 'selfdriveState', 'deviceMotion', 'starpilotLongitudinalPlan']
CHECKPOINT_S = 60.0
METERS_PER_MILE = 1609.344


class Recorder:
  def __init__(self, mass: float, drive_id: str, root: Path | None = None, checkpoint_s: float = CHECKPOINT_S,
               tank_path: Path | None = None):
    self.mass, self.drive_id, self.root, self.checkpoint_s = mass, drive_id, root, checkpoint_s
    self.tank_path = tank_path if tank_path is not None else (root / "tank.json" if root is not None else None)
    self.ledger = Ledger()
    self.tank = tank_store.load(self.tank_path)
    self.flow: PowerFlow | None = None
    self.last_avg_mpg: float | None = None
    self.fuel_gal: float | None = None
    self.last_checkpoint = None
    self.last_t = None

  def observe_can(self, flow: PowerFlow | None, cluster: Cluster | None) -> None:
    if flow is not None:
      self.flow = flow
    if cluster is not None:
      if self.tank.observe_range(cluster.range_mi):
        self.last_avg_mpg = None  # a fill-up: the dash average starts over
      # Gallons so far = miles since the fill-up / dash average. The dash refreshes its average about
      # every 10 s; between refreshes this advances at the current average, and each refresh corrects it.
      self.last_avg_mpg = cluster.avg_mpg
      self.fuel_gal = self.tank.gallons_used(cluster.avg_mpg)

  def observe(self, t: float, car_state, car_control=None, radar_state=None, selfdrive_state=None, device_motion=None,
              pulse_glide: str | None = None) -> None:
    if self.last_t is not None and t <= self.last_t:
      return  # the same carState seen again on the next radarState wake-up
    if self.last_t is not None and t - self.last_t < 1.0 and math.isfinite(car_state.vEgo):
      self.tank.drive(max(car_state.vEgo, 0.0) * (t - self.last_t) / METERS_PER_MILE)
    self.last_t = t
    self.ledger.add(make_sample(t, self.mass, car_state, car_control, radar_state, selfdrive_state, device_motion,
                                fuel_gal=self.fuel_gal, power_flow=self.flow, pulse_glide=pulse_glide))

  def save(self) -> None:
    if self.ledger.stretches:
      store.save_drive(self.drive_id, list(self.ledger.stretches), self.root, self.ledger.flow_hist)
    tank_store.save(self.tank, self.tank_path)

  def maybe_checkpoint(self, now: float) -> bool:
    if self.last_checkpoint is None:
      self.last_checkpoint = now
    if now - self.last_checkpoint < self.checkpoint_s or not self.ledger.stretches:
      return False
    self.last_checkpoint = now
    self.save()
    return True

  def finish(self) -> None:
    self.ledger.finish()
    self.save()


def drive_id(now: float | None = None) -> str:
  # Wall-clock name for humans; uniqueness comes from the pid if the clock is not yet set.
  stamp = time.strftime("%Y%m%d-%H%M%S") if now is None else time.strftime("%Y%m%d-%H%M%S", time.localtime(now))
  return f"{stamp}-{os.getpid()}"


def pulse_glide_phase(sm) -> str | None:
  if not sm.seen['starpilotLongitudinalPlan']:
    return None
  plan = sm['starpilotLongitudinalPlan']
  return str(plan.pulseGlidePhase) if plan.pulseGlideEnabled else None


def main() -> None:
  import openpilot.cereal.messaging as messaging
  from opendbc.car.structs import car
  from openpilot.common.params import Params

  os.nice(10)
  params = Params()
  cp = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  sm = messaging.SubMaster(SERVICES, poll='radarState')
  can_sock = messaging.sub_sock('can', conflate=True)
  recorder = Recorder(float(cp.mass), drive_id())

  def stop(*_):
    raise SystemExit(0)
  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  try:
    while True:
      sm.update()
      raw = can_sock.receive(non_blocking=True)
      if raw is not None:
        event = messaging.log_from_bytes(raw)
        recorder.observe_can(*scan((c.address, bytes(c.dat), c.src) for c in event.can))
      if sm.seen['carState']:
        optional = {name: (sm[name] if sm.seen[name] else None) for name in ('carControl', 'radarState', 'selfdriveState', 'deviceMotion')}
        recorder.observe(sm.logMonoTime['carState'] / 1e9, sm['carState'], optional['carControl'], optional['radarState'],
                         optional['selfdriveState'], optional['deviceMotion'], pulse_glide_phase(sm))
      recorder.maybe_checkpoint(time.monotonic())
  finally:
    recorder.finish()


if __name__ == "__main__":
  main()
