#!/usr/bin/env python3
"""Onroad energy ledger: observe only, never publishes control. Ancillary: it cannot block engagement.

Samples at the 20 Hz radarState cadence, checkpoints the drive file every minute and on shutdown.
Real fuel comes from the dash average MPG since the last fill-up (tank.py); the hybrid power flow and
the cluster readings come from the newest CAN batch only, which keeps this process light; older batches are only
checked for the drive-mode signals.
When the BlueDriver logger runs, its measured fuel flow and hybrid battery charge are read once a second
from its live file and kept alongside the dash figure.
"""
from __future__ import annotations

import math
import os
import signal
import time
from dataclasses import replace
from pathlib import Path

from openpilot.starpilot.diagnostics.features import EVENT, ChangeLog, log_error
from openpilot.starpilot.efficiency import store, tank as tank_store
from openpilot.starpilot.efficiency.experiment import BUDGET_FILE, read_budget_totals, read_tag
from openpilot.starpilot.efficiency.car_signals import CABIN_ADDR, CLIMATE_ADDR, MDPS11_ADDR, MODE_ADDR, CarSignals
from openpilot.starpilot.efficiency.hybrid import Cluster, PowerFlow, scan
from openpilot.starpilot.efficiency.ledger import Ledger, make_sample
from openpilot.starpilot.obd.bluedriver import GRAMS_PER_GALLON

SERVICES = ['radarState', 'carState', 'carControl', 'selfdriveState', 'deviceMotion', 'starpilotLongitudinalPlan', 'gpsLocationExternal']
GPS_MAX_ACCURACY_M = 25.0
CHECKPOINT_S = 60.0
OBD_READ_S = 1.0
ENGINE_ON_RPM = 50.0  # the BlueDriver's rpm at or above this is the engine running; 0 is electric
SIGNAL_ADDRS = (MODE_ADDR, MDPS11_ADDR, CLIMATE_ADDR, CABIN_ADDR)  # short announcements and 1 Hz frames a single batch can miss
METERS_PER_MILE = 1609.344
ANNOUNCE_AFTER_S = 10.0   # the start event waits until the trace daemon (started alongside) is listening


class Recorder:
  def __init__(self, mass: float, drive_id: str | None, root: Path | None = None, checkpoint_s: float = CHECKPOINT_S,
               tank_path: Path | None = None, speed_factor: float = 1.0):
    self.mass, self.drive_id, self.root, self.checkpoint_s = mass, drive_id, root, checkpoint_s
    self.speed_factor = speed_factor   # CarParams.wheelSpeedFactor: the scale this drive's speeds and miles are on
    self.tank_path = tank_path if tank_path is not None else (root / "tank.json" if root is not None else None)
    self.ledger = Ledger()
    self.tank = tank_store.load(self.tank_path)
    self._fill_pending = False  # a fill-up seen before this drive had its id: stamped at the first save
    self.flow: PowerFlow | None = None
    self.last_avg_mpg: float | None = None
    self.fuel_gal: float | None = None
    self.last_checkpoint = None
    self.last_t = None
    self.obd_fuel_gal: float | None = None
    self.soc: float | None = None
    self.obd_engine: bool | None = None
    self.signals = CarSignals()
    self.ambient_c: float | None = None
    self.coolant_c: float | None = None
    self.fuel_level: float | None = None
    self.position: tuple[float, float, float] | None = None  # lat, lon, bearing
    self.experiment: str | None = None  # the fuel experiment in force (plannerd's status, read once a second)
    self._obd_prev: tuple[str, float] | None = None
    self._obd_total_g = 0.0

  def observe_obd(self, doc: dict | None) -> None:
    """The BlueDriver live document (or None): keeps a fuel counter that never jumps when the logger
    restarts or reconnects, and is unknown (None) while the link is down."""
    def number(name, lo, hi):
      value = doc.get(name) if doc else None
      return float(value) if isinstance(value, (int, float)) and lo <= value <= hi else None
    self.soc = number('soc', 0.0, 100.0)
    rpm = number('rpm', 0.0, 10000.0)
    self.obd_engine = None if not doc or not doc.get('connected') or rpm is None else rpm >= ENGINE_ON_RPM
    self.ambient_c = number('ambient', -40.0, 60.0)
    self.coolant_c = number('coolant', -40.0, 150.0)
    self.fuel_level = number('fuel_level', 0.0, 100.0)
    if self.fuel_level is not None and doc.get('connected'):
      before = replace(self.tank)
      if self.tank.observe_level(self.fuel_level):
        self.last_avg_mpg = None  # a fill-up: the dash average starts over
        self.fuel_gal = 0.0 if self.tank.miles_since_fill is not None else None
        self._filled(before)
    grams = doc.get('fuel_g') if doc else None
    if not doc or not doc.get('connected') or not isinstance(grams, (int, float)) or grams < 0:
      self.obd_fuel_gal, self._obd_prev = None, None
      return
    session = str(doc.get('drive'))
    if self._obd_prev is not None and self._obd_prev[0] == session and grams >= self._obd_prev[1]:
      self._obd_total_g += grams - self._obd_prev[1]
    self._obd_prev = (session, float(grams))
    self.obd_fuel_gal = self._obd_total_g / GRAMS_PER_GALLON

  def observe_can(self, flow: PowerFlow | None, cluster: Cluster | None) -> None:
    if flow is not None:
      self.flow = flow
    if cluster is not None:
      self.tank.observe_range(cluster.range_mi)   # recorded only: the range jumps 50+ mi on its own (fills: tank.py)
      # Gallons so far = miles since the fill-up / dash average. The dash refreshes its average about
      # every 10 s; between refreshes this advances at the current average, and each refresh corrects it.
      self.last_avg_mpg = cluster.avg_mpg
      self.fuel_gal = self.tank.gallons_used(cluster.avg_mpg)

  def _filled(self, before) -> None:
    """A fill-up: keep the tank that ended ("comma vs you" per tank), and start the new one at this drive."""
    try:
      from openpilot.starpilot.efficiency.compare import record_finished_tank
      record_finished_tank(self.root, before)
    except Exception as e:   # the history is a nicety; it must never cost the drive's ledger
      log_error('efficiency', e, where='tank history')
    self.tank.fill_drive = self.drive_id
    self._fill_pending = self.drive_id is None

  def observe_gps(self, gps) -> None:
    ok = (gps is not None and gps.hasFix and 0.0 < gps.horizontalAccuracy <= GPS_MAX_ACCURACY_M and
          math.isfinite(gps.latitude) and math.isfinite(gps.longitude))
    self.position = (gps.latitude, gps.longitude, gps.bearingDeg) if ok else None

  def tags(self) -> dict:
    lat, lon, bearing = self.position if self.position is not None else (None, None, None)
    return {'drive_mode': self.signals.drive_mode, 'tire_avg': self.signals.tire_avg, 'tire_min': self.signals.tire_min,
            'map_limit': self.signals.map_limit, 'road_code': self.signals.road_code, 'ambient_c': self.ambient_c,
            'fuel_level': self.fuel_level, 'lat': lat, 'lon': lon, 'bearing': bearing, 'experiment': self.experiment,
            'obd_engine': self.obd_engine, 'coolant_c': self.coolant_c, 'climate_set_f': self.signals.climate_set_f, 'cabin_f': self.signals.cabin_f}

  def observe(self, t: float, car_state, car_control=None, radar_state=None, selfdrive_state=None, device_motion=None,
              pulse_glide: str | None = None) -> None:
    if self.last_t is not None and t <= self.last_t:
      return  # the same carState seen again on the next radarState wake-up
    if self.last_t is not None and t - self.last_t < 1.0 and math.isfinite(car_state.vEgo):
      self.tank.drive(max(car_state.vEgo, 0.0) * (t - self.last_t) / METERS_PER_MILE)
    self.last_t = t
    self.ledger.add(make_sample(t, self.mass, car_state, car_control, radar_state, selfdrive_state, device_motion,
                                fuel_gal=self.fuel_gal, power_flow=self.flow, pulse_glide=pulse_glide,
                                obd_fuel_gal=self.obd_fuel_gal, soc=self.soc, **self.tags()))

  def save(self) -> None:
    try:
      if self.drive_id is None:
        # named at the first save (a minute in), once the clock has synced: at process start it can still read the
        # previous boot's date (drives on 2026-11-14 were named 20261012)
        self.drive_id = drive_id()
      if self._fill_pending:
        self.tank.fill_drive, self._fill_pending = self.drive_id, False
      if self.ledger.stretches:
        store.save_drive(self.drive_id, list(self.ledger.stretches), self.root, self.ledger.flow_hist, self.speed_factor)
      tank_store.save(self.tank, self.tank_path)
    except Exception as e:   # a failed checkpoint is retried next minute; it must not end the drive's ledger
      log_error('efficiency', e, where='save')

  def sources(self) -> dict:
    """Decision log key: which inputs the ledger has right now (a fuel source dropping out shows up here)."""
    return {'bluedriver_fuel': self.obd_fuel_gal is not None, 'dash_fuel': self.fuel_gal is not None,
            'hybrid_flow': self.flow is not None, 'gps': self.position is not None}

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


BUDGET_SAVE_S = 30.0


def start_judge(root):
  """Judge the stored drives now, in a child at the lowest priority on the little cores. It used to run when this
  process stopped, which a normal drive never reaches (the car turns off and the comma loses power), so the dials
  only moved when an install forced the comma offroad (review 2026-11-17). plannerd picks the result up when it lands."""
  try:
    import subprocess
    import sys
    return subprocess.Popen([sys.executable, '-m', 'openpilot.starpilot.efficiency.experiment_learn', str(root)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
  except Exception as e:
    log_error('efficiency', e, where='starting the judge')
    return None


def save_budget(root, totals) -> None:
  """plannerd's running time-budget totals, kept on disk every BUDGET_SAVE_S so the next drive carries them on."""
  if totals is None:
    return
  try:
    store._atomic_write(root / BUDGET_FILE, {'engaged_s': totals[0], 'spent_s': totals[1]})
  except OSError as e:
    log_error('efficiency', e, where='saving the time budget')


def main() -> None:
  import openpilot.cereal.messaging as messaging
  from opendbc.car.structs import car
  from openpilot.common.params import Params
  from openpilot.starpilot.obd.bluedriverd import read_live

  os.nice(10)
  params = Params()
  cp = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  sm = messaging.SubMaster(SERVICES, poll='radarState')
  can_sock = messaging.sub_sock('can', conflate=False)
  from openpilot.common.swaglog import cloudlog
  recorder = Recorder(float(cp.mass), None, speed_factor=float(cp.wheelSpeedFactor) or 1.0)
  judge = start_judge(store.default_root())
  last_budget_save = 0.0
  last_obd_read = 0.0
  sources_log = ChangeLog('efficiency_sources', min_gap_s=5.0)
  started = time.monotonic()
  loop_errors = 0
  announced = False

  def stop(*_):
    raise SystemExit(0)
  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  try:
    while True:
      sm.update()
      try:
        step(sm, can_sock, recorder, messaging)
      except Exception as e:   # one bad frame must not end the drive's ledger
        loop_errors += 1
        log_error('efficiency', e, where='main loop')
      now = time.monotonic()
      if now - last_obd_read >= OBD_READ_S:
        last_obd_read = now
        try:
          recorder.observe_obd(read_live(now=now))
        except Exception as e:
          log_error('efficiency', e, where='BlueDriver live file')
        recorder.experiment = read_tag(now)
        if now - last_budget_save >= BUDGET_SAVE_S:
          last_budget_save = now
          save_budget(store.default_root(), read_budget_totals(now))
        sources_log.observe(recorder.sources(), {'ledger_stretches': len(recorder.ledger.stretches)}, now=now)
      if not announced and now - started >= ANNOUNCE_AFTER_S:
        announced = True
        cloudlog.event(EVENT, feature='efficiency', what='recording', drive=recorder.drive_id, **recorder.sources())
      recorder.maybe_checkpoint(now)
  finally:
    recorder.finish()
    if judge is not None and judge.poll() is None:
      judge.terminate()
    try:
      cloudlog.event(EVENT, feature='efficiency', what='stopped', drive=recorder.drive_id,
                     stretches=len(recorder.ledger.stretches), minutes=round((time.monotonic() - started) / 60, 1),
                     loop_errors=loop_errors, **recorder.sources())
    except Exception:
      pass


def step(sm, can_sock, recorder: Recorder, messaging) -> None:
  """One radarState wake-up: CAN signals, GPS and the ledger sample (errors raise to the caller)."""
  # Every CAN batch is read for the drive-mode signals (the mode announcement lasts only a few frames); the
  # cluster scan still uses just the newest batch, as before.
  raws = messaging.drain_sock_raw(can_sock)
  for i, raw in enumerate(raws):
    event = messaging.log_from_bytes(raw)
    if i == len(raws) - 1:
      frames = [(c.address, bytes(c.dat), c.src) for c in event.can]
      recorder.observe_can(*scan(frames))
      recorder.signals.update(frames)
    else:
      recorder.signals.update([(c.address, bytes(c.dat), c.src) for c in event.can if c.address in SIGNAL_ADDRS])
  if sm.updated['gpsLocationExternal']:
    recorder.observe_gps(sm['gpsLocationExternal'])
  if sm.seen['carState']:
    optional = {name: (sm[name] if sm.seen[name] else None) for name in ('carControl', 'radarState', 'selfdriveState', 'deviceMotion')}
    recorder.observe(sm.logMonoTime['carState'] / 1e9, sm['carState'], optional['carControl'], optional['radarState'],
                     optional['selfdriveState'], optional['deviceMotion'], pulse_glide_phase(sm))


if __name__ == "__main__":
  main()
