#!/usr/bin/env python3
"""BlueDriver Pro logger: reads fuel flow, engine and hybrid battery PIDs over Bluetooth LE during drives.

Observe only: it never touches control or the comma's own CAN traffic, and it is ancillary, so a failure
here cannot block engagement. It starts querying only after the car interface is up (first carState), so
the comma's own startup diagnostics are finished first. Opt-in: <starpilot storage>/bluedriver/config.json
{"enabled": true}, set from Settings -> L4 custom. The scanner must already be bonded to the comma
(pairing is accepted only in a short window after the scanner powers up).

Outputs, under <starpilot storage>/bluedriver/:
  live.json         latest readings and the fuel burned this drive, rewritten every second for efficiencyd
  drives/<id>.csv   one row a second for later analysis (the newest MAX_DRIVE_LOGS are kept)
  health.json       once a drive: check-engine light, stored and pending trouble codes (read only, never cleared)
"""
from __future__ import annotations

import asyncio
import csv
import json
import os
import signal
import sys
import tempfile
import time
from pathlib import Path

from openpilot.starpilot.obd import bluedriver as bd

LIVE_VERSION = 1
MAX_DRIVE_LOGS = 200
QUERY_TIMEOUT_S = 1.0
INIT_TIMEOUT_S = 3.0
STARTUP_SETTLE_S = 5.0
BACKOFF_S = (5.0, 10.0, 30.0, 60.0)
CSV_FIELDS = ('t', 'rpm', 'maf', 'lambda', 'fuel_gps', 'fuel_g', 'torque_pct', 'soc', 'stft', 'ltft', 'load', 'coolant',
              'ambient', 'intake', 'volts', 'fuel_level', 'odometer_km', 'speed_kph', 'fuel_rate_direct', 'fuel_direct_g',
              'hv_volts')
HEALTH_TIMEOUT_S = 3.0
VENDOR = Path(__file__).resolve().parents[1] / 'galaxy_bluetooth' / 'vendor'
MAX_CONFIG_BYTES = 4096


def default_root() -> Path:
  from openpilot.starpilot.storage import starpilot_storage_root
  return starpilot_storage_root() / 'bluedriver'


def enabled(root: Path | None = None) -> bool:
  try:
    raw = ((default_root() if root is None else root) / 'config.json').read_bytes()
    doc = json.loads(raw) if len(raw) <= MAX_CONFIG_BYTES else None
  except (OSError, ValueError):
    return False
  return isinstance(doc, dict) and doc.get('enabled') is True


def write_json_atomic(path: Path, doc: dict) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f'.{path.name}.')
  try:
    with os.fdopen(fd, 'w') as f:
      json.dump(doc, f, separators=(',', ':'))
    os.replace(tmp, path)
  finally:
    if os.path.exists(tmp):
      os.unlink(tmp)


def read_live(root: Path | None = None, now: float | None = None, max_age: float = 3.0) -> dict | None:
  """The logger's latest live document, or None when it is missing, malformed or stale (same boot clock)."""
  try:
    raw = ((default_root() if root is None else root) / 'live.json').read_bytes()
    doc = json.loads(raw) if len(raw) <= MAX_CONFIG_BYTES else None
  except (OSError, ValueError):
    return None
  if not isinstance(doc, dict) or doc.get('version') != LIVE_VERSION or not isinstance(doc.get('t'), (int, float)):
    return None
  now = time.monotonic() if now is None else now
  return doc if 0.0 <= now - doc['t'] <= max_age else None


class Session:
  """Readings of one drive: the latest value of each PID, the integrated fuel, and the outputs."""

  def __init__(self, root: Path, drive_id: str, clock=time.monotonic):
    self.root, self.drive_id, self.clock = root, drive_id, clock
    self.values: dict[str, float] = {}
    self.fuel = bd.FuelIntegrator()
    self.fuel_direct = bd.FuelIntegrator()  # the ECU's own fuel rate (PID 9D), kept alongside for comparison
    self.fuel_gps: float | None = None
    self.connected = False
    self._csv = None
    self._writer = None
    self._last_write = None

  def update(self, code: str, value: float | None, t: float) -> None:
    name = bd.PIDS[code].name
    if value is None:
      self.values.pop(name, None)
    else:
      self.values[name] = value
    if code == '0144':  # the second fuel reading of the cycle: integrate with the newest air flow and rpm
      self.fuel_gps = bd.fuel_rate_gps(self.values.get('maf'), self.values.get('lambda'), self.values.get('rpm'))
      self.fuel.add(t, self.fuel_gps)
    elif code == '019D':
      self.fuel_direct.add(t, value)  # None (no answer) breaks the integration rather than bridging it

  def disconnected(self) -> None:
    self.connected = False
    self.fuel.add(self.clock(), None)  # never integrate across a dropped link
    self.fuel_direct.add(self.clock(), None)
    self.fuel_gps = None

  def live_document(self, now: float) -> dict:
    return {'version': LIVE_VERSION, 'drive': self.drive_id, 't': now, 'connected': self.connected,
            'fuel_g': round(self.fuel.grams, 3), 'fuel_s': round(self.fuel.seconds, 2),
            'fuel_gps': None if self.fuel_gps is None else round(self.fuel_gps, 4),
            'fuel_direct_g': round(self.fuel_direct.grams, 3),
            **{k: round(v, 4) for k, v in self.values.items()}}

  def maybe_write(self, now: float, period: float = 1.0) -> bool:
    if self._last_write is not None and now - self._last_write < period:
      return False
    self._last_write = now
    write_json_atomic(self.root / 'live.json', self.live_document(now))
    if self.connected:
      self._row(now)
    return True

  def _row(self, now: float) -> None:
    if self._writer is None:
      drives = self.root / 'drives'
      drives.mkdir(parents=True, exist_ok=True)
      self._csv = open(drives / f'{self.drive_id}.csv', 'a', newline='')
      self._writer = csv.DictWriter(self._csv, CSV_FIELDS, extrasaction='ignore')
      if self._csv.tell() == 0:
        self._writer.writeheader()
      prune(drives)
    row = {'t': round(now, 2), 'fuel_gps': self.fuel_gps, 'fuel_g': round(self.fuel.grams, 3),
           'fuel_direct_g': round(self.fuel_direct.grams, 3), **self.values}
    self._writer.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in row.items()})
    self._csv.flush()

  def close(self) -> None:
    if self._csv is not None:
      self._csv.close()
      self._csv = self._writer = None


def prune(drives: Path, keep: int = MAX_DRIVE_LOGS) -> None:
  logs = sorted(drives.glob('*.csv'), key=lambda p: p.stat().st_mtime)
  for old in logs[:-keep] if keep > 0 else logs:
    old.unlink(missing_ok=True)


def is_bonded_scanner(device: dict | None) -> bool:
  """A BlueZ Device1 property set (name -> Variant) that is a paired BlueDriver."""
  if not device:
    return False
  paired, name = device.get('Paired'), device.get('Name') or device.get('Alias')
  return bool(paired is not None and paired.value and name is not None and str(name.value).startswith(bd.NAME_PREFIXES))


class Link:
  """The scanner over BlueZ (D-Bus), using the dbus_next copy vendored for the phone bridge."""

  def __init__(self):
    if str(VENDOR) not in sys.path:
      sys.path.insert(0, str(VENDOR))
    self.bus = self.device = self.cmd = self.data = None
    self.reassembler = bd.Reassembler()
    self.reply: asyncio.Future | None = None
    self._handlers = []

  async def _proxy(self, path: str):
    return self.bus.get_proxy_object('org.bluez', path, await self.bus.introspect('org.bluez', path))

  async def connect(self) -> str:
    from dbus_next import BusType, Variant
    from dbus_next.aio import MessageBus
    self.Variant = Variant
    self.bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
    om = (await self._proxy('/')).get_interface('org.freedesktop.DBus.ObjectManager')
    objects = await om.call_get_managed_objects()
    path = next((p for p, ifs in objects.items() if is_bonded_scanner(ifs.get('org.bluez.Device1'))), None)
    if path is None:
      raise ConnectionError('no bonded BlueDriver (pair it in the minute after plugging it in)')
    self.device = (await self._proxy(path)).get_interface('org.bluez.Device1')
    await asyncio.wait_for(self.device.call_connect(), 20)
    for _ in range(100):
      if await self.device.get_services_resolved():
        break
      await asyncio.sleep(0.1)
    objects = await om.call_get_managed_objects()
    chars = {ifs['org.bluez.GattCharacteristic1']['UUID'].value: p for p, ifs in objects.items()
             if p.startswith(path) and 'org.bluez.GattCharacteristic1' in ifs}
    if bd.CMD_UUID not in chars or bd.DATA_UUID not in chars:
      raise ConnectionError('BlueDriver characteristics not found')
    cmd_obj, data_obj = await self._proxy(chars[bd.CMD_UUID]), await self._proxy(chars[bd.DATA_UUID])
    self.cmd = cmd_obj.get_interface('org.bluez.GattCharacteristic1')
    self.data = data_obj.get_interface('org.bluez.GattCharacteristic1')
    data_obj.get_interface('org.freedesktop.DBus.Properties').on_properties_changed(self._on_data)
    await asyncio.wait_for(self.cmd.call_start_notify(), 8)
    await asyncio.wait_for(self.data.call_start_notify(), 8)
    await asyncio.sleep(0.3)
    await self._write(self.data, bd.OPEN_WINDOW)
    await asyncio.sleep(0.1)
    ident = ''
    for command in bd.INIT_COMMANDS:
      reply = await self.query(command, INIT_TIMEOUT_S)
      if command == 'LMI' and reply:
        ident = reply.splitlines()[0]
    if not ident:
      raise ConnectionError('BlueDriver did not answer its init')
    return ident

  async def _write(self, char, payload: bytes) -> None:
    await char.call_write_value(payload, {'type': self.Variant('s', 'command')})

  def _on_data(self, _iface, changed, _invalidated) -> None:
    if 'Value' not in changed:
      return
    fragment = bytes(changed['Value'].value)
    if not fragment:
      return
    task = asyncio.ensure_future(self._write(self.data, bd.ack(len(fragment))))
    self._handlers.append(task)
    task.add_done_callback(self._handlers.remove)
    text = self.reassembler.feed(fragment)
    if text is not None and self.reply is not None and not self.reply.done():
      self.reply.set_result(text)

  async def query(self, command: str, wait_s: float = QUERY_TIMEOUT_S) -> str | None:
    self.reassembler.clear()
    self.reply = asyncio.get_running_loop().create_future()
    await self._write(self.cmd, bd.encode_command(command))
    try:
      return await asyncio.wait_for(self.reply, wait_s)
    except TimeoutError:
      await asyncio.sleep(0.3)  # let a late reply drain so it cannot answer the next query
      return None

  async def close(self) -> None:
    try:
      if self.device is not None:
        await asyncio.wait_for(self.device.call_disconnect(), 5)
    except Exception:
      pass
    if self.bus is not None:
      self.bus.disconnect()


async def health_check(link, root: Path, drive_id: str) -> dict:
  """Read-only trouble-code check, once per drive: monitor status, stored (03) and pending (07) codes."""
  status = bd.monitor_status(await link.query('0101', HEALTH_TIMEOUT_S))
  stored_raw = await link.query('03', HEALTH_TIMEOUT_S)
  pending_raw = await link.query('07', HEALTH_TIMEOUT_S)
  doc = {'version': 1, 'drive': drive_id, 'mil': None if status is None else status[0],
         'dtc_count': None if status is None else status[1], 'stored': bd.trouble_codes(stored_raw),
         'pending': bd.trouble_codes(pending_raw), 'stored_raw': stored_raw, 'pending_raw': pending_raw}
  write_json_atomic(root / 'health.json', doc)
  return doc


async def poll(link, session: Session, clock=time.monotonic, should_run=lambda: True) -> None:
  """Query the plan in a loop until the link fails (raises) or should_run turns false."""
  cycle, misses = 0, 0
  while should_run():
    for code in bd.plan(cycle):
      reply = await link.query(code)
      value = bd.decode(code, reply)
      session.update(code, value, clock())
      misses = 0 if reply is not None else misses + 1
      if misses >= 10:
        raise ConnectionError('BlueDriver stopped answering')
    session.maybe_write(clock())
    cycle += 1


async def run(root: Path, drive_id: str, log) -> None:
  session = Session(root, drive_id)
  attempt = 0
  checked = False
  try:
    while True:
      if not enabled(root):
        session.maybe_write(time.monotonic())
        await asyncio.sleep(5)
        continue
      link = Link()
      try:
        ident = await link.connect()
        log(f'connected: {ident}')
        if not checked:
          health = await health_check(link, root, drive_id)
          checked = True
          log(f"health: light {health['mil']}, codes {health['stored']} pending {health['pending']}")
        session.connected, attempt = True, 0
        await poll(link, session, should_run=lambda: enabled(root))
      except Exception as e:
        log(f'link lost: {type(e).__name__}: {e}')
      finally:
        session.disconnected()
        session.maybe_write(time.monotonic(), period=0.0)
        await link.close()
      await asyncio.sleep(BACKOFF_S[min(attempt, len(BACKOFF_S) - 1)])
      attempt += 1
  finally:
    session.close()


def main() -> None:
  import openpilot.cereal.messaging as messaging
  from openpilot.common.swaglog import cloudlog
  from openpilot.starpilot.efficiency.efficiencyd import drive_id

  os.nice(10)

  def stop(*_):
    raise SystemExit(0)
  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  # The car interface publishes carState only after fingerprinting and its startup diagnostics.
  sm = messaging.SubMaster(['carState'])
  while not sm.seen['carState']:
    sm.update(1000)
  time.sleep(STARTUP_SETTLE_S)
  root = default_root()
  asyncio.run(run(root, drive_id(), lambda msg: cloudlog.info(f'bluedriverd: {msg}')))


if __name__ == '__main__':
  main()
