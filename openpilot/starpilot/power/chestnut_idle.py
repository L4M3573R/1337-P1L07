"""Switch the Chestnut's PCIe power off while parked and nothing is using it.

Measured 2026-11-22: after car-off the Chestnut kept drawing 12.5 W (27-42 W if powered with no driver), which with
comma power drained the Elantra's lithium 12 V in a day. The bridge firmware's PCIe power switch (vendor request 0xF3,
the one tinygrad's CustomASM24Controller uses) takes it to 0.0-0.4 W, and any later open (modeld at car-on, a test
script) powers it back on by itself: tinygrad switches PCIe on whenever the link is down.

Rule: never while driving. After the car turns off, off CAR_OFF_GRACE_S after modeld lets go of the device; otherwise
off after IDLE_S with no program holding it. "In use" = another process has the Chestnut's /dev/bus/usb node open
(tinygrad keeps it open for as long as it uses the card). `<storage>/chestnut_idle.json` {"mode": "off"} disables it.
"""

import json
import os
from dataclasses import dataclass, field

IDLE_S = 300.0
CAR_OFF_GRACE_S = 30.0
PCIE_POWER_REQUEST = 0xF3
LINK_UP = 0x78
SETTINGS_PATH = '/data/starpilot/chestnut_idle.json'


@dataclass
class ChestnutIdle:
  idle_since: float | None = None
  car_off_pending: bool = False
  was_started: bool = False
  switched_off: int = field(default=0)

  def update(self, now: float, started: bool, in_use: bool, link_up: bool) -> bool:
    """True when the caller should switch the PCIe power off now."""
    if started:
      self.idle_since, self.car_off_pending, self.was_started = None, False, True
      return False
    if self.was_started:
      self.was_started, self.car_off_pending = False, True
    if not link_up:   # already off (or never powered on since boot)
      self.idle_since, self.car_off_pending = None, False
      return False
    if in_use:
      self.idle_since = None
      return False
    if self.idle_since is None or now < self.idle_since:
      self.idle_since = now
    if now - self.idle_since >= (CAR_OFF_GRACE_S if self.car_off_pending else IDLE_S):
      self.idle_since, self.car_off_pending = None, False
      self.switched_off += 1
      return True
    return False


def enabled(path: str = SETTINGS_PATH) -> bool:
  try:
    with open(path) as f:
      return json.load(f).get('mode') != 'off'
  except (OSError, ValueError, AttributeError):
    return True


def node_path(device: dict) -> str | None:
  bus, dev = device.get('busnum'), device.get('devnum')
  return f'/dev/bus/usb/{bus:03d}/{dev:03d}' if isinstance(bus, int) and isinstance(dev, int) and bus > 0 and dev > 0 else None


def other_users(node: str, own_pid: int | None = None, proc: str = '/proc') -> bool:
  """True if a process other than own_pid has `node` open. Unreadable processes count as not holding it."""
  own_pid = os.getpid() if own_pid is None else own_pid
  try:
    pids = [e.name for e in os.scandir(proc) if e.name.isdigit() and int(e.name) != own_pid]
  except OSError:
    return True   # cannot tell: keep it on
  for pid in pids:
    try:
      fds = os.scandir(f'{proc}/{pid}/fd')
    except OSError:
      continue
    with fds:
      for fd in fds:
        try:
          if os.readlink(fd.path) == node:
            return True
        except OSError:
          continue
  return False
