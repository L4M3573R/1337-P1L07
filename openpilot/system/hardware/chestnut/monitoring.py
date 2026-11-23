import struct
import time

import usb1

import openpilot.cereal.messaging as messaging
from openpilot.cereal.services import SERVICE_LIST
from openpilot.common.hardware.usb import CHESTNUT_USB_PRODUCT, get_usb_state, is_chestnut_usb_id
from openpilot.common.swaglog import cloudlog
from openpilot.starpilot.power import chestnut_idle

IDLE_CHECK_S = 5.0


def read_chestnut_state(handle, gpu_state=None):
  msg = messaging.new_message('chestnutState')
  if gpu_state is not None:
    msg.chestnutState = gpu_state
  state = msg.chestnutState
  try:
    raw = handle.controlRead(0xC0, 0xC0, 0, 0, 5, timeout=100)
    state.supplyVoltage, state.supplyCurrent, state.supplyFault = struct.unpack('<Hh?', bytes(raw))
    raw = handle.controlRead(0xC0, 0xE4, 0xB450, 0, 1, timeout=100)
    state.pcieLtssm, = struct.unpack('B', bytes(raw))
    msg.valid = True
  except (usb1.USBError, struct.error):
    msg.valid = False
  return msg


def idle_power_off(handle, idle, sm, node, ltssm, now) -> None:
  """Parked and unused: switch the Chestnut's PCIe power off (the next open powers it back on)."""
  try:
    # deviceState unseen = unknown drive state: treat as driving, never switch off
    started = sm['deviceState'].started if sm.seen['deviceState'] else True
    link_up = ltssm == chestnut_idle.LINK_UP
    in_use = (node is None or chestnut_idle.other_users(node)) if link_up and not started else False
    if idle.update(now, started, in_use, link_up) and chestnut_idle.enabled():
      handle.controlWrite(0x40, chestnut_idle.PCIE_POWER_REQUEST, 0, 0, b'', timeout=10000)
      cloudlog.event("chestnut idle power off", count=idle.switched_off)
  except Exception:
    cloudlog.exception("chestnut idle power off failed")


def chestnut_state_thread(end_event):
  pm = messaging.PubMaster(['chestnutState'])
  sm = messaging.SubMaster(['chestnutGpuState', 'deviceState'])
  idle, node, next_idle_check = chestnut_idle.ChestnutIdle(), None, 0.
  with usb1.USBContext() as context:
    handle = None
    try:
      while not end_event.is_set():
        if handle is None:
          devices = [d for d in get_usb_state() if is_chestnut_usb_id(d['vendorId'], d['productId']) and
                     d['product'] == CHESTNUT_USB_PRODUCT]
          if len(devices) == 1:
            try:
              handle = context.openByVendorIDAndProductID(devices[0]['vendorId'], devices[0]['productId'], skip_on_error=True)
              node = chestnut_idle.node_path(devices[0])
            except usb1.USBError:
              pass
        if handle is not None:
          sm.update(0)
          gpu_valid = sm.alive['chestnutGpuState'] and sm.valid['chestnutGpuState']
          msg = read_chestnut_state(handle, sm['chestnutGpuState'] if gpu_valid else None)
          now = time.monotonic()
          if msg.valid and now >= next_idle_check:
            next_idle_check = now + IDLE_CHECK_S
            idle_power_off(handle, idle, sm, node, msg.chestnutState.pcieLtssm, now)
          if not msg.valid:
            handle.close()
            handle = None
          msg.valid &= not sm.seen['chestnutGpuState'] or gpu_valid
          pm.send('chestnutState', msg)
        end_event.wait(1 / SERVICE_LIST['chestnutState'].frequency if handle is not None else 1.)
    finally:
      if handle is not None:
        handle.close()
