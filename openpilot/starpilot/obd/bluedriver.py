"""BlueDriver Pro (Lemur Monitors, BGX Bluetooth LE module + ELM327-style OBD core): wire protocol and OBD decoding.

Pure functions only; the Bluetooth transport lives in bluedriverd.py. Written from the observed protocol:
  - Every byte on the wire is XORed with 0x26, except carriage return (0x0D) and the '>' prompt (0x3E).
  - Text commands go to the CMD characteristic (write without response), terminated by CR.
  - Replies arrive as DATA notifications. Before the first command the host opens a 255-byte receive window
    by writing 00 FF 7F to DATA, and every DATA fragment must be acknowledged with 01 <len> 00, or the
    scanner stops after the first fragment. A reply is complete at the '>' prompt.
  - The scanner pairs only in a short window after it powers up; a bond made then (Just Works) is kept,
    so later connections are encrypted with the stored keys and need no new pairing.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

CMD_UUID = 'a9da6040-0823-4995-94ec-9ce41ca28833'
DATA_UUID = 'a73e9a10-628f-4494-a099-12efaf72258f'
STATUS_UUID = '75a9f022-af03-4e41-b4bc-9de90a47d50b'
NAME_PREFIXES = ('BlueDriv', 'LSB2-', 'LSB3-')
OPEN_WINDOW = bytes([0x00, 0xFF, 0x7F])
# The app's init: identify, defaults, protocol auto-detect, then fast timing (LMX0, about 45 ms a query).
# Standard AT commands after this upset the firmware's state, so none are sent.
INIT_COMMANDS = ('LMI', 'LMD', 'LML0', 'LMI', 'LMD', 'LMDP', 'LMI', 'LMX1', 'LMX0')
PROMPT = 0x3E

STOICH_AFR = 14.1  # US pump gasoline (E10); the ECU's lambda is relative to its own stoichiometry
GRAMS_PER_GALLON = 2840.0  # E10 at about 0.75 kg/L
MAX_FUEL_GPS = 15.0  # g/s; far above this engine's full load, so anything larger is a bad reading
MAX_GAP_S = 2.0  # a longer gap between fuel readings is not integrated across


def xor(data: bytes) -> bytes:
  """Encode or decode (the cipher is symmetric)."""
  return bytes(b if b in (0x0D, PROMPT) else b ^ 0x26 for b in data)


def encode_command(text: str) -> bytes:
  if not text or any(not (0x20 <= ord(ch) < 0x7F) for ch in text):
    raise ValueError('commands are printable ASCII')
  return xor(text.encode() + b'\r')


def ack(fragment_len: int) -> bytes:
  return bytes([0x01, fragment_len & 0xFF, 0x00])


class Reassembler:
  """Collects decoded DATA fragments until the '>' prompt; returns the reply text without the prompt."""
  MAX_BYTES = 4096

  def __init__(self):
    self.buf = bytearray()

  def clear(self) -> None:
    self.buf.clear()

  def feed(self, fragment: bytes) -> str | None:
    self.buf.extend(xor(fragment))
    if len(self.buf) > self.MAX_BYTES:
      self.buf.clear()
      return None
    end = self.buf.find(bytes([PROMPT]))
    if end < 0:
      return None
    text = self.buf[:end].decode('ascii', errors='replace')
    del self.buf[:end + 1]
    return text.replace('\r', '\n').strip()


ERRORS = ('NO DATA', '?', 'STOPPED', 'CAN ERROR', 'BUS', 'UNABLE', 'ERROR', 'SEARCHING')


def reply_bytes(text: str | None) -> bytes | None:
  """Data bytes of a single-frame mode 01 reply ("15 C4"), or None for errors and malformed replies."""
  if text is None:
    return None
  line = text.strip().splitlines()[-1].strip() if text.strip() else ''
  if not line or any(err in line.upper() for err in ERRORS):
    return None
  try:
    return bytes(int(tok, 16) for tok in line.split())
  except ValueError:
    return None


@dataclass(frozen=True)
class Pid:
  code: str
  name: str
  size: int

  def decode(self, b: bytes) -> float:
    return DECODERS[self.code](b)


def _u16(b: bytes) -> int:
  return b[0] << 8 | b[1]


DECODERS = {
  '0104': lambda b: b[0] * 100 / 255,  # engine load, %
  '0105': lambda b: b[0] - 40.0,  # coolant, C
  '0106': lambda b: (b[0] - 128) * 100 / 128,  # short-term fuel trim, %
  '0107': lambda b: (b[0] - 128) * 100 / 128,  # long-term fuel trim, %
  '010C': lambda b: _u16(b) / 4,  # engine speed, rpm
  '010D': lambda b: float(b[0]),  # vehicle speed, km/h
  '010F': lambda b: b[0] - 40.0,  # intake air, C
  '0110': lambda b: _u16(b) / 100,  # mass air flow, g/s
  '012F': lambda b: b[0] * 100 / 255,  # fuel tank level, %
  '0142': lambda b: _u16(b) / 1000,  # control module voltage, V
  '0144': lambda b: _u16(b) * 2 / 65536,  # commanded lambda
  '0146': lambda b: b[0] - 40.0,  # ambient air, C
  '015B': lambda b: b[0] * 100 / 255,  # hybrid battery remaining charge, %
  '0162': lambda b: b[0] - 125.0,  # actual engine torque, % of reference
  '0163': lambda b: float(_u16(b)),  # engine reference torque, Nm
  '01A6': lambda b: (b[0] << 24 | b[1] << 16 | b[2] << 8 | b[3]) / 10,  # odometer, km
  # 11-14 probe on this car: '00 10 00 10' (0.32 g/s at idle, matching the MAF/lambda estimate of 0.323)
  '019D': lambda b: _u16(b) * 0.02,  # engine fuel rate, g/s (the ECU's own figure; bytes 3-4 repeat it as vehicle fuel rate)
}
HV_VOLTS_RANGE = (50.0, 500.0)
DIRECT_FUEL_MAX_GPS = 15.0
PIDS = {p.code: p for p in (
  Pid('0104', 'load', 1), Pid('0105', 'coolant', 1), Pid('0106', 'stft', 1), Pid('0107', 'ltft', 1),
  Pid('010C', 'rpm', 2), Pid('010D', 'speed_kph', 1), Pid('0110', 'maf', 2), Pid('0142', 'volts', 2),
  Pid('010F', 'intake', 1), Pid('012F', 'fuel_level', 1),
  Pid('0144', 'lambda', 2), Pid('0146', 'ambient', 1), Pid('015B', 'soc', 1), Pid('0162', 'torque_pct', 1),
  Pid('0163', 'torque_ref', 2), Pid('01A6', 'odometer_km', 4), Pid('019D', 'fuel_rate_direct', 4),
  Pid('019A', 'hv_volts', 4),
)}


def hv_volts(b: bytes) -> float | None:
  """PID 9A (hybrid/EV battery): A support bits, B charging state, C-D system voltage at 1/64 V. This car's single-frame
  reply ends after C-D ('9A 07 00 3B AB' = 238.7 V); the current in E-F would need a multi-frame read."""
  if len(b) < 4:
    return None
  volts = (b[2] << 8 | b[3]) / 64.0
  return volts if HV_VOLTS_RANGE[0] <= volts <= HV_VOLTS_RANGE[1] else None


def decode(code: str, text: str | None) -> float | None:
  """Decoded value for a mode 01 PID reply, or None when the reply is missing, an error or the wrong size."""
  pid = PIDS[code]
  b = reply_bytes(text)
  if b is None:
    return None
  # Some replies keep the PID number in front of the data ('9A 07 00 3B AB'): drop it when that is all that differs.
  if len(b) == pid.size + 1 and b[0] == int(code[2:], 16):
    b = b[1:]
  if code == '019A':
    return hv_volts(b) if len(b) == pid.size else None
  if len(b) != pid.size:
    return None
  value = pid.decode(b)
  if not math.isfinite(value):
    return None
  if code == '019D' and not 0.0 <= value <= DIRECT_FUEL_MAX_GPS:
    return None
  return value


# Poll plan: the fuel PIDs every cycle, the rest round-robin every N cycles (about 40 ms a query).
FAST = ('0110', '0144', '010C')
SLOW = (('0162', 1), ('015B', 3), ('0106', 5), ('0107', 10), ('0104', 5), ('0105', 25), ('0146', 50),
        ('0142', 25), ('012F', 50), ('01A6', 100), ('0163', 500),
        # the ECU's own fuel rate every cycle (compared against the MAF/lambda estimate), battery voltage every 5th
        ('019D', 1), ('019A', 5))


def plan(cycle: int) -> list[str]:
  return list(FAST) + [code for code, every in SLOW if cycle % every == 0]


def monitor_status(text: str | None) -> tuple[bool, int] | None:
  """PID 0101 from every answering ECU: (check-engine light on in any, stored trouble codes summed)."""
  if text is None:
    return None
  mil, count, seen = False, 0, False
  for line in text.splitlines():
    b = reply_bytes(line)
    if b is not None and len(b) == 4:
      seen = True
      mil |= bool(b[0] & 0x80)
      count += b[0] & 0x7F
  return (mil, count) if seen else None


DTC_PREFIX = 'PCBU'


def trouble_codes(text: str | None) -> list[str]:
  """Mode 03 / 07 reply lines to codes like P0420. Tolerates an echoed 43/47 byte and a leading count byte."""
  codes = []
  for line in (text or '').splitlines():
    b = reply_bytes(line)
    if not b:
      continue
    if b[0] in (0x43, 0x47):
      b = b[1:]
    if len(b) % 2 == 1:
      b = b[1:]  # CAN replies lead with the number of codes
    for hi, lo in zip(b[::2], b[1::2], strict=True):
      if hi or lo:
        codes.append(f'{DTC_PREFIX[hi >> 6]}{(hi >> 4) & 3}{hi & 0xF:X}{lo:02X}')
  return codes


def fuel_rate_gps(maf: float | None, lam: float | None, rpm: float | None) -> float | None:
  """Fuel mass flow from air flow and commanded lambda (closed loop holds the actual mixture at the command).
  Engine stopped (no rpm, or no air) burns nothing; a fuel cut shows as a very lean lambda and little air."""
  if rpm is not None and rpm < 100:
    return 0.0
  if maf is None or lam is None or not math.isfinite(maf) or not math.isfinite(lam) or maf < 0 or lam <= 0.3:
    return None
  if maf < 0.05:
    return 0.0
  rate = maf / (STOICH_AFR * lam)
  return rate if rate <= MAX_FUEL_GPS else None


@dataclass
class FuelIntegrator:
  """Grams of fuel burned, integrated from successive rate readings (trapezoid, gaps never bridged)."""
  grams: float = 0.0
  seconds: float = 0.0  # time actually covered by readings
  _t: float | None = None
  _rate: float | None = None

  def add(self, t: float, rate: float | None) -> None:
    if rate is None:
      self._t = self._rate = None
      return
    if self._t is not None and self._rate is not None and 0.0 < t - self._t <= MAX_GAP_S:
      dt = t - self._t
      self.grams += 0.5 * (rate + self._rate) * dt
      self.seconds += dt
    self._t, self._rate = t, rate

  @property
  def gallons(self) -> float:
    return self.grams / GRAMS_PER_GALLON
