"""Elantra Hybrid powertrain signals on the comma's bus, decoded for the efficiency ledger.

0x595 (power flow): byte 1 tracks battery charge, byte 2 is a power meter (about 211 when no energy
flows; lower while the battery drives, higher while it charges, 255 at the limit), byte 5 is the
power-flow mode. Mode meanings were matched against the dash EV light on a recorded drive:
  16 stopped, 32 EV drive, 96 EV with regen (EV light on), 48 / 144 engine driving (battery assisting),
  128 / 160 engine on while slowing or coasting.

0x50C (CLU13, cluster): CF_Clu_AvgFCI is the dash's average fuel economy since the last fill-up
(0.1 units, US mpg here) and CF_Clu_DTE is the range in miles. Both reset when the tank is filled.
"""
from __future__ import annotations

from dataclasses import dataclass

POWER_FLOW_ADDR = 0x595
CLUSTER_ADDR = 0x50C
POWER_METER_NEUTRAL = 211

FLOW_CLASS = {16: 'stopped', 32: 'ev', 96: 'regen', 48: 'engine', 144: 'engine', 128: 'engine', 160: 'engine'}
FLOW_CLASSES = ('stopped', 'ev', 'regen', 'engine', 'other')


@dataclass(frozen=True)
class PowerFlow:
  mode: int
  flow: str  # one of FLOW_CLASSES
  meter: int
  charge: int  # battery charge, raw units


@dataclass(frozen=True)
class Cluster:
  avg_mpg: float  # average since the last fill-up
  range_mi: int


def decode_power_flow(dat: bytes) -> PowerFlow | None:
  if len(dat) < 6:
    return None
  return PowerFlow(dat[5], FLOW_CLASS.get(dat[5], 'other'), dat[2], dat[1])


def decode_cluster(dat: bytes) -> Cluster | None:
  if len(dat) != 8:
    return None
  raw = int.from_bytes(dat, 'little')
  return Cluster(((raw >> 6) & 0x3FF) * 0.1, (raw >> 24) & 0x3FF)


def scan(frames) -> tuple[PowerFlow | None, Cluster | None]:
  """Latest power flow and cluster readings in one batch of bus-0 CAN frames (address, dat, src)."""
  flow = cluster = None
  for address, dat, src in frames:
    if src != 0:
      continue
    if address == POWER_FLOW_ADDR:
      flow = decode_power_flow(dat) or flow
    elif address == CLUSTER_ADDR:
      cluster = decode_cluster(dat) or cluster
  return flow, cluster
