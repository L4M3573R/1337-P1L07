"""Context signals for the efficiency ledger from the Elantra Hybrid's bus 0, matched against bookmarked tests:

  drive mode   MDPS11 (0x381) CF_Mdps_CurrMode (byte 7 bits 3-4) is 1 in Sport (the steering follows the mode);
               0x44E byte 0 bit 7 is set in Smart; neither means Eco. Verified on 2026-11-12 (Eco > Sport > Smart > Eco).
  tires        TPMS11 (0x593) bytes 2-5 are front-left, front-right, rear-left, rear-right pressure in psi; 255 while
               the sensors are asleep (parked). Seen at 32-35 psi warming up on a drive.
  map          Navi_HU (0x544) byte 0 is the head unit map's speed limit (mph here; 0 or 255 unknown) and the low
               nibble of byte 2 looks like a road-type code (1 on highways and major roads, 5 on local streets,
               2 on slow streets); the code is recorded, not yet trusted for decisions.

Observe only. Each value persists until a newer frame replaces it; the conflated CAN batch can skip slow frames.
"""
from __future__ import annotations

from dataclasses import dataclass

MDPS11_ADDR = 0x381
SMART_ADDR = 0x44E
TPMS_ADDR = 0x593
NAVI_ADDR = 0x544
MIN_PSI, MAX_PSI = 10, 80


@dataclass
class CarSignals:
  sport: bool | None = None
  smart: bool | None = None
  tires_psi: tuple[int, int, int, int] | None = None
  map_limit: int | None = None  # posted limit from the car's map, in the cluster's units
  road_code: int | None = None

  @property
  def drive_mode(self) -> str | None:
    if self.sport is None or self.smart is None:
      return None
    return 'sport' if self.sport else 'smart' if self.smart else 'eco'

  @property
  def tire_avg(self) -> float | None:
    return sum(self.tires_psi) / 4 if self.tires_psi else None

  @property
  def tire_min(self) -> int | None:
    return min(self.tires_psi) if self.tires_psi else None

  def update(self, frames) -> None:
    """One batch of (address, dat, src) frames."""
    for address, dat, src in frames:
      if src != 0:
        continue
      if address == MDPS11_ADDR and len(dat) == 8:
        self.sport = (dat[7] >> 3) & 0x3 == 1
      elif address == SMART_ADDR and len(dat) >= 1:
        self.smart = bool(dat[0] & 0x80)
      elif address == TPMS_ADDR and len(dat) >= 6:
        psi = tuple(dat[2:6])
        self.tires_psi = psi if all(MIN_PSI <= p <= MAX_PSI for p in psi) else None
      elif address == NAVI_ADDR and len(dat) >= 3:
        self.map_limit = dat[0] if 0 < dat[0] < 255 else None
        self.road_code = dat[2] & 0x0F if dat[2] != 0xFF else None
