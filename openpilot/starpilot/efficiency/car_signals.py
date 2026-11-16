"""Context signals for the efficiency ledger from the Elantra Hybrid's bus 0, matched against bookmarked tests:

  drive mode   0x2FC byte 7 announces each mode change for a few frames (0x40 Eco, 0x60 Sport, 0xA0 Smart; 0 otherwise),
               and MDPS11 (0x381) CF_Mdps_CurrMode (byte 7 bits 3-4) is 1 for as long as Sport is on. Confirmed in the
               bookmarked mode-switch drive and live on 2026-11-13. The car starts every drive in Eco and the button
               cycles Eco > Sport > Smart > Eco, so Sport ending without a seen announcement means Smart.
               (0x44E byte 0, used before, is not the drive mode: it wanders through 0x01-0x87 in every mode.)
  tires        TPMS11 (0x593) bytes 2-5 are front-left, front-right, rear-left, rear-right pressure in psi; 255 while
               the sensors are asleep (parked). Seen at 32-35 psi warming up on a drive.
  map          Navi_HU (0x544) byte 0 is the head unit map's speed limit (mph here; 0 or 255 unknown) and the low
               nibble of byte 2 looks like a road-type code (1 on highways and major roads, 5 on local streets,
               2 on slow streets); the code is recorded, not yet trusted for decisions.
  climate      0x5C4 bytes 0-3 are the four zone setpoints as degrees F - 56 (16 = 72 F), all 0 with the climate off;
               1 Hz. Confirmed live 2026-11-16 at 72, 75, 69, 72 F and off. 0x4BC byte 3 looks like the cabin
               temperature in F (75 -> 71 in the minutes after the heat went off); recorded, not yet trusted. The fan
               level and the A/C request were not found on bus 0 (an auto fan step changed nothing here).

Observe only. Each value persists until a newer frame replaces it; the conflated CAN batch can skip slow frames.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

MDPS11_ADDR = 0x381
MODE_ADDR = 0x2FC
ANNOUNCED = {0x40: 'eco', 0x60: 'sport', 0xA0: 'smart'}
TPMS_ADDR = 0x593
NAVI_ADDR = 0x544
MIN_PSI, MAX_PSI = 10, 80
CLIMATE_ADDR = 0x5C4
CABIN_ADDR = 0x4BC
SETPOINT_F_OFFSET = 56
CABIN_F_RANGE = (-30, 130)
# setpoint minus outside temperature, C: past HEAT_C the climate is heating hard, below COOL_C it is cooling
HEAT_C, COOL_C = 8.0, -3.0


@dataclass
class CarSignals:
  sport: bool | None = None       # the steering's Sport flag (persistent)
  announced: str | None = None    # last mode the car announced (or inferred from the Sport flag ending)
  tires_psi: tuple[int, int, int, int] | None = None
  map_limit: int | None = None  # posted limit from the car's map, in the cluster's units
  road_code: int | None = None
  climate_set_f: int | None = None  # driver's setpoint, F; 0 with the climate off, None until heard
  cabin_f: int | None = None

  @property
  def drive_mode(self) -> str | None:
    if self.sport:
      return 'sport'
    if self.announced in ('smart', 'sport'):
      return self.announced
    if self.sport is None and self.announced is None:
      return None  # nothing heard from the car yet
    return 'eco'  # every drive starts in Eco

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
      if address == MODE_ADDR and len(dat) == 8 and dat[7] in ANNOUNCED:
        self.announced = ANNOUNCED[dat[7]]
      elif address == MDPS11_ADDR and len(dat) == 8:
        sport = (dat[7] >> 3) & 0x3 == 1
        if sport:
          self.announced = 'sport'
        elif self.sport and self.announced == 'sport':
          self.announced = 'smart'  # Sport ended with no announcement seen: the next mode in the cycle
        self.sport = sport
      elif address == TPMS_ADDR and len(dat) >= 6:
        psi = tuple(dat[2:6])
        self.tires_psi = psi if all(MIN_PSI <= p <= MAX_PSI for p in psi) else None
      elif address == NAVI_ADDR and len(dat) >= 3:
        self.map_limit = dat[0] if 0 < dat[0] < 255 else None
        self.road_code = dat[2] & 0x0F if dat[2] != 0xFF else None
      elif address == CLIMATE_ADDR and len(dat) >= 1:
        self.climate_set_f = dat[0] + SETPOINT_F_OFFSET if dat[0] else 0
      elif address == CABIN_ADDR and len(dat) >= 4:
        self.cabin_f = dat[3] if CABIN_F_RANGE[0] <= dat[3] <= CABIN_F_RANGE[1] else None


def climate_band(set_f: int | None, ambient_c: float | None) -> str | None:
  """'off', 'heat', 'cool' or 'mild' from the setpoint and the outside temperature; 'on' without an outside reading;
  None while the climate state is unknown."""
  if set_f is None:
    return None
  if set_f == 0:
    return 'off'
  if ambient_c is None or not math.isfinite(ambient_c):
    return 'on'
  delta = (set_f - 32) * 5 / 9 - ambient_c
  return 'heat' if delta >= HEAT_C else 'cool' if delta <= COOL_C else 'mild'
