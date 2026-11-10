from opendbc.car import CanData
from opendbc.car.hyundai.hyundaicanfd import CanBus, hkg_can_fd_checksum
from opendbc.car.hyundai.values import CAR, HyundaiFlags, is_blended


CANFD_DAW_WORDS = frozenset((0x11, 0x91, 0x811, 0x891, 0x8015, 0x8095, 0x8815, 0x8895))

# Classic LKAS12 (0x53E) comes in two layouts. The 6-byte one reports advisories as status 1-3.
# The 8-byte one (e.g. Elantra Hybrid 2024) reports an attention meter: status counts down 5..1
# while the driver makes no steering corrections (openpilot is steering), then 6 plus flag 0x10
# raises "take a break". Hold that meter full (5) and clear the flag.
CLASSIC_LENGTHS = (6, 8)
METER_FULL = 5
METER_MASK = 0x17


def classic_transform(data: bytearray) -> None:
  status = data[5] & 7
  if len(data) == 8:
    if 1 <= status <= 6:
      data[5] = (data[5] & ~METER_MASK & 0xFF) | METER_FULL
  elif 1 <= status <= 3:
    data[5] &= ~7


class DawSource:
  def __init__(self, cp):
    self.canfd = bool(cp.flags & HyundaiFlags.CANFD)
    self.enabled = (cp.brand == 'hyundai' and not cp.passive and not cp.dashcamOnly and not cp.notCar and
                    not is_blended(cp) and (not self.canfd or cp.carFingerprint == CAR.HYUNDAI_IONIQ_6))
    if self.canfd:
      self.enabled = bool(self.enabled and cp.safetyConfigs and
                          int(cp.safetyConfigs[-1].safetyParam) in CANFD_DAW_WORDS and CanBus(cp).ECAN % 4 == 1)
    self.address = 0x11A if self.canfd else 0x53E
    self.source_bus = CanBus(cp).ECAN if self.canfd else 2
    self.target_bus = CanBus(cp).ECAN if self.canfd else 0
    self.lengths = (16,) if self.canfd else CLASSIC_LENGTHS
    self.data = None
    self.stamp = 0

  def update(self, packets):
    if not self.enabled:
      return
    for stamp, frames in packets:
      for address, data, bus in frames:
        if address != self.address or bus != self.source_bus:
          continue
        if stamp <= self.stamp or len(data) not in self.lengths or (self.canfd and
            int.from_bytes(data[:2], 'little') != hkg_can_fd_checksum(address, None, data)):
          self.data = None
          continue
        self.data, self.stamp = bytes(data), stamp

  def command(self, now_ns):
    if (not self.enabled or self.source_bus == self.target_bus or self.data is None or
        not 0 < self.stamp <= now_ns <= self.stamp + 100_000_000):
      return []
    data = bytearray(self.data)
    if self.canfd:
      if (data[7] >> 3) & 7 == 1:
        data[7] &= ~0x38
      data[:2] = hkg_can_fd_checksum(self.address, None, data).to_bytes(2, 'little')
    else:
      classic_transform(data)
    return [CanData(self.address, bytes(data), self.target_bus)]
