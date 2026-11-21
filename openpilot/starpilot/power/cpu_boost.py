"""1337: CPU clock boost (big cores to a target, little cores to their top step) and a matching fan curve.

Stock openpilot caps both CPU clusters at 1689.6 MHz onroad (comma hardware.set_power_save). The big cores (4-7: plannerd,
radard, controlsd, modeld) can run to about 2.8 GHz; on 11-19 drives the device was CPU-bound, never thermal-limited
(66-72 C, thermal status always ok, fan ~40 %). This raises the big cluster's cap to the highest step the kernel offers
at or below the target, lifts the little cluster (0-3, the busiest cores at 85-91 %) to its top step (~5 % over the
stock cap), and runs the fan a little harder to pay for the heat.

cpu_boost.json in the StarPilot storage root, re-read every few seconds so it can be flipped during a drive:
  {"mode": "on" | "off", "big_mhz": 2200, "little": "max" | "stock"}
Missing or unreadable file = on at DEFAULT_BIG_MHZ. A heat guard drops back to the stock cap at GUARD_HOT_C and returns
only below GUARD_COOL_C, far under the mici's overheated band (100 C).
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path

STOCK_KHZ = 1689600
DEFAULT_BIG_MHZ = 2200
MAX_BIG_MHZ = 2900
GUARD_HOT_C = 85.0
GUARD_COOL_C = 78.0
READ_PERIOD_S = 5.0
LITTLE_POLICY = '/sys/devices/system/cpu/cpufreq/policy0'
BIG_POLICY = '/sys/devices/system/cpu/cpufreq/policy4'
CONFIG_NAME = 'cpu_boost.json'

# mici fan while boosted: the stock controller's output plus 5 points (user's pick 2026-11-20). A shifted curve does not
# work: the fan PID floors its integrator at the 30 % onroad minimum whenever the feedforward alone is above 30 %, so
# moving the feedforward ramp 2 C lower jumped the fan from 30 % to ~60 % at 70-72 C (seen on the comma 2026-11-20).
# (A Chestnut USB IRQ core switch was tried here 2026-11-20 and removed: core 6 vs stock 0-1 gained ~0.3 ms of 33,
# parked and driving; the USB wait did not move.)
BOOST_FAN_EXTRA = 5


@dataclass
class Config:
  on: bool = True
  big_mhz: int = DEFAULT_BIG_MHZ
  little_max: bool = True


def read_config(path: Path) -> Config:
  """Anything unreadable falls back to the defaults (on, big 2200 MHz, little at max)."""
  try:
    doc = json.loads(path.read_bytes()[:4096])
    mhz = doc.get('big_mhz', DEFAULT_BIG_MHZ)
    if type(mhz) not in (int, float) or not math.isfinite(mhz):
      mhz = DEFAULT_BIG_MHZ
    return Config(doc.get('mode', 'on') != 'off', int(min(max(mhz, STOCK_KHZ / 1000), MAX_BIG_MHZ)),
                  doc.get('little', 'max') != 'stock')
  except (OSError, ValueError, AttributeError):
    return Config()


def pick_step(available_khz: list[int], target_mhz: float) -> int:
  """Highest available step at or below the target, never below stock; stock when the table is unknown."""
  steps = [k for k in available_khz if STOCK_KHZ <= k <= target_mhz * 1000]
  return max(steps) if steps else STOCK_KHZ


def read_cap(policy: str) -> int | None:
  try:
    return int(Path(policy, 'scaling_max_freq').read_text())
  except (OSError, ValueError):
    return None


def read_steps(policy: str) -> list[int]:
  try:
    return sorted(int(x) for x in Path(policy, 'scaling_available_frequencies').read_text().split())
  except (OSError, ValueError):
    return []


@dataclass(eq=False)  # identity: used as a dict key
class Cluster:
  policy: str
  steps: list[int] = field(default_factory=list)
  applied: int | None = None


class CpuBoost:
  def __init__(self, config_path: Path, write, steps: dict[str, list[int]] | None = None, log=None, read=read_cap):
    self.config_path = config_path
    self.write = write  # write(value, sysfs path)
    self.read = read  # read(policy): the cap the kernel holds now, to notice anything else rewriting it
    self.little = Cluster(LITTLE_POLICY)
    self.big = Cluster(BIG_POLICY)
    for c in (self.little, self.big):
      c.steps = read_steps(c.policy) if steps is None else steps.get(c.policy, [])
    self.log = log
    self.config = Config()
    self.hot = False
    self.next_read = 0.0

  @property
  def on(self) -> bool:
    return self.config.on

  def reset(self) -> None:
    """set_power_save rewrites the stock caps: write ours again on the next update."""
    self.little.applied = self.big.applied = None

  @property
  def boosting(self) -> bool:
    return any(c.applied is not None and c.applied > STOCK_KHZ for c in (self.little, self.big))

  def fan_boost(self, ignition: bool) -> bool:
    """The harder fan curve follows the switch, not the clock: it stays on while the heat guard holds the stock cap."""
    return ignition and self.config.on

  def update(self, ignition: bool, online: bool, temp_c: float, now: float | None = None) -> dict[str, int]:
    """Call every hardwared loop. ignition: boost allowed; online: big cores up (not power save).
    Returns {policy: cap} for what was written this call."""
    now = time.monotonic() if now is None else now
    if now >= self.next_read:
      self.next_read = now + READ_PERIOD_S
      self.config = read_config(self.config_path)
      for c in (self.little, self.big):
        if c.applied is not None and (online or c is self.little) and self.read(c.policy) not in (None, c.applied):
          c.applied = None  # something else wrote the cap: put ours back
    if math.isfinite(temp_c):
      if temp_c >= GUARD_HOT_C:
        self.hot = True
      elif temp_c < GUARD_COOL_C:
        self.hot = False
    boost = ignition and self.config.on and not self.hot
    if not online:  # big cluster is offline in power save; set_power_save owns it
      self.big.applied = None
    wants = {self.little: pick_step(self.little.steps, math.inf) if boost and self.config.little_max else STOCK_KHZ}
    if online:
      wants[self.big] = pick_step(self.big.steps, self.config.big_mhz) if boost else STOCK_KHZ
    written = {}
    for c, want in wants.items():
      if want != c.applied:
        self.write(str(want), f'{c.policy}/scaling_max_freq')
        c.applied = written[c.policy] = want
    if written and self.log is not None:
      self.log('cpu_boost', little_khz=self.little.applied, big_khz=self.big.applied, on=self.config.on, hot=self.hot,
               temp_c=round(temp_c, 1) if math.isfinite(temp_c) else None)
    return written
