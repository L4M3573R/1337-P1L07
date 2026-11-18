"""1337: the fast torque ramp only for a demand that keeps going one way.

The 2024 Elantra Hybrid's EPS takes a torque step of 10 up / 9 down per frame (values.py). With those as flat limits the
low-speed steering loop overshoots back and forth at about 2 Hz (first drive, 2026-11-18: 5 deg of wheel shake at
8-25 mph against 2 deg on 3 / 7; the slow ramp was what kept that hunting small). A turn asks for torque one way for
a long time; hunting reverses every quarter second. So after the applied torque reverses, the ramp is the old 3 / 7
for HOLD_FRAMES, and only then grows to the full step. Anything that swings faster than about 1 Hz therefore gets
the old ramp up and down; a steady build towards a turn gets the fast one.

One thing is not the ramp's to slow: torque above what the driver's hands allow. The panda (driver_limit_check in
safety/lateral.h) wants that to come down by the full release step every frame, and refuses the frame otherwise. The
first build of this file released at 7 there, the panda refused the frames, the EPS faulted and beeped and the fault
latch turned the steering off (second drive, 2026-11-18). apply() gives that case the full step;
safety/tests/test_hyundai.py sends this ramp's torque through the panda's own check.
"""
import json
from types import SimpleNamespace

from opendbc.car.lateral import apply_driver_steer_torque_limits

# {"ramp": "steady" | "fast" | "slow", "hold_frames": n, "grow_frames": n} on the comma switches it at the next car
# start without an install: fast is the flat 10 / 9 of the first drive, slow the flat 3 / 7 of before. No file: steady.
RAMP_PATH = '/data/starpilot/steer_ramp.json'

BASE_UP = 3            # the Hyundai default ramp, used while the torque keeps reversing
BASE_DOWN = 7
REVERSAL = 15          # applied torque coming back this far from its last extreme is a reversal (400 is the maximum)
HOLD_FRAMES = 50       # 100 Hz frames on the base ramp after a reversal (0.5 s: half a cycle of anything above 1 Hz)
GROW_FRAMES = 25       # then this many frames from the base ramp to the full one


class SteadyRamp:
  def __init__(self, limits, base_up: int = BASE_UP, base_down: int = BASE_DOWN, reversal: int = REVERSAL,
               hold_frames: int = HOLD_FRAMES, grow_frames: int = GROW_FRAMES):
    self.full_up, self.full_down = int(limits.STEER_DELTA_UP), int(limits.STEER_DELTA_DOWN)
    self.base_up, self.base_down = min(base_up, self.full_up), min(base_down, self.full_down)
    self.reversal, self.hold_frames, self.grow_frames = reversal, hold_frames, max(1, grow_frames)
    self.direction = 0
    self.extreme = self.high = self.low = 0
    self.steady_frames = hold_frames + grow_frames   # nothing has reversed yet: the full ramp is free
    self.view = SimpleNamespace(**{k: getattr(limits, k) for k in dir(limits) if k.isupper()})

  def steps(self) -> tuple[int, int]:
    share = min(1.0, max(0.0, (self.steady_frames - self.hold_frames) / self.grow_frames))
    return (int(self.base_up + share * (self.full_up - self.base_up)),
            int(self.base_down + share * (self.full_down - self.base_down)))

  def limits(self, limits=None):
    """The limits for this frame: everything as given, the two steps as the ramp allows them now."""
    if limits is not None:
      for k in dir(limits):
        if k.isupper():
          setattr(self.view, k, getattr(limits, k))
    self.view.STEER_DELTA_UP, self.view.STEER_DELTA_DOWN = self.steps()
    return self.view

  def apply(self, wanted: int, last: int, driver_torque: float) -> int:
    """The torque to send this frame."""
    L = self.limits()
    torque = apply_driver_steer_torque_limits(wanted, last, driver_torque, L)
    # torque above what the driver's hands allow comes down by the full release step, whatever the ramp is doing
    hands = driver_torque * L.STEER_DRIVER_FACTOR
    if last > 0:
      allowed = max(min(L.STEER_MAX, L.STEER_MAX + (L.STEER_DRIVER_ALLOWANCE + hands) * L.STEER_DRIVER_MULTIPLIER), 0)
      torque = min(torque, max(last - self.full_down, allowed))
    elif last < 0:
      allowed = min(max(-L.STEER_MAX, -L.STEER_MAX + (-L.STEER_DRIVER_ALLOWANCE + hands) * L.STEER_DRIVER_MULTIPLIER), 0)
      torque = max(torque, min(last + self.full_down, allowed))
    torque = int(round(float(torque)))
    self.update(torque)
    return torque

  def update(self, applied: int) -> None:
    """Tell it the torque that was sent this frame."""
    applied = int(applied)
    if self.direction == 0:
      # nothing has moved yet: the first move either way sets the direction and is not a reversal
      self.high, self.low = max(self.high, applied), min(self.low, applied)
      if applied - self.low >= self.reversal:
        self.direction, self.extreme = 1, applied
      elif self.high - applied >= self.reversal:
        self.direction, self.extreme = -1, applied
    elif self.direction * (applied - self.extreme) >= 0:
      self.extreme = applied
    elif abs(applied - self.extreme) >= self.reversal:
      self.direction, self.extreme = -self.direction, applied
      self.steady_frames = -1
    self.steady_frames = min(self.steady_frames + 1, self.hold_frames + self.grow_frames)


def from_file(limits, path: str = RAMP_PATH):
  """The ramp the file asks for; None is the flat full ramp. Anything unreadable or out of range is the default."""
  try:
    with open(path) as f:
      doc = json.load(f)
    doc = doc if isinstance(doc, dict) else {}
  except (OSError, ValueError):
    doc = {}
  mode = doc.get('ramp', 'steady')
  if mode == 'fast':
    return None
  def frames(key, default):
    v = doc.get(key, default)
    return int(v) if isinstance(v, (int, float)) and not isinstance(v, bool) and 0 <= v <= 300 else default
  ramp = SteadyRamp(limits, hold_frames=frames('hold_frames', HOLD_FRAMES), grow_frames=frames('grow_frames', GROW_FRAMES))
  if mode == 'slow':
    ramp.full_up, ramp.full_down = ramp.base_up, ramp.base_down
  return ramp
