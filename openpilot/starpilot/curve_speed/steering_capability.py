"""How much cornering the comma gets per unit of steering torque, learned per speed band while it steers.

Starts from the table measured on this car (6.3 h of logged steering, 2026-11-13) and moves toward what live
steering shows: every moment the comma steers on its own (no driver torque, not at its limit) with real torque and
real cornering is one sample of lateral acceleration per unit torque. The planner's steering limit is this times its
margin, so the margin applies to a limit that keeps getting more accurate for this car, road and season.
Pure: the daemon feeds samples and saves the document.
"""
from __future__ import annotations

import math

import numpy as np

from openpilot.starpilot.curve_speed.map_target import TORQUE_CAP_A, TORQUE_CAP_V

MIN_TORQUE = 0.15          # below this the ratio is mostly noise
MIN_LATERAL = 0.3          # m/s^2
MIN_SPEED = 4.0            # m/s
PRIOR_COUNT = 200          # the measured table counts as this many samples per band
HISTORY = 5000             # newer driving keeps a say: at most this many samples of history per band
RANGE = (0.5, 2.0)         # a learned band stays within this factor of the measured table


class SteeringCapability:
  def __init__(self):
    self.gain = [float(a) for a in TORQUE_CAP_A]
    self.count = [0] * len(TORQUE_CAP_A)

  def _band(self, speed: float) -> int:
    return int(np.argmin([abs(speed - v) for v in TORQUE_CAP_V]))

  def observe(self, speed: float, lateral: float, torque: float) -> bool:
    """One moment of the comma steering on its own; returns whether it was used."""
    if not all(math.isfinite(x) for x in (speed, lateral, torque)):
      return False
    lateral, torque = abs(lateral), abs(torque)
    if torque < MIN_TORQUE or lateral < MIN_LATERAL or speed < MIN_SPEED or torque > 1.0:
      return False
    i = self._band(speed)
    sample = float(np.clip(lateral / torque, RANGE[0] * TORQUE_CAP_A[i], RANGE[1] * TORQUE_CAP_A[i]))
    weight = 1.0 / (min(self.count[i], HISTORY) + PRIOR_COUNT + 1)
    self.gain[i] += (sample - self.gain[i]) * weight
    self.count[i] += 1
    return True

  def cap(self, speed: float) -> float:
    """Lateral acceleration (m/s^2) at full steering torque, at this speed."""
    return float(np.interp(speed, TORQUE_CAP_V, self.gain))

  def document(self) -> dict:
    return {'version': 1, 'gain': [round(g, 4) for g in self.gain], 'count': list(self.count)}

  @classmethod
  def load(cls, doc) -> SteeringCapability:
    out = cls()
    try:
      if doc and doc.get('version') == 1 and len(doc['gain']) == len(TORQUE_CAP_A) == len(doc['count']):
        gain = [float(g) for g in doc['gain']]
        count = [int(c) for c in doc['count']]
        if all(math.isfinite(g) and RANGE[0] * a <= g <= RANGE[1] * a for g, a in zip(gain, TORQUE_CAP_A, strict=True)) and \
           all(c >= 0 for c in count):
          out.gain, out.count = gain, count
    except (TypeError, ValueError, KeyError, AttributeError):
      pass  # anything unreadable starts again from the measured table
    return out
