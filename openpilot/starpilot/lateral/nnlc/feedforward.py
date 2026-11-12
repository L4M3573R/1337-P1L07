"""Builds the NNLC inputs each control tick and returns a bounded feedforward torque.

Inputs, in model order: speed, lateral accel the car should reach after the steering delay, jerk
term, roll, that lateral accel 0.3/0.2/0.1 s ago, the planned lateral accel 0.3/0.6/1.0/1.5 s past
the delay, roll 0.3/0.2/0.1 s ago, and the predicted roll at those future times. Training uses the
car's actual response at the same offsets, so the model maps a lateral-accel path to the torque it took.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.starpilot.lateral.nnlc.model import NnlcModel

PAST_S = (0.3, 0.2, 0.1)
FUTURE_S = (0.3, 0.6, 1.0, 1.5)
JERK_GAIN = 0.4
# The network may move the feedforward at most this far (normalized torque) from StarPilot's own
# feedforward. Offline its corrections in curves were 0.05-0.2; this stops a bad file from steering hard.
MAX_DEVIATION = 0.3
MAX_TORQUE = 1.0
# Few training moments are this tight, so the network flattens out there (a known NNLC weakness). In
# tight curves it is never allowed to steer less than StarPilot would: blend fully back by 2.2 m/s^2.
TIGHT_CURVE_BLEND = (1.5, 2.2)


class NnlcFeedforward:
  def __init__(self, model: NnlcModel, dt: float, model_sha256: str = ''):
    self.model = model
    self.model_sha256 = model_sha256
    self.dt = dt
    self.history_len = int(round(max(PAST_S) / dt))
    self.offsets = [int(round(s / dt)) for s in PAST_S]
    self.lat_accel_history: deque[float] = deque(maxlen=self.history_len)
    self.roll_history: deque[float] = deque(maxlen=self.history_len)
    self.plan = None
    self.last_torque = math.nan
    self.last_used = False

  def set_plan(self, model_v2) -> None:
    self.plan = model_v2

  def observe(self, lat_accel: float, roll: float) -> None:
    """Called every tick, engaged or not, so the history is always continuous."""
    self.lat_accel_history.append(float(lat_accel))
    self.roll_history.append(float(roll))

  def _past(self, history: deque[float]) -> list[float]:
    return [history[-n] if len(history) >= n else history[0] for n in self.offsets]

  def torque(self, v_ego: float, a_ego: float, lat_accel: float, lat_jerk: float, roll: float, lat_delay: float,
             baseline_torque: float) -> float | None:
    """The bounded NNLC feedforward torque, or None when it cannot be computed (use the baseline)."""
    self.last_used = False
    plan = self.plan
    if plan is None or not self.lat_accel_history:
      return None
    acc_y, ori_x = plan.acceleration.y, plan.orientation.x
    if len(acc_y) != len(ModelConstants.T_IDXS) or len(ori_x) != len(ModelConstants.T_IDXS):
      return None
    times = [lat_delay + s for s in FUTURE_S]
    times = [t + 0.5 * a_ego * (t / max(v_ego, 1.0)) for t in times]
    future_lat = [float(np.interp(t, ModelConstants.T_IDXS, acc_y)) for t in times]
    future_roll = [float(np.interp(t, ModelConstants.T_IDXS, ori_x)) + roll for t in times]
    x = [v_ego, lat_accel, JERK_GAIN * lat_jerk, roll] + self._past(self.lat_accel_history) + future_lat + \
        self._past(self.roll_history) + future_roll
    if not all(math.isfinite(value) for value in x) or not math.isfinite(baseline_torque):
      return None
    out = self.model.evaluate(x)
    if not math.isfinite(out):
      return None
    blend = float(np.interp(abs(lat_accel), TIGHT_CURVE_BLEND, (0.0, 1.0)))
    toward_curve = math.copysign(1.0, lat_accel)
    if blend > 0.0 and baseline_torque * toward_curve > out * toward_curve:
      out += blend * (baseline_torque - out)
    out = min(max(out, baseline_torque - MAX_DEVIATION), baseline_torque + MAX_DEVIATION)
    out = min(max(out, -MAX_TORQUE), MAX_TORQUE)
    self.last_torque, self.last_used = out, True
    return out
