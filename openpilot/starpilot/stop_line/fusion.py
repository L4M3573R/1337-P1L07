"""Where to stop: the stop-line finder's frames fused along the road, with chestnut's planned stop as the fallback.

Each inference says whether a light or sign stop is ahead, how far, and how sure (a Laplace scale). A believed stop
point is kept in odometer metres, so between inferences (a few a second) the distance left is just that point minus
the distance driven. Measurements are fused like a 1-D Kalman filter; a far measurement that disagrees by more than
JUMP_SIGMAS restarts the estimate (a new intersection rather than noise).

choose() gives the stop distance to use: the driving model's planned stop when its plan reaches one (chestnut when it
runs, modelV2.big), otherwise ours when it is fresh and sure, otherwise none. Pure: no IPC, no Params, no logs.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

LABEL_RANGE_M = 120.0
P_ON = 0.6  # stop-ahead probability that starts or feeds an estimate
P_OFF = 0.3  # below this for MISS_LIMIT inferences in a row, the estimate is dropped
MISS_LIMIT = 3
MAX_SIGMA_M = 12.0  # an estimate less sure than this is not used over chestnut
MIN_SIGMA_M = 0.5
JUMP_SIGMAS = 4.0
PASSED_M = 3.0  # driven this far past the point (still moving): it was not a stop after all
MAX_AGE_S = 1.0


@dataclass(frozen=True)
class Inference:
  probability: float
  distance_m: float
  sigma_m: float


def decode(out) -> Inference:
  """Model output row (logit, distance fraction, log Laplace scale of the fraction) to an Inference."""
  logit, fraction, log_b = (float(x) for x in out)
  probability = 1.0 / (1.0 + math.exp(-max(min(logit, 30.0), -30.0)))
  sigma = math.sqrt(2.0) * math.exp(log_b) * LABEL_RANGE_M  # Laplace scale b has std sqrt(2) b
  return Inference(probability, fraction * LABEL_RANGE_M, max(sigma, MIN_SIGMA_M))


class StopPointFuser:
  def __init__(self):
    self.reset()

  def reset(self):
    self.point_m: float | None = None  # odometer metres of the believed stop point
    self.var_m2 = 0.0
    self.misses = 0
    self.updated_s = -1e9
    self.samples = 0

  def update(self, inf: Inference, odometer_m: float, now_s: float) -> None:
    self.updated_s = now_s
    if not (math.isfinite(inf.probability) and math.isfinite(inf.distance_m) and math.isfinite(inf.sigma_m)):
      return
    if inf.probability < P_ON:
      if inf.probability < P_OFF:
        self.misses += 1
        if self.misses >= MISS_LIMIT:
          self.reset()
          self.updated_s = now_s
      return
    self.misses = 0
    z, r = odometer_m + inf.distance_m, inf.sigma_m ** 2
    if self.point_m is None or abs(z - self.point_m) > JUMP_SIGMAS * math.sqrt(self.var_m2 + r):
      self.point_m, self.var_m2, self.samples = z, r, 1
      return
    k = self.var_m2 / (self.var_m2 + r)
    self.point_m += k * (z - self.point_m)
    self.var_m2 *= 1.0 - k
    self.samples += 1

  def passed(self, odometer_m: float, speed_mps: float) -> None:
    if self.point_m is not None and odometer_m - self.point_m > PASSED_M and speed_mps > 1.0:
      self.reset()

  def distance(self, odometer_m: float) -> tuple[float, float] | None:
    """(metres left to the stop point, its sigma) or None."""
    if self.point_m is None:
      return None
    return max(self.point_m - odometer_m, 0.0), math.sqrt(self.var_m2)


@dataclass(frozen=True)
class Choice:
  source: str  # 'ours', 'chestnut', 'driving_model' (the small model when chestnut is not running), 'none'
  distance_m: float | None
  ours_m: float | None
  ours_sigma_m: float | None
  model_m: float | None
  reason: str


def choose(fuser: StopPointFuser, odometer_m: float, now_s: float, model_stop_m: float | None, model_big: bool,
           model_should_stop: bool, ours_available: bool) -> Choice:
  """Chestnut first: once the driving model's plan reaches a stop it is the better point (cross-validation:
  chestnut-first with ours filling in beat either alone). Ours fills the stretch before that (far out, signs)."""
  ours = fuser.distance(odometer_m) if ours_available and now_s - fuser.updated_s <= MAX_AGE_S else None
  model_m = (model_stop_m if model_stop_m is not None and math.isfinite(model_stop_m) and 0.0 <= model_stop_m <= LABEL_RANGE_M
             else None)
  driving = 'chestnut' if model_big else 'driving_model'
  ours_m, ours_sigma = (ours[0], ours[1]) if ours else (None, None)
  if model_m is not None:
    return Choice(driving, model_m, ours_m, ours_sigma, model_m, 'plan_stops')
  if ours is not None and ours[1] <= MAX_SIGMA_M:
    return Choice('ours', ours[0], ours_m, ours_sigma, None, 'plan_clear')
  reason = ('unavailable' if not ours_available else 'stale' if now_s - fuser.updated_s > MAX_AGE_S else
            'no_stop' if ours is None else 'unsure')
  return Choice('none', None, ours_m, ours_sigma, None, reason)
