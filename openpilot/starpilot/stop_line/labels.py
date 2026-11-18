"""Stop-point labels from a drive's own stops, for training the stop-line finder.

A stop is a standstill of at least MIN_STOP_S after a real approach. Where the car came to rest as the
first car in line (no radar lead close ahead) is taken as where the stop belongs, painted line or not,
and every model frame on the approach is labeled with the distance still to drive to that point.
Queue stops behind a car say nothing about where the line is and are left out; so is the last stop of
a drive (parking). Pure numpy over 20 Hz model-frame arrays; no IPC, no logs.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

STOP_SPEED_MPS = 0.3
MIN_STOP_S = 1.0
APPROACH_SPEED_MPS = 4.0  # the car must have been driving this fast within LABEL_RANGE_M before the stop
LABEL_RANGE_M = 120.0
QUEUE_LEAD_M = 12.0  # a radar lead this close at rest means a queue stop
NO_STOP_SPEED_MPS = 5.0
MODEL_AGREE_M = 15.0
MODEL_STOP_SPEED_MPS = 0.5
ROLL_SPEED_MPS = 3.0  # a rolling stop slows below this (about 6.7 mph) without coming to rest
ROLL_APPROACH_MPS = 6.0
ROLL_LEAD_M = 15.0  # a radar lead this close at the slowest point means slowing for traffic, not a stop


@dataclass(frozen=True)
class Stop:
  rest_index: int
  front: bool  # first in line: the rest point is where the stop belongs
  driver_braked: bool  # the driver braked on the last stretch rather than openpilot stopping alone
  model_agreed: bool  # chestnut's planned stop was within MODEL_AGREE_M of the rest point on the approach
  sign_seen: bool  # the sign detector saw a stop sign on the approach
  last: bool  # the last stop of the drive (parking), never a label


def model_stop_distance(position_x, velocity_x) -> float:
  """Distance along the driving model's plan to where its planned speed reaches zero, or NaN."""
  x = np.asarray(position_x, dtype=np.float64)
  v = np.asarray(velocity_x, dtype=np.float64)
  if x.size == 0 or x.size != v.size or not np.all(np.isfinite(v)):
    return float('nan')
  below = np.flatnonzero(v < MODEL_STOP_SPEED_MPS)
  if below.size == 0:
    return float('nan')
  i = int(below[0])
  if i == 0:
    return float(max(x[0], 0.0))
  # interpolate where the speed crosses the threshold between the two plan points
  f = (v[i - 1] - MODEL_STOP_SPEED_MPS) / max(v[i - 1] - v[i], 1e-6)
  return float(x[i - 1] + f * (x[i] - x[i - 1]))


def travel(t, v) -> np.ndarray:
  """Cumulative distance driven at each frame (trapezoid on speed)."""
  t = np.asarray(t, dtype=np.float64)
  v = np.clip(np.asarray(v, dtype=np.float64), 0.0, None)
  s = np.zeros_like(t)
  if t.size > 1:
    s[1:] = np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(t))
  return s


def find_stops(t, v, s, lead_d, brake, model_stop, sign) -> list[Stop]:
  t, v, s = (np.asarray(a, dtype=np.float64) for a in (t, v, s))
  lead_d, model_stop = np.asarray(lead_d, dtype=np.float64), np.asarray(model_stop, dtype=np.float64)
  brake, sign = np.asarray(brake, dtype=bool), np.asarray(sign, dtype=bool)
  stopped = v < STOP_SPEED_MPS
  stops: list[Stop] = []
  i, n = 0, t.size
  moved_from = 0  # first frame after the previous standstill: a creep-up after a stop is not an approach
  while i < n:
    if not stopped[i]:
      i += 1
      continue
    j = i
    while j < n and stopped[j]:
      j += 1
    if t[j - 1] - t[i] >= MIN_STOP_S:
      approach = (s >= s[i] - LABEL_RANGE_M) & (np.arange(n) < i) & (np.arange(n) >= moved_from)
      moved_from = j
      if np.any(v[approach] > APPROACH_SPEED_MPS):
        at_rest = slice(i, min(j, i + 20))
        lead_close = bool(np.any(np.isfinite(lead_d[at_rest]) & (lead_d[at_rest] < QUEUE_LEAD_M)))
        near = approach & (s >= s[i] - 30.0)
        remaining = s[i] - s
        agree = approach & np.isfinite(model_stop) & (np.abs(model_stop - remaining) < MODEL_AGREE_M) & (remaining > 10.0)
        stops.append(Stop(i, not lead_close, bool(np.any(brake[near])), bool(np.count_nonzero(agree) >= 10),
                          bool(np.any(sign[approach])), j >= n))
    i = j
  return stops


@dataclass(frozen=True)
class Roll:
  slow_index: int  # the slowest frame; where the line is comes from the video (lines.py), not from here
  min_speed: float
  sign_seen: bool


def find_rolls(t, v, s, lead_d, sign) -> list[Roll]:
  """Slow-downs below ROLL_SPEED_MPS that never reach a stop, after a real approach and with no lead close."""
  v, s = np.asarray(v, dtype=np.float64), np.asarray(s, dtype=np.float64)
  lead_d, sign = np.asarray(lead_d, dtype=np.float64), np.asarray(sign, dtype=bool)
  slow = v < ROLL_SPEED_MPS
  edges = np.flatnonzero(np.diff(np.r_[0, slow.astype(np.int8), 0]))
  rolls = []
  for a, b in zip(edges[::2], edges[1::2], strict=True):
    if np.any(v[a:b] < STOP_SPEED_MPS) or b >= v.size:
      continue
    k = a + int(np.argmin(v[a:b]))
    before = (s >= s[k] - LABEL_RANGE_M) & (np.arange(v.size) < a)
    after = (s <= s[k] + 60.0) & (np.arange(v.size) >= b)
    if not (np.any(v[before] > ROLL_APPROACH_MPS) and np.any(v[after] > NO_STOP_SPEED_MPS)):
      continue
    if np.isfinite(lead_d[k]) and lead_d[k] < ROLL_LEAD_M:
      continue
    rolls.append(Roll(k, float(v[k]), bool(np.any(sign[before]))))
  return rolls


def label_frames(t, v, s, stops: list[Stop], rolls: list[Roll] = ()) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
  """Per frame: stop distance (NaN = unknown), has-stop target (1 stop ahead, 0 none, -1 ignore), stop id (-1 none).

  Frames on a front stop's approach get the distance to its rest point. Frames moving faster than
  NO_STOP_SPEED_MPS with no stop of any kind in the next LABEL_RANGE_M get 'no stop'. Everything else
  (queue approaches, parking, standing still, crawling) is ignored.
  """
  t, v, s = (np.asarray(a, dtype=np.float64) for a in (t, v, s))
  n = t.size
  dist = np.full(n, np.nan)
  target = np.full(n, -1, dtype=np.int8)
  stop_id = np.full(n, -1, dtype=np.int32)
  rest_s = np.array([s[st.rest_index] for st in stops] + [s[r.slow_index] for r in rolls])
  # 'no stop' needs no stop of any kind ahead (queue, parking and rolling stops count: something slowed the car)
  for k in range(n):
    if v[k] <= NO_STOP_SPEED_MPS:
      continue
    ahead = rest_s[(rest_s >= s[k])] if rest_s.size else rest_s
    if ahead.size == 0:
      # unknown future past the end of the log within range: only label when the log runs on far enough
      if s[-1] - s[k] >= LABEL_RANGE_M:
        target[k] = 0
    elif ahead.min() - s[k] > LABEL_RANGE_M:
      target[k] = 0
  prev_rest = -1
  for sid, st in enumerate(stops):
    r = st.rest_index
    lo = prev_rest + 1
    prev_rest = r
    if not st.front or st.last:
      continue
    for k in range(lo, r):
      d = s[r] - s[k]
      if d <= LABEL_RANGE_M and v[k] >= STOP_SPEED_MPS:
        dist[k], target[k], stop_id[k] = d, 1, sid
  return dist, target, stop_id
