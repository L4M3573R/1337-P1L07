"""What the camera (the driving model) says about the road ahead, for checking the map: lane position, which branch the
car is heading onto at a split, and whether the map's curve matches what the camera sees.

Pure functions on modelV2-like objects. Shadow use only: nothing here drives. Distances are metres ahead; curvature is
signed left positive after CameraSign has learned which way the model's yaw rate points on this car (from steering).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

NEAR_WINDOW = (30.0, 110.0)      # metres ahead the camera sees well enough to judge the road's shape
RIGHTMOST_EDGE_M = 3.0           # a road edge this close (lane centre to edge) means no lane beyond it
OUTER_LINE_PROB = 0.3            # the next lane line out is barely seen
MISMATCH_K = 0.002               # map and camera curvature differ by more than this (500 m radius)...
MISMATCH_SHARE = 0.4             # ...and by more than this share of the map's
MIN_SPEED = 5.0                  # m/s; the camera's path is unreliable when crawling


def camera_curvature(model) -> tuple[np.ndarray, np.ndarray] | None:
  """(distance ahead, curvature) along the camera's planned path, curvature in the model's own sign."""
  try:
    rates, vel, pos = list(model.orientationRate.z), list(model.velocity.x), list(model.position.x)
  except AttributeError:
    return None
  if not rates or not len(rates) == len(vel) == len(pos):
    return None
  d = np.maximum(np.array(pos, dtype=float), 0.0)
  k = np.array([r / v if v >= MIN_SPEED else 0.0 for r, v in zip(rates, vel, strict=True)], dtype=float)
  return d, k


def near_curvature(d: np.ndarray, k: np.ndarray, window: tuple[float, float] = NEAR_WINDOW) -> float | None:
  inside = (d >= window[0]) & (d <= window[1])
  return float(np.mean(k[inside])) if inside.sum() >= 2 else None


@dataclass(frozen=True)
class LanePosition:
  right_edge_m: float | None     # lateral distance to the right road edge
  left_edge_m: float | None
  outer_right_prob: float | None  # probability of the lane line beyond the car's right line
  outer_left_prob: float | None
  rightmost: bool
  leftmost: bool


def lane_position(model) -> LanePosition:
  """Which lane the car is in, from the road edges and the outer lane lines (openpilot frame: y to the right)."""
  try:
    edges = [float(e.y[0]) for e in model.roadEdges] if len(model.roadEdges) >= 2 else []
    probs = list(model.laneLineProbs)
  except (AttributeError, IndexError):
    edges, probs = [], []
  left_edge = abs(edges[0]) if edges else None
  right_edge = abs(edges[1]) if len(edges) > 1 else None
  outer_left = float(probs[0]) if len(probs) == 4 else None
  outer_right = float(probs[3]) if len(probs) == 4 else None
  rightmost = right_edge is not None and right_edge <= RIGHTMOST_EDGE_M and (outer_right is None or outer_right < OUTER_LINE_PROB)
  leftmost = left_edge is not None and left_edge <= RIGHTMOST_EDGE_M and (outer_left is None or outer_left < OUTER_LINE_PROB)
  return LanePosition(right_edge, left_edge, outer_right, outer_left, rightmost, leftmost)


class SignLearner:
  """Learns whether two curvature signals share a sign convention, from their sign agreement in real curves. Used for
  the camera's yaw rate against the car's measured curvature, and the measured curvature against the map (left
  positive), so nothing here assumes a convention that may differ by car port."""

  def __init__(self):
    self.agree = self.disagree = 0

  def observe(self, a: float, b: float) -> None:
    if abs(a) >= 0.003 and abs(b) >= 0.002:
      if a * b > 0:
        self.agree += 1
      else:
        self.disagree += 1

  @property
  def factor(self) -> float | None:
    if self.agree + self.disagree < 50:
      return None  # not enough curves yet to say
    return 1.0 if self.agree >= self.disagree else -1.0


def which_branch(camera_k: float, branches: dict[str, float]) -> tuple[str, dict[str, float]]:
  """The branch whose near curvature (left positive) the camera's path matches best, with each branch's error."""
  errors = {name: abs(camera_k - k) for name, k in branches.items() if k is not None and math.isfinite(k)}
  return (min(errors, key=errors.get) if errors else 'unknown'), errors


def map_agrees(camera_k: float, map_k: float) -> bool:
  """Whether the map's near curve matches the camera's (left positive both)."""
  diff = abs(camera_k - map_k)
  return diff <= MISMATCH_K or diff <= MISMATCH_SHARE * abs(map_k)
