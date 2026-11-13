"""Curves on the road ahead, from the stored road-line tiles (the same tiles blinker turns use).

Pure geometry: the car's road is matched by position and heading (highways included), followed ahead through
connected lines up to ~1 km, resampled every STEP_M, and its curvature measured with a wide three-point circle so
map vertex jitter does not read as a sharp bend. Unknown (no tiles, no matching road) is None, never a guess.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from openpilot.starpilot.navigation import side_road
from openpilot.starpilot.navigation.road_tiles import RoadTile

AHEAD_M = 1000.0
STEP_M = 10.0
CHORD_M = 30.0          # half-width of the three-point circle: smooths polyline corners over ~60 m
CURVE_MIN_K = 0.002     # 500 m radius: gentler bends are straight for speed purposes
MIN_CURVE_LENGTH_M = 20.0
MAX_K = 0.05            # 20 m radius; tighter is an intersection corner, not a curve to drive through


@dataclass(frozen=True)
class MapCurve:
  start_m: float        # along the road from the car
  apex_m: float
  end_m: float
  curvature: float      # 1/m, representative tightness (80th percentile through the curve, not one spike)
  direction: int        # +1 left, -1 right
  apex_east: float = 0.0   # metres from the car
  apex_north: float = 0.0
  apex_bearing: float = 0.0  # degrees, direction of travel at the apex

  @property
  def radius_m(self) -> float:
    return 1.0 / max(self.curvature, 1e-6)


def road_ahead(tiles: list[RoadTile], latitude: float, longitude: float, bearing_deg: float,
               ahead_m: float = AHEAD_M, prefer: str | None = None) -> np.ndarray | None:
  """Points (N, 2), metres east/north of the car, along the matched road ahead; None when the road is unknown.
  prefer ('left'/'right', from the blinker) follows the branch to that side at a split ahead (an exit ramp)."""
  if not tiles or not all(math.isfinite(v) for v in (latitude, longitude, bearing_deg)):
    return None
  heading = np.array([math.sin(math.radians(bearing_deg)), math.cos(math.radians(bearing_deg))])
  lines = side_road._polylines(tiles, latitude, longitude, ahead_m + 60.0, include_motorway=True)
  start = side_road._match(lines, heading)
  if start is None:
    return None
  path, _ = side_road._path(lines, start, heading, ahead_m, prefer)
  return path if len(path) >= 2 else None


def resample(path: np.ndarray, step_m: float = STEP_M) -> np.ndarray:
  seg = np.hypot(*np.diff(path, axis=0).T)
  keep = np.concatenate([[True], seg > 1e-3])
  path, seg = path[keep], seg[seg > 1e-3]
  if len(path) < 2:
    return path
  along = np.concatenate([[0.0], np.cumsum(seg)])
  s = np.arange(0.0, along[-1] + 1e-6, step_m)
  return np.stack([np.interp(s, along, path[:, 0]), np.interp(s, along, path[:, 1])], axis=1)


def curvature_profile(path: np.ndarray, step_m: float = STEP_M,
                      chord_m: float = CHORD_M) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
  """(distance along the road, signed curvature (left positive), resampled points) every step_m."""
  pts = resample(path, step_m)
  n = len(pts)
  k = np.zeros(n)
  h = max(1, int(round(chord_m / step_m)))
  for i in range(h, n - h):
    a, b, c = pts[i - h], pts[i], pts[i + h]
    ab, bc, ac = b - a, c - b, c - a
    cross = ab[0] * bc[1] - ab[1] * bc[0]
    denom = np.hypot(*ab) * np.hypot(*bc) * np.hypot(*ac)
    k[i] = 0.0 if denom < 1e-6 else 2.0 * cross / denom
  return np.arange(n) * step_m, np.clip(k, -MAX_K, MAX_K), pts


def find_curves(distances: np.ndarray, curvature: np.ndarray, pts: np.ndarray | None = None) -> list[MapCurve]:
  """Contiguous bends tighter than CURVE_MIN_K, one per direction change."""
  curves = []
  mag = np.abs(curvature)
  i, n = 0, len(mag)
  while i < n:
    if mag[i] < CURVE_MIN_K:
      i += 1
      continue
    sign = 1 if curvature[i] > 0 else -1
    j = i
    while j < n and mag[j] >= CURVE_MIN_K and (curvature[j] > 0) == (sign > 0):
      j += 1
    if distances[j - 1] - distances[i] >= MIN_CURVE_LENGTH_M:
      seg = mag[i:j]
      tight = np.flatnonzero(seg >= 0.9 * seg.max())
      apex = i + int(tight[len(tight) // 2])  # middle of the tightest stretch (a constant arc has no single peak)
      east = north = bearing = 0.0
      if pts is not None and len(pts) == n:
        east, north = float(pts[apex, 0]), float(pts[apex, 1])
        d = pts[min(apex + 1, n - 1)] - pts[max(apex - 1, 0)]
        bearing = math.degrees(math.atan2(d[0], d[1])) % 360.0
      curves.append(MapCurve(float(distances[i]), float(distances[apex]), float(distances[j - 1]),
                             float(np.percentile(seg, 80)), sign, east, north, bearing))
    i = j
  return curves


@dataclass(frozen=True)
class RoadProfile:
  distances: np.ndarray     # every STEP_M along the road from the car
  curvature: np.ndarray     # signed, left positive
  points: np.ndarray        # metres east/north of the car
  curves: list[MapCurve]
  road_class: int = -1      # the class of the road the car is matched to (road_tiles LINK: on a ramp)


BRANCH_SEPARATION_M = 8.0   # a side branch counts as a different way ahead once it is this far from the straight one
BRANCH_WITHIN_M = 600.0


def _profile_of(path: np.ndarray, road_class: int) -> RoadProfile:
  distances, curvature, pts = curvature_profile(path)
  return RoadProfile(distances, curvature, pts, find_curves(distances, curvature, pts), road_class)


def branches_from_lines(lines, start, heading: np.ndarray, ahead_m: float = AHEAD_M) -> dict[str, RoadProfile]:
  """The straight way ahead, plus a 'left'/'right' branch where the road splits that way within BRANCH_WITHIN_M."""
  road_class = int(lines[start[0]][0])
  straight, _ = side_road._path(lines, start, heading, ahead_m)
  if len(straight) < 2:
    return {}
  out = {'straight': _profile_of(straight, road_class)}
  base = resample(straight, STEP_M)
  for side in ('left', 'right'):
    path, _ = side_road._path(lines, start, heading, ahead_m, prefer=side)
    if len(path) < 2:
      continue
    other = resample(path, STEP_M)
    n = min(len(base), len(other), int(BRANCH_WITHIN_M / STEP_M) + 1)
    if n >= 2 and float(np.max(np.hypot(*(base[:n] - other[:n]).T))) > BRANCH_SEPARATION_M:
      out[side] = _profile_of(path, road_class)
  return out


def branch_profiles(tiles: list[RoadTile], latitude: float, longitude: float, bearing_deg: float,
                    ahead_m: float = AHEAD_M) -> dict[str, RoadProfile]:
  """branches_from_lines for the car's position; empty when the road is unknown."""
  if not tiles or not all(math.isfinite(v) for v in (latitude, longitude, bearing_deg)):
    return {}
  heading = np.array([math.sin(math.radians(bearing_deg)), math.cos(math.radians(bearing_deg))])
  lines = side_road._polylines(tiles, latitude, longitude, ahead_m + 60.0, include_motorway=True)
  start = side_road._match(lines, heading)
  return {} if start is None else branches_from_lines(lines, start, heading, ahead_m)


def profile_ahead(tiles: list[RoadTile], latitude: float, longitude: float, bearing_deg: float,
                  ahead_m: float = AHEAD_M, prefer: str | None = None) -> RoadProfile | None:
  """The road ahead in STEP_M chunks with its curvature, and the curves found in it; None when the road is unknown."""
  path = road_ahead(tiles, latitude, longitude, bearing_deg, ahead_m, prefer)
  if path is None:
    return None
  distances, curvature, pts = curvature_profile(path)
  return RoadProfile(distances, curvature, pts, find_curves(distances, curvature, pts))


def curves_ahead(tiles: list[RoadTile], latitude: float, longitude: float, bearing_deg: float,
                 ahead_m: float = AHEAD_M) -> list[MapCurve] | None:
  profile = profile_ahead(tiles, latitude, longitude, bearing_deg, ahead_m)
  return None if profile is None else profile.curves
