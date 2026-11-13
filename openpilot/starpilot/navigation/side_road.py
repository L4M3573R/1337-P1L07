"""The next road (or mapped driveway) branching off to one side ahead of the car, from the stored road-line tiles.

Pure geometry on decoded RoadTile lines: no network or process state. The car's road is matched by position and
heading, followed ahead (continuing through connected lines), and every other line that meets it is a junction.
A junction whose branch leaves to the requested side at 25-155 degrees is a turn candidate; the nearest one ahead
is returned with its distance along the road. Unknown (no tiles, no matching road) is None, never a guess.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from openpilot.starpilot.navigation.road_tiles import (
  CLASS_NAMES, DATA_ZOOM, EXTENT, LINK, MOTORWAY, RoadTile, meters_per_tile, world_xy,
)

MATCH_M = 12.0  # the car's road must pass this close
MATCH_COS = 0.82  # and point within ~35 degrees of the heading
JOIN_M = 4.0  # line ends this close are connected; a branch vertex this close to the road meets it
CONTINUE_COS = 0.82
BRANCH_MIN_M = 8.0  # a branch must leave the road by at least this much to count
BRANCH_PROBE_M = 20.0
ANGLE_RANGE = (25.0, 155.0)
MAX_AHEAD_M = 220.0
BEHIND_M = 5.0  # a junction just passed still counts (GPS lag)


@dataclass(frozen=True)
class SideRoad:
  distance_m: float  # along the car's road to the junction (negative: just passed)
  angle_deg: float  # branch direction relative to the road, positive to the left
  road_class: str


def _min_distance(line: np.ndarray) -> float:
  """Closest approach of a polyline to the car (origin), over its segments, not just its vertices."""
  a, b = line[:-1], line[1:]
  ab = b - a
  length2 = np.maximum((ab * ab).sum(axis=1), 1e-9)
  t = np.clip(-(a * ab).sum(axis=1) / length2, 0.0, 1.0)
  q = a + ab * t[:, None]
  return float(np.min(np.hypot(q[:, 0], q[:, 1])))


def _polylines(tiles: list[RoadTile], latitude: float, longitude: float, radius_m: float, include_motorway: bool = False):
  """[(class, points (N, 2) in metres east/north of the car)] for lines that come within radius."""
  cx, cy = world_xy(latitude, longitude, DATA_ZOOM)
  mpt = meters_per_tile(latitude, DATA_ZOOM)
  out = []
  for tile in tiles:
    for road_class, (points, offsets) in tile.lines.items():
      if road_class == MOTORWAY and not include_motorway:
        continue  # never a turn: highways are lane changes (curve speed passes include_motorway)
      east = ((tile.key.x + points[:, 0] / EXTENT) - cx) * mpt
      north = -((tile.key.y + points[:, 1] / EXTENT) - cy) * mpt
      xy = np.stack([east, north], axis=1)
      for i in range(len(offsets) - 1):
        line = xy[offsets[i]:offsets[i + 1]]
        if len(line) >= 2 and _min_distance(line) <= radius_m:
          out.append((road_class, line.astype(np.float64)))
  return out


def _project(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> tuple[float, np.ndarray, float]:
  ab = b - a
  length2 = float(ab @ ab)
  t = 0.0 if length2 <= 0 else float(np.clip((p - a) @ ab / length2, 0.0, 1.0))
  q = a + t * ab
  return float(np.hypot(*(p - q))), q, t


def _match(lines, heading: np.ndarray):
  """(line index, segment index, projection point, direction +1/-1) of the car's road, or None."""
  best = None
  origin = np.zeros(2)
  for li, (_, line) in enumerate(lines):
    for si in range(len(line) - 1):
      a, b = line[si], line[si + 1]
      seg = b - a
      length = float(np.hypot(*seg))
      if length < 0.5:
        continue
      dist, q, _ = _project(origin, a, b)
      if dist > MATCH_M:
        continue
      cos = float(seg @ heading) / length
      if abs(cos) < MATCH_COS:
        continue
      if best is None or dist < best[0]:
        best = (dist, li, si, q, 1 if cos > 0 else -1)
  return None if best is None else best[1:]


PREFER_WITHIN_M = 600.0  # a blinker picks the branch to its side at splits this close ahead (an exit or a fork)


def _branch_direction(ordered: np.ndarray, probe_m: float = BRANCH_PROBE_M) -> np.ndarray:
  """Direction from a line's first point to the point probe_m along it (ramps diverge slowly)."""
  seg = np.hypot(*np.diff(ordered, axis=0).T)
  along = np.concatenate([[0.0], np.cumsum(seg)])
  far = ordered[min(int(np.searchsorted(along, probe_m)), len(ordered) - 1)]
  v = far - ordered[0]
  return v / max(float(np.hypot(*v)), 1e-6)


def _take_branch(lines, pts: list, used: set, prefer: str) -> list:
  """A ramp often leaves from the middle of the road's line, not its end: with a blinker, cut the path at the nearest
  junction within PREFER_WITHIN_M where a line peels off to that side, and follow that line instead."""
  along = 0.0
  for s in range(len(pts) - 1):
    a, b = pts[s], pts[s + 1]
    seg = float(np.hypot(*(b - a)))
    if seg < 0.5:
      continue
    if along > PREFER_WITHIN_M:
      break
    d = (b - a) / seg
    best = None
    for lj, (_, other) in enumerate(lines):
      if lj in used:
        continue
      for k, ordered in ((0, other), (len(other) - 1, other[::-1])):
        dist, q, t = _project(other[k], a, b)
        if dist > JOIN_M or len(ordered) < 2:
          continue
        bdir = _branch_direction(ordered)
        turn = float(d[0] * bdir[1] - d[1] * bdir[0])
        if float(bdir @ d) >= CONTINUE_COS and (turn > 0.02 if prefer == 'left' else turn < -0.02):
          if best is None or t < best[0]:
            best = (t, q, lj, ordered)
    if best is not None:
      used.add(best[2])
      return pts[:s + 1] + [best[1]] + list(best[3][1:])
    along += seg
  return pts


def _path(lines, start, heading: np.ndarray, max_m: float, prefer: str | None = None):
  """Points along the car's road ahead, continuing through connected lines, with the line indices used.

  At a split the straightest continuation is followed (stay on the highway), unless prefer is 'left' or 'right'
  (a blinker): then, within PREFER_WITHIN_M, the branch peeling off to that side is followed (the exit ramp)."""
  li, si, q, direction = start
  line = lines[li][1]
  pts = [q]
  idx = range(si + 1, len(line)) if direction > 0 else range(si, -1, -1)
  pts += [line[i] for i in idx]
  used = {li}
  if prefer in ('left', 'right'):
    pts = _take_branch(lines, pts, used, prefer)
  total = sum(float(np.hypot(*(pts[i + 1] - pts[i]))) for i in range(len(pts) - 1))
  while total < max_m:
    end, prev = pts[-1], pts[-2] if len(pts) > 1 else pts[-1] - heading
    d = end - prev
    d = d / max(float(np.hypot(*d)), 1e-6)
    candidates = []  # (cos to the current direction, signed turn left-positive, line index, points in travel order)
    for lj, (_, other) in enumerate(lines):
      if lj in used:
        continue
      for k, ordered in ((0, other), (len(other) - 1, other[::-1])):
        if float(np.hypot(*(other[k] - end))) <= JOIN_M:
          step = ordered[1] - ordered[0]
          cos = float(step @ d) / max(float(np.hypot(*step)), 1e-6)
          if cos >= CONTINUE_COS:
            b = _branch_direction(ordered)
            candidates.append((cos, float(d[0] * b[1] - d[1] * b[0]), lj, ordered))
    if not candidates:
      break
    toward = [c for c in candidates if prefer == 'left' and c[1] > 0.02 or prefer == 'right' and c[1] < -0.02]
    if toward and total <= PREFER_WITHIN_M:
      choice = max(toward, key=lambda c: abs(c[1]))  # the branch peeling off most to the blinker's side
    else:
      choice = max(candidates, key=lambda c: c[0])  # straightest: stay on the road being driven
    nxt = (choice[2], choice[3])
    used.add(nxt[0])
    for p in nxt[1][1:]:
      total += float(np.hypot(*(p - pts[-1])))
      pts.append(p)
  return np.array(pts), used


def _branch_angle(path_dir: np.ndarray, junction: np.ndarray, branch: np.ndarray) -> float | None:
  """Signed angle (left positive) of the branch leaving the junction, aimed at the point BRANCH_PROBE_M along it
  (or its end if it is shorter, but at least BRANCH_MIN_M away)."""
  pts = np.vstack([junction, branch])
  seg = np.hypot(*np.diff(pts, axis=0).T)
  along = np.concatenate([[0.0], np.cumsum(seg)])
  if along[-1] < BRANCH_MIN_M:
    return None
  target = min(BRANCH_PROBE_M, along[-1])
  i = int(np.searchsorted(along, target))
  i = min(max(i, 1), len(pts) - 1)
  frac = (target - along[i - 1]) / max(seg[i - 1], 1e-6)
  far = pts[i - 1] + (pts[i] - pts[i - 1]) * frac
  v = far - junction
  if float(np.hypot(*v)) < BRANCH_MIN_M * 0.5:
    return None
  cross = path_dir[0] * v[1] - path_dir[1] * v[0]
  return math.degrees(math.atan2(cross, float(path_dir @ v)))


def _cross(a, b, c, d):
  """Intersection of segments ab and cd as (t along ab, u along cd), or None."""
  r, q = b - a, d - c
  den = r[0] * q[1] - r[1] * q[0]
  if abs(den) < 1e-9:
    return None
  w = c - a
  t = (w[0] * q[1] - w[1] * q[0]) / den
  u = (w[0] * r[1] - w[1] * r[0]) / den
  return (t, u) if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0 else None


def _junctions(path, seg_len, along, lines, used):
  """(distance along the road, point, path segment, [branch polylines leaving the point], class) for every place
  another line meets the car's road: a vertex within JOIN_M of it, or a segment crossing it."""
  for lj, (road_class, other) in enumerate(lines):
    for k in range(len(other)):
      if lj in used and k in (0, len(other) - 1):
        continue  # the road's own joints are not junctions
      for s in range(len(path) - 1):
        if seg_len[s] < 0.5:
          continue
        dist, q, t = _project(other[k], path[s], path[s + 1])
        if dist <= JOIN_M:
          yield along[s] + t * seg_len[s], q, s, (other[k:], other[k::-1]), road_class
          break
    if lj in used:
      continue
    for k in range(len(other) - 1):
      for s in range(len(path) - 1):
        if seg_len[s] < 0.5:
          continue
        hit = _cross(path[s], path[s + 1], other[k], other[k + 1])
        if hit is None:
          continue
        t, u = hit
        q = path[s] + (path[s + 1] - path[s]) * t
        yield along[s] + t * seg_len[s], q, s, (other[k + 1:], other[k::-1]), road_class


def find(tiles: list[RoadTile], latitude: float, longitude: float, bearing_deg: float, side: str,
         max_ahead_m: float = MAX_AHEAD_M) -> SideRoad | None:
  """The nearest road branching off to `side` ('left' or 'right') within max_ahead_m along the car's road."""
  if side not in ('left', 'right') or not all(math.isfinite(v) for v in (latitude, longitude, bearing_deg)):
    return None
  heading = np.array([math.sin(math.radians(bearing_deg)), math.cos(math.radians(bearing_deg))])
  lines = _polylines(tiles, latitude, longitude, max_ahead_m + 60.0)
  start = _match(lines, heading)
  if start is None:
    return None
  path, used = _path(lines, start, heading, max_ahead_m + BEHIND_M)
  if len(path) < 2:
    return None
  seg_len = np.hypot(*np.diff(path, axis=0).T)
  along = np.concatenate([[0.0], np.cumsum(seg_len)])
  best = None
  for distance, q, s_idx, branches, road_class in _junctions(path, seg_len, along, lines, used):
    if not -BEHIND_M <= distance <= max_ahead_m:
      continue
    path_dir = (path[s_idx + 1] - path[s_idx]) / seg_len[s_idx]
    for branch in branches:
      angle = _branch_angle(path_dir, q, branch)
      if angle is None or (side == 'left') != (angle > 0) or not ANGLE_RANGE[0] <= abs(angle) <= ANGLE_RANGE[1]:
        continue
      if best is None or distance < best.distance_m:
        name = CLASS_NAMES[road_class] if road_class < len(CLASS_NAMES) else 'unknown'
        best = SideRoad(round(float(distance), 1), round(angle, 1), 'link' if road_class == LINK else name)
  return best


def matches_road(tiles: list[RoadTile], latitude: float, longitude: float, bearing_deg: float) -> bool:
  """Whether the stored tiles know the road the car is on (so 'no side road' is an answer, not a gap)."""
  if not all(math.isfinite(v) for v in (latitude, longitude, bearing_deg)):
    return False
  heading = np.array([math.sin(math.radians(bearing_deg)), math.cos(math.radians(bearing_deg))])
  return _match(_polylines(tiles, latitude, longitude, MATCH_M + 5.0), heading) is not None
