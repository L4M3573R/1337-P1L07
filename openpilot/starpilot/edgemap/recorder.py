"""Edge map recorder: compact per-drive record of where the road edges are, for learning where turns really are.

Observe only. Each sample ties the driving model's road edges (curbs, guard rails, walls) and the car's own
motion to a GPS position and heading. Later passes over the same road are aligned and averaged into a map of
"edge here" versus "opening here", and the driver's own turns mark where each turn really starts.

Raw samples are kept (not a pre-built map) so the map can be rebuilt as the method improves.
"""
from __future__ import annotations

import math
import os
import tempfile
from pathlib import Path

import numpy as np

VERSION = 1
SAMPLE_PERIOD_S = 0.2  # 5 Hz: about 1.5 m apart at 17 mph, 6 m at 65 mph
PERIOD_SLACK_S = 0.01
GPS_MAX_AGE_S = 1.0
EDGE_LOOKAHEAD_M = (5.0, 15.0, 30.0, 50.0)
MAX_DRIVES = 400
MAX_TOTAL_BYTES = 200 * 1024 * 1024

COLUMNS = (
  "t",  # s, monotonic (logMonoTime of the model frame)
  "lat", "lon",  # deg, NaN when GPS is stale or absent
  "bearing",  # deg, GPS course over ground
  "gps_speed",  # m/s
  "gps_accuracy",  # m, horizontal
  "v_ego",  # m/s, from the car
  "steering_angle",  # deg
  "yaw_rate",  # rad/s, device frame z
  "left_blinker", "right_blinker",
  "lat_active",  # openpilot steering engaged
  "standstill",
  *(f"left_edge_{int(d)}m" for d in EDGE_LOOKAHEAD_M),  # m, model frame (left is negative)
  *(f"right_edge_{int(d)}m" for d in EDGE_LOOKAHEAD_M),
  "left_edge_std", "right_edge_std",
  "lane_prob_0", "lane_prob_1", "lane_prob_2", "lane_prob_3",
)
NAN = float("nan")


def _edge_at(edge, distances) -> list[float]:
  x, y = list(edge.x), list(edge.y)
  if len(x) < 2 or len(x) != len(y):
    return [NAN] * len(distances)
  return [float(v) for v in np.interp(distances, x, y, left=NAN, right=NAN)]


def sample_row(t: float, model, car_state, car_control=None, device_motion=None, gps=None, gps_age_s: float = math.inf) -> list[float]:
  """One recorder row from the latest messages (shared by the daemon and log replays)."""
  fresh_gps = (gps is not None and 0.0 <= gps_age_s <= GPS_MAX_AGE_S and bool(getattr(gps, "hasFix", True)) and
               math.isfinite(gps.latitude) and math.isfinite(gps.longitude) and (gps.latitude, gps.longitude) != (0.0, 0.0))
  if fresh_gps:
    position = [gps.latitude, gps.longitude, gps.bearingDeg, gps.speed, gps.horizontalAccuracy]
  else:
    position = [NAN] * 5
  edges = list(model.roadEdges)
  left = _edge_at(edges[0], EDGE_LOOKAHEAD_M) if len(edges) == 2 else [NAN] * len(EDGE_LOOKAHEAD_M)
  right = _edge_at(edges[1], EDGE_LOOKAHEAD_M) if len(edges) == 2 else [NAN] * len(EDGE_LOOKAHEAD_M)
  stds = list(model.roadEdgeStds)
  stds = stds if len(stds) == 2 else [NAN, NAN]
  probs = list(model.laneLineProbs)
  probs = probs if len(probs) == 4 else [NAN] * 4
  cs = car_state
  return [
    t, *position,
    cs.vEgo, cs.steeringAngleDeg,
    device_motion.angularVelocityDevice.z if device_motion is not None else NAN,
    float(cs.leftBlinker), float(cs.rightBlinker),
    float(bool(car_control is not None and car_control.latActive)), float(cs.standstill),
    *left, *right, *stds, *probs,
  ]


class DriveLog:
  def __init__(self, drive_id: str, root: Path | None = None, period_s: float = SAMPLE_PERIOD_S):
    self.drive_id, self.root, self.period_s = drive_id, root, period_s
    self.rows: list[list[float]] = []
    self._last_t: float | None = None

  def add(self, row: list[float]) -> bool:
    t = row[0]
    # Decimate to the sample period (with slack, so 20 Hz frames keep every 4th despite float rounding).
    # A clock reset (negative step) starts sampling again.
    if self._last_t is not None and 0.0 <= t - self._last_t < self.period_s - PERIOD_SLACK_S:
      return False
    self._last_t = t
    self.rows.append(row)
    return True

  def save(self) -> Path | None:
    if not self.rows:
      return None
    return save_drive(self.drive_id, np.asarray(self.rows, dtype=np.float64), self.root)


def default_root() -> Path:
  from openpilot.starpilot.storage import starpilot_storage_root
  return starpilot_storage_root() / "edgemap"


def _check_id(drive_id: str) -> None:
  if not drive_id or any(ch in drive_id for ch in "/\\") or drive_id.startswith("."):
    raise ValueError("invalid drive id")


def save_drive(drive_id: str, rows: np.ndarray, root: Path | None = None) -> Path:
  _check_id(drive_id)
  if rows.ndim != 2 or rows.shape[1] != len(COLUMNS):
    raise ValueError("rows do not match the recorder columns")
  root = default_root() if root is None else root
  path = root / "drives" / f"{drive_id}.npz"
  path.parent.mkdir(parents=True, exist_ok=True)
  fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.", suffix=".npz")
  try:
    with os.fdopen(fd, "wb") as f:
      # Positions keep float64 precision; everything else fits float32 and halves the size.
      np.savez_compressed(f, version=np.int32(VERSION), columns=np.array(COLUMNS), position=rows[:, 1:3],
                          data=rows.astype(np.float32), t=rows[:, 0])
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, path)
  finally:
    if os.path.exists(tmp):
      os.unlink(tmp)
  prune(root)
  return path


def load_drive(path: Path) -> tuple[tuple[str, ...], np.ndarray] | None:
  """(columns, rows) with full-precision time and position, or None for unreadable or foreign files."""
  try:
    with np.load(path, allow_pickle=False) as z:
      if int(z["version"]) != VERSION:
        return None
      columns = tuple(str(c) for c in z["columns"])
      rows = z["data"].astype(np.float64)
      rows[:, 0] = z["t"]
      rows[:, 1:3] = z["position"]
      return columns, rows
  except (OSError, ValueError, KeyError):
    return None


def prune(root: Path, keep: int = MAX_DRIVES, max_bytes: int = MAX_TOTAL_BYTES) -> None:
  drives = sorted((root / "drives").glob("*.npz"), key=lambda p: p.stat().st_mtime)
  total = sum(p.stat().st_size for p in drives)
  while drives and (len(drives) > keep or total > max_bytes):
    old = drives.pop(0)
    total -= old.stat().st_size
    old.unlink(missing_ok=True)
