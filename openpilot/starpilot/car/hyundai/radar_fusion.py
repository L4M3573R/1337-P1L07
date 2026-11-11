"""Camera + front-radar lead fusion for Hyundai cars with a usable front radar.

The stock radard lets the camera gate every lead: a lead exists only while the model's lead
probability is above 0.5, and the radar merely refines it. On this radar that throws away good
information, so leadOne is fused in tandem instead:

* Camera and radar agree: the radar track is the lead (stock behavior).
* Radar hold: a track the camera has confirmed as the lead stays the lead through camera
  dropouts while it remains in the predicted path.
* Radar pickup: while the camera has no confident lead, a moving track that has stayed in the
  predicted path becomes the lead (or replaces a clearly farther held/low-confidence lead).
  A confident camera lead is never overridden by a closer radar track: on recorded drives those
  were almost always adjacent-lane cars crossing the predicted path during passes.

Stationary radar returns (signs, bridges, parked cars) never become a lead without the camera,
except through radard's existing low-speed override. Radar-only leads carry the camera's real
probability, so camera-gated consumers (FCW, model lead trajectory) keep ignoring them.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

RADAR_FUSION_CARS = frozenset({"HYUNDAI_ELANTRA_HEV_2024"})

RADAR_TO_CAMERA = 1.52        # m, radar sits ahead of the model frame origin (radard)
CONFIRM_PROB = 0.5            # camera probability that confirms a radar track
PATH_HALF_WIDTH = 1.2         # m, track must be this close to the predicted path centerline
HOLD_PATH_HALF_WIDTH = 1.5    # m, slightly wider while holding an already-confirmed car
HOLD_MAX_FRAMES = 60          # model frames (3 s) a confirmed car is held without the camera
PICKUP_MIN_FRAMES = 20        # model frames (1 s) a track must stay in path before pickup
PICKUP_MIN_SPEED = 3.0        # m/s ground speed; slower returns need the camera
PICKUP_MAX_DISTANCE = 100.0   # m
PICKUP_MIN_DISTANCE = 2.0     # m
PICKUP_CLOSER_MARGIN = 5.0    # m, pickup must be this much closer than the camera's lead


def supported(car_fingerprint: str, radar_unavailable: bool) -> bool:
  return not radar_unavailable and str(car_fingerprint) in RADAR_FUSION_CARS


def path_offset(track, model) -> float:
  """Lateral distance (m) of a radar track from the model's predicted path at its range."""
  xs, ys = model.position.x, model.position.y
  path_y = float(np.interp(track.dRel + RADAR_TO_CAMERA, xs, ys)) if len(xs) >= 2 else 0.0
  return abs(track.yRel + path_y)  # radar yRel is the negated model y


@dataclass
class _HeldLead:
  track_id: int = -1
  frames_without_camera: int = 0


class TandemLeadPolicy:
  def __init__(self) -> None:
    self.in_path_frames: dict[int, int] = {}
    self.held = _HeldLead()
    # Decision log for remote review: "camera", "hold", "pickup" or "none" for the last frame.
    self.last_action = 'none'
    self.last_pickup_id = -1
    self.last_lead_prob = 0.0

  def _update_in_path(self, tracks: dict, model) -> None:
    for track_id in list(self.in_path_frames):
      if track_id not in tracks:
        del self.in_path_frames[track_id]
    for track_id, track in tracks.items():
      if path_offset(track, model) < PATH_HALF_WIDTH:
        self.in_path_frames[track_id] = self.in_path_frames.get(track_id, 0) + 1
      else:
        self.in_path_frames[track_id] = 0

  def _pickup(self, tracks: dict) -> Any | None:
    candidates = [track for track_id, track in tracks.items()
                  if self.in_path_frames.get(track_id, 0) >= PICKUP_MIN_FRAMES and track.vLead >= PICKUP_MIN_SPEED and
                  PICKUP_MIN_DISTANCE < track.dRel < PICKUP_MAX_DISTANCE]
    return min(candidates, key=lambda track: track.dRel) if candidates else None

  def lead_one(self, base: dict[str, Any], tracks: dict, model, lead_prob: float) -> dict[str, Any]:
    """Fuse radard's camera-gated leadOne (`base`) with the radar tracks."""
    self._update_in_path(tracks, model)
    present = bool(base.get('present', False))
    camera_confirmed = present and base.get('radar', False) and base.get('modelProb', 0.0) > CONFIRM_PROB

    camera_confident = lead_prob > CONFIRM_PROB
    lead = base
    self.last_action, self.last_pickup_id, self.last_lead_prob = 'none', -1, float(lead_prob)
    if camera_confirmed:
      self.last_action = 'camera'
      self.held = _HeldLead(int(base['radarTrackId']), 0)
    else:
      held = tracks.get(self.held.track_id)
      self.held.frames_without_camera += 1
      # Hold through a camera dropout, or keep radar measuring the car the camera still reports
      # (vision-only because radard found no match). A confident camera lead on a different car
      # wins: that is a lane change or pass, and the old car is leaving the path.
      same_car = present and held is not None and abs(held.dRel - base['dRel']) < max(5.0, 0.25 * base['dRel'])
      dropout = not camera_confident and (not present or held is not None and held.dRel < base['dRel'] + PICKUP_CLOSER_MARGIN)
      if (held is not None and self.held.frames_without_camera <= HOLD_MAX_FRAMES and
          path_offset(held, model) < HOLD_PATH_HALF_WIDTH and (dropout or same_car)):
        lead = held.get_RadarState(lead_prob)
        self.last_action = 'hold'
      else:
        self.held = _HeldLead()

    # A confident camera lead decides which car is followed: replays showed radar tracks "closer"
    # than it are mostly adjacent-lane cars crossing the predicted path while passing.
    if lead_prob <= CONFIRM_PROB:
      pickup = self._pickup(tracks)
      if pickup is not None and (not lead.get('present', False) or pickup.dRel < lead['dRel'] - PICKUP_CLOSER_MARGIN):
        lead = pickup.get_RadarState(lead_prob)
        self.last_action = 'pickup'
        self.last_pickup_id = int(getattr(pickup, 'identifier', -1))
    return lead
