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
* Turn veto: that low-speed override takes the closest stationary return straight ahead of the car's
  heading, so at a turn it picks up whatever is across the intersection (2026-11-15: braked from
  6.8 to 4.8 mph and dragged a left turn from a stop sign to ~1 mph, model probability 0). Such a lead
  is dropped when the predicted path does not go through it, or when a signalled turn's mapped side road
  (turn_speed status) is reached before it. Camera-confirmed and held leads are never vetoed.
"""
from __future__ import annotations

import math
import time
from collections.abc import Callable
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
PICKUP_KEEP_JUMP_M = 5.0      # m per model frame: a picked-up track that jumps further is another object in its slot
VETO_MAX_SPEED = 4.0          # m/s, radard's V_EGO_STATIONARY: below it stationary returns become leads without the camera
STATIONARY_SPEED = 1.0        # m/s ground speed under which a return counts as stationary
TURN_POINT_MARGIN = 2.0       # m, a return this close to the side road (or past it) is beyond where the car turns off


def _turn_status() -> dict | None:
  from openpilot.starpilot.navigation.turn_speed import read_status
  return read_status(time.monotonic())


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
  def __init__(self, turn_status: Callable[[], dict | None] = _turn_status) -> None:
    self.in_path_frames: dict[int, int] = {}
    self.held = _HeldLead()
    self.turn_status = turn_status
    # Decision log for remote review: "camera", "hold", "pickup", "veto_off_path", "veto_past_turn" or "none".
    self.last_action = 'none'
    self.last_pickup_id = -1
    self.last_lead_prob = 0.0
    self.picked_id = -1       # the track that was the pickup lead last frame
    self.picked_drel = 0.0

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
    # The speed gate is for taking a track up. The car already being followed stays the lead as it slows to a stop
    # (review 2026-11-17: it was dropped at 3 m/s, with the camera unsure, while we could still be doing 20 mph).
    def fast_enough(track_id, track) -> bool:
      return track.vLead >= PICKUP_MIN_SPEED or (track_id == self.picked_id and
                                                 abs(track.dRel - self.picked_drel) < PICKUP_KEEP_JUMP_M)
    candidates = [(track.dRel, track_id) for track_id, track in tracks.items()
                  if self.in_path_frames.get(track_id, 0) >= PICKUP_MIN_FRAMES and fast_enough(track_id, track) and
                  PICKUP_MIN_DISTANCE < track.dRel < PICKUP_MAX_DISTANCE]
    return min(candidates)[1] if candidates else None

  def _turn_veto(self, lead: dict[str, Any], tracks: dict, model, lead_prob: float, v_ego: float) -> str | None:
    """Why radard's low-speed override lead is not in the car's way, or None to keep it."""
    if (not lead.get('present', False) or not lead.get('radar', False) or lead_prob > CONFIRM_PROB or
        not v_ego < VETO_MAX_SPEED or not abs(float(lead.get('vLead', 0.0))) < STATIONARY_SPEED):
      return None
    track = tracks.get(lead.get('radarTrackId'))
    if track is None:
      return None
    if path_offset(track, model) >= PATH_HALF_WIDTH:
      return 'veto_off_path'
    turn = self.turn_status()
    distance = turn.get('distance_m') if isinstance(turn, dict) and not turn.get('turning') else None
    if isinstance(distance, (int, float)) and math.isfinite(distance) and track.dRel >= distance - TURN_POINT_MARGIN:
      return 'veto_past_turn'
    return None

  def lead_one(self, base: dict[str, Any], tracks: dict, model, lead_prob: float, v_ego: float | None = None) -> dict[str, Any]:
    """Fuse radard's camera-gated leadOne (`base`) with the radar tracks. With v_ego, a stationary low-speed
    override lead the car will not reach (see the turn veto above) is dropped."""
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
    picked = -1
    if lead_prob <= CONFIRM_PROB:
      picked_key = self._pickup(tracks)
      pickup = tracks[picked_key] if picked_key is not None else None
      if pickup is not None and (not lead.get('present', False) or pickup.dRel < lead['dRel'] - PICKUP_CLOSER_MARGIN):
        lead = pickup.get_RadarState(lead_prob)
        self.last_action = 'pickup'
        self.last_pickup_id = int(getattr(pickup, 'identifier', -1))
        picked, self.picked_drel = picked_key, float(pickup.dRel)
    self.picked_id = picked
    if self.last_action == 'none' and v_ego is not None and (veto := self._turn_veto(lead, tracks, model, lead_prob, v_ego)):
      self.last_action = veto
      return {'present': False}
    return lead
