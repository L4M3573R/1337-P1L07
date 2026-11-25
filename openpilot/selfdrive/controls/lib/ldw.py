import math
from pathlib import Path

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.common.constants import CV


# Lens position right of the car center (model y is right-positive), so a line within LANE_CLOSE_M of the car
# center counts as close on either side. Stock assumes 0.04; LaneCenterOffset holds the measured mount instead.
CAMERA_OFFSET = 0.04
LANE_CLOSE_M = 1.08
MAX_MOUNT_OFFSET_M = 0.3
LDW_MIN_SPEED = 31 * CV.MPH_TO_MS
LANE_DEPARTURE_THRESHOLD = 0.1


def mount_offset(params) -> float:
  """The lens offset from LaneCenterOffset (-0.23 = lens 0.23 m left of the car center), else stock. With the stock
  0.04 on a lens mounted 0.23 m left, the left warning beeped with 0.4 m still to the line (2026-11-25)."""
  try:
    value = float(Path(params.get_param_path('LaneCenterOffset')).read_bytes())
  except (OSError, ValueError, TypeError):
    return CAMERA_OFFSET
  return value if math.isfinite(value) and abs(value) <= MAX_MOUNT_OFFSET_M else CAMERA_OFFSET


class LaneDepartureWarning:
  def __init__(self, camera_offset: float = CAMERA_OFFSET):
    self.camera_offset = camera_offset
    self.left = False
    self.right = False
    self.last_blinker_frame = 0

  def update(self, frame, modelV2, CS, CC):
    if CS.leftBlinker or CS.rightBlinker:
      self.last_blinker_frame = frame

    recent_blinker = (frame - self.last_blinker_frame) * DT_MDL < 5.0  # 5s blinker cooldown
    ldw_allowed = CS.vEgo > LDW_MIN_SPEED and not recent_blinker and not CC.latActive

    desire_prediction = modelV2.meta.desirePrediction
    if len(desire_prediction) and ldw_allowed:
      right_lane_visible = modelV2.laneLineProbs[2] > 0.5
      left_lane_visible = modelV2.laneLineProbs[1] > 0.5
      l_lane_change_prob = desire_prediction[log.Desire.laneChangeLeft]
      r_lane_change_prob = desire_prediction[log.Desire.laneChangeRight]

      lane_lines = modelV2.laneLines
      l_lane_close = left_lane_visible and (lane_lines[1].y[0] > -(LANE_CLOSE_M + self.camera_offset))
      r_lane_close = right_lane_visible and (lane_lines[2].y[0] < (LANE_CLOSE_M - self.camera_offset))

      self.left = bool(l_lane_change_prob > LANE_DEPARTURE_THRESHOLD and l_lane_close)
      self.right = bool(r_lane_change_prob > LANE_DEPARTURE_THRESHOLD and r_lane_close)
    else:
      self.left, self.right = False, False

  @property
  def warning(self) -> bool:
    return bool(self.left or self.right)
