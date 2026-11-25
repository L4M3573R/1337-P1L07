from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.ldw import CAMERA_OFFSET, LaneDepartureWarning, mount_offset

MOUNT = -0.23  # this comma: lens 0.23 m left of the car center


def model(left_y, right_y, desire=log.Desire.laneChangeLeft):
  pred = [0.0] * 8
  pred[desire] = 0.5
  lines = [SimpleNamespace(y=[y]) for y in (-5.0, left_y, right_y, 5.0)]
  return SimpleNamespace(meta=SimpleNamespace(desirePrediction=pred), laneLineProbs=[0.0, 0.9, 0.9, 0.0], laneLines=lines)


def run(offset, left_y, right_y, desire=log.Desire.laneChangeLeft):
  ldw = LaneDepartureWarning(offset)
  cs = SimpleNamespace(leftBlinker=False, rightBlinker=False, vEgo=18.0)
  ldw.update(1000, model(left_y, right_y, desire), cs, SimpleNamespace(latActive=False))
  return ldw


def test_left_lens_no_warning_with_room_to_the_line():
  # 2026-11-25 drive: left line 1.05 m from the lens = 1.28 m from the car center, ~0.37 m from the car's side.
  assert not run(MOUNT, -1.05, 1.95).left
  assert run(CAMERA_OFFSET, -1.05, 1.95).left  # the stock offset beeped here (28 of the drive's 31 warnings)


def test_left_lens_still_warns_at_the_line():
  assert run(MOUNT, -0.58, 2.70).left


def test_left_lens_right_side_warns_in_time():
  # right line 1.25 m from the lens = 1.02 m from the car center: the stock offset waited until 1.04 m from the lens
  assert run(MOUNT, -2.0, 1.25, log.Desire.laneChangeRight).right
  assert not run(CAMERA_OFFSET, -2.0, 1.25, log.Desire.laneChangeRight).right


def test_default_is_stock():
  assert LaneDepartureWarning().camera_offset == CAMERA_OFFSET


class _Params:
  def __init__(self, path):
    self.path = path

  def get_param_path(self, key):
    assert key == 'LaneCenterOffset'
    return str(self.path)


def test_mount_offset_reads_the_param(tmp_path):
  f = tmp_path / 'LaneCenterOffset'
  f.write_bytes(b'-0.23')
  assert mount_offset(_Params(f)) == -0.23


def test_mount_offset_falls_back_to_stock(tmp_path):
  f = tmp_path / 'LaneCenterOffset'
  assert mount_offset(_Params(f)) == CAMERA_OFFSET  # missing
  for bad in (b'', b'abc', b'nan', b'0.9'):
    f.write_bytes(bad)
    assert mount_offset(_Params(f)) == CAMERA_OFFSET
