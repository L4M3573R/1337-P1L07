import numpy as np

from openpilot.common.transformations.camera import get_view_frame_from_road_frame
from openpilot.starpilot.stop_line.lines import clusters, view_from_road


def test_projection_matches_openpilot_camera_code():
  for rpy in ((0.0, 0.0, 0.0), (-0.0004, -0.0273, 0.0387), (0.01, 0.03, -0.05)):
    assert np.allclose(view_from_road(*rpy, 1.29), get_view_frame_from_road_frame(*rpy, 1.29))


def test_a_line_seen_from_many_frames_is_one_cluster_and_noise_is_dropped():
  line = [(i, 120.0 + 0.1 * np.sin(i)) for i in range(8)]
  noise = [(20, 140.0), (21, 150.0)]
  out = clusters(line + noise)
  assert len(out) == 1 and abs(out[0]['odometer'] - 120.0) < 0.2 and out[0]['support'] == 8
