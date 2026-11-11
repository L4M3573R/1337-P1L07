import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.starpilot.edgemap import recorder
from openpilot.starpilot.edgemap.recorder import COLUMNS, EDGE_LOOKAHEAD_M, DriveLog, load_drive, sample_row, save_drive

X = [0.0, 10.0, 20.0, 40.0, 60.0]


def model(left=-4.0, right=4.0, left_slope=0.0):
  edge = lambda y0, slope: NS(x=X, y=[y0 + slope * x for x in X])  # noqa: E731
  return NS(roadEdges=[edge(left, left_slope), edge(right, 0.0)], roadEdgeStds=[0.5, 0.6], laneLineProbs=[0.1, 0.9, 0.8, 0.2])


def car(**kw):
  base = {"vEgo": 8.0, "steeringAngleDeg": -3.0, "leftBlinker": True, "rightBlinker": False, "standstill": False}
  return NS(**{**base, **kw})


GPS = NS(latitude=41.6, longitude=-72.88, bearingDeg=120.0, speed=8.1, horizontalAccuracy=1.4, hasFix=True)
MOTION = NS(angularVelocityDevice=NS(z=0.05))


def row_dict(row):
  assert len(row) == len(COLUMNS)
  return dict(zip(COLUMNS, row, strict=True))


def test_a_row_ties_road_edges_and_motion_to_a_fresh_gps_position():
  r = row_dict(sample_row(5.0, model(left=-4.0, left_slope=-0.1), car(), NS(latActive=True), MOTION, GPS, gps_age_s=0.1))
  assert (r["lat"], r["lon"], r["bearing"], r["gps_accuracy"]) == (41.6, -72.88, 120.0, 1.4)
  assert r["left_blinker"] == 1.0 and r["right_blinker"] == 0.0 and r["lat_active"] == 1.0
  assert r["yaw_rate"] == pytest.approx(0.05)
  for d in EDGE_LOOKAHEAD_M:
    assert r[f"left_edge_{int(d)}m"] == pytest.approx(-4.0 - 0.1 * d)  # interpolated at each lookahead
    assert r[f"right_edge_{int(d)}m"] == pytest.approx(4.0)


@pytest.mark.parametrize("gps,age", [(GPS, 2.5), (None, 0.0), (NS(**{**vars(GPS), "hasFix": False}), 0.1),
                                     (NS(**{**vars(GPS), "latitude": 0.0, "longitude": 0.0}), 0.1)])
def test_stale_missing_or_unfixed_gps_records_no_position(gps, age):
  r = row_dict(sample_row(5.0, model(), car(), None, None, gps, gps_age_s=age))
  assert all(math.isnan(r[k]) for k in ("lat", "lon", "bearing", "gps_speed", "gps_accuracy"))
  assert r["v_ego"] == 8.0  # the rest of the row is still recorded


def test_malformed_model_output_becomes_nan_not_a_crash():
  bad = NS(roadEdges=[], roadEdgeStds=[], laneLineProbs=[])
  r = row_dict(sample_row(5.0, bad, car(), None, None, GPS, 0.1))
  assert math.isnan(r["left_edge_15m"]) and math.isnan(r["right_edge_std"]) and math.isnan(r["lane_prob_0"])


def test_drive_log_decimates_to_five_hertz_and_survives_a_clock_reset():
  log = DriveLog("d")
  base = sample_row(0.0, model(), car(), None, None, GPS, 0.1)
  kept = [log.add([t, *base[1:]]) for t in (10.0, 10.05, 10.1, 10.15, 10.2, 10.45, 3.0)]
  assert kept == [True, False, False, False, True, True, True]
  frames = DriveLog("f")
  assert sum(frames.add([i * 0.05, *base[1:]]) for i in range(400)) == 100  # 20 s of 20 Hz frames -> exactly 5 Hz


def test_round_trip_keeps_full_gps_and_clock_precision(tmp_path):
  log = DriveLog("20261110-1200-1", tmp_path)
  for i in range(3):
    gps = NS(**{**vars(GPS), "latitude": 41.600001234 + i * 1e-7})
    log.add(sample_row(1_000_000.123 + i, model(), car(), None, MOTION, gps, 0.1))
  path = log.save()
  columns, rows = load_drive(path)
  assert columns == COLUMNS and rows.shape == (3, len(COLUMNS))
  assert rows[2, 1] == pytest.approx(41.600001434, abs=1e-12)  # sub-centimetre, not float32 metres
  assert rows[1, 0] == pytest.approx(1_000_001.123, abs=1e-6)


def test_checkpoints_replace_the_same_drive_file(tmp_path):
  log = DriveLog("same", tmp_path)
  log.add(sample_row(1.0, model(), car(), None, None, GPS, 0.1))
  log.save()
  log.add(sample_row(2.0, model(), car(), None, None, GPS, 0.1))
  log.save()
  files = list((tmp_path / "drives").glob("*.npz"))
  assert len(files) == 1 and load_drive(files[0])[1].shape[0] == 2


def test_nothing_recorded_writes_nothing(tmp_path):
  assert DriveLog("empty", tmp_path).save() is None
  assert not (tmp_path / "drives").exists()


@pytest.mark.parametrize("bad", ["", "../x", "a/b", ".hidden"])
def test_drive_ids_cannot_escape_the_store(tmp_path, bad):
  with pytest.raises(ValueError):
    save_drive(bad, np.zeros((1, len(COLUMNS))), tmp_path)


def test_old_drives_are_pruned_by_count_and_size(tmp_path, monkeypatch):
  rows = np.zeros((5, len(COLUMNS)))
  for i in range(5):
    save_drive(f"d{i}", rows, tmp_path)
  monkeypatch.setattr(recorder, "MAX_DRIVES", 3)
  recorder.prune(tmp_path, keep=3)
  assert len(list((tmp_path / "drives").glob("*.npz"))) == 3
  recorder.prune(tmp_path, keep=10, max_bytes=1)
  assert not list((tmp_path / "drives").glob("*.npz"))


def test_foreign_or_corrupt_files_are_skipped(tmp_path):
  (tmp_path / "junk.npz").write_bytes(b"not a zip")
  assert load_drive(tmp_path / "junk.npz") is None


def test_every_subscribed_service_exists_in_this_fork():
  from openpilot.cereal.services import SERVICE_LIST
  from openpilot.starpilot.edgemap.edgemapd import SERVICES
  assert set(SERVICES) <= set(SERVICE_LIST)  # an unknown name crashes SubMaster at startup
