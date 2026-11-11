"""On-device ledger storage and the efficiencyd recorder."""
import json
import os
from types import SimpleNamespace as NS

import pytest

from openpilot.starpilot.efficiency import store
from openpilot.starpilot.efficiency.efficiencyd import Recorder
from openpilot.starpilot.efficiency.ledger import Ledger, Sample
from openpilot.starpilot.efficiency.report import stored_report


def stretches(km=1.0, speed=20.0):
  ledger, t = Ledger(), 0.0
  for _ in range(int(km * 1000 / speed / 0.05)):
    ledger.add(Sample(t=t, speed=speed, accel=0.0, pitch=0.0, mass=1500.0))
    t += 0.05
  return ledger.finish()


def test_drives_round_trip_and_sum(tmp_path):
  store.save_drive("a", stretches(1.0), tmp_path)
  store.save_drive("b", stretches(2.0), tmp_path)
  total, contexts, drives = store.load_totals(tmp_path)
  assert drives == 2
  assert total.distance == pytest.approx(3000.0, rel=0.01)
  assert sum(t.distance for t in contexts.values()) == pytest.approx(total.distance)
  assert "2 drives recorded" in stored_report(tmp_path)


def test_checkpoint_replaces_the_same_drive_file_atomically(tmp_path):
  store.save_drive("drive", stretches(1.0), tmp_path)
  store.save_drive("drive", stretches(2.0), tmp_path)
  files = list((tmp_path / "drives").iterdir())
  assert [f.name for f in files] == ["drive.json"]  # no leftover temporary files
  assert store.load_totals(tmp_path)[0].distance == pytest.approx(2000.0, rel=0.01)


def test_corrupt_or_foreign_files_are_skipped(tmp_path):
  store.save_drive("good", stretches(1.0), tmp_path)
  (tmp_path / "drives" / "bad.json").write_text("{not json")
  (tmp_path / "drives" / "old.json").write_text(json.dumps({"version": 0}))
  assert store.load_totals(tmp_path)[2] == 1


def test_old_drives_are_pruned(tmp_path):
  for i in range(5):
    store.save_drive(f"d{i}", stretches(0.3), tmp_path)
    os.utime(tmp_path / "drives" / f"d{i}.json", (i, i))
  store.prune(tmp_path, keep=2)
  assert sorted(f.name for f in (tmp_path / "drives").iterdir()) == ["d3.json", "d4.json"]


@pytest.mark.parametrize("bad", ["", "../x", "a/b", ".hidden"])
def test_drive_ids_cannot_escape_the_store(tmp_path, bad):
  with pytest.raises(ValueError):
    store.save_drive(bad, stretches(0.3), tmp_path)


def car_state(speed=20.0):
  return NS(vEgo=speed, aEgo=0.0, standstill=False)


def test_recorder_ignores_repeated_samples_and_saves_on_finish(tmp_path):
  rec = Recorder(1500.0, "drive", tmp_path, checkpoint_s=60.0)
  t = 0.0
  for _ in range(800):  # 40 s at 20 Hz, each carState seen twice (radarState and carState cadences)
    rec.observe(t, car_state())
    rec.observe(t, car_state())
    t += 0.05
  assert len(rec.ledger.stretches) >= 2  # duplicates did not split the drive into gaps
  assert not rec.maybe_checkpoint(0.0)
  assert rec.maybe_checkpoint(61.0)
  rec.finish()
  total = store.load_totals(tmp_path)[0]
  assert total.distance == pytest.approx(20.0 * 40, rel=0.02)


def test_every_subscribed_service_exists_in_this_fork():
  from openpilot.cereal.services import SERVICE_LIST
  from openpilot.starpilot.efficiency.efficiencyd import SERVICES
  assert set(SERVICES) <= set(SERVICE_LIST)  # an unknown name crashes SubMaster at startup


def test_yaw_rate_from_device_motion_sets_curvature():
  from openpilot.starpilot.efficiency.ledger import make_sample
  motion = NS(angularVelocityDevice=NS(z=0.2))
  assert make_sample(0.0, 1500.0, car_state(20.0), device_motion=motion).curvature == pytest.approx(0.01)
