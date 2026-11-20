import os
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from openpilot.common.params import Params
from openpilot.selfdrive.ui import ui_state as ui_state_module
from openpilot.selfdrive.ui.ui_state import SettledParams, UIState


@pytest.fixture
def settled(monkeypatch):
  # Files written by a test are seconds old at most; let them count as settled unless a test says otherwise.
  monkeypatch.setattr(ui_state_module, "RECENT_CHANGE_NS", -10**18)
  params = Params()
  params.put_bool("IsMetric", False, block=True)
  params.remove("NavDestination")
  reader = SettledParams(params)
  reader.params = Mock(wraps=params)
  return reader


def test_unchanged_key_is_not_reread(settled):
  for _ in range(3):
    assert settled.get_bool("IsMetric") is False
  assert settled.params.get_bool.call_count == 1


def test_written_and_removed_keys_are_reread(settled):
  assert settled.get_bool("IsMetric") is False
  settled.params.put_bool("IsMetric", True, block=True)
  assert settled.get_bool("IsMetric") is True
  settled.params.remove("IsMetric")
  assert settled.get_bool("IsMetric") is False
  assert settled.params.get_bool.call_count == 3


def test_missing_key_stays_cached_until_it_appears(settled):
  assert settled.get("NavDestination") is None
  assert settled.get("NavDestination") is None
  assert settled.params.get.call_count == 1
  settled.params.put("NavDestination", {"latitude": 1.0, "longitude": 2.0}, block=True)
  assert settled.get("NavDestination") == {"latitude": 1.0, "longitude": 2.0}


def test_other_keys_changing_do_not_force_a_reread(settled):
  assert settled.get_bool("IsMetric") is False
  for i in range(5):
    settled.params.put_bool("NetworkMetered", bool(i % 2), block=True)  # hardwared writes this twice a second
  assert settled.get_bool("IsMetric") is False
  assert settled.params.get_bool.call_count == 1


def test_recently_changed_key_is_always_reread(settled, monkeypatch):
  monkeypatch.setattr(ui_state_module, "RECENT_CHANGE_NS", 10**18)
  settled.get_bool("IsMetric")
  settled.get_bool("IsMetric")
  assert settled.params.get_bool.call_count == 2


def test_get_with_and_without_default_are_cached_separately(settled):
  settled.params.remove("InitialCruiseSpeedKph")
  plain = settled.get("InitialCruiseSpeedKph")
  with_default = settled.get("InitialCruiseSpeedKph", return_default=True)
  assert plain is None
  assert with_default == settled.params.get("InitialCruiseSpeedKph", return_default=True)


def test_in_place_rewrite_is_detected(settled):
  # Params replaces files (Python puts are asynchronous unless block=True); an in-place edit still moves ctime.
  assert settled.get_bool("IsMetric") is False
  path = settled.params.get_param_path("IsMetric")
  before = os.stat(path)
  with open(path, "wb") as f:
    f.write(b"1")
  os.utime(path, ns=(before.st_atime_ns, before.st_mtime_ns))
  assert settled.get_bool("IsMetric") is True


@pytest.fixture
def state(monkeypatch):
  monkeypatch.setattr(ui_state_module, "RECENT_CHANGE_NS", -10**18)
  monkeypatch.setattr(ui_state_module, "cable_connected", lambda: False)
  view = object.__new__(UIState)
  view.params = Params()
  view.params.put_bool("RecordAudio", True, block=True)
  view.replay_clock = None
  view.started = False
  view.chestnut_output_seen = False
  view._gpu_artifacts_at = None
  view._gpu_artifacts = SimpleNamespace(selected_gpu_status=lambda: (False, False))
  view.usb_connected = False
  view.usb_connected_ts = None
  view.usb_disconnected_ts = None
  view.usb_unknown = False
  return view


def test_update_params_keeps_live_values_current(state):
  state.update_params()
  assert state.recording_audio is False
  state.started = True
  state.update_params()
  assert state.recording_audio is True
  state.params.put_bool("RecordAudio", False, block=True)
  state.update_params()
  assert state.recording_audio is False


def test_car_params_parsed_once_while_unchanged(state, monkeypatch):
  parses = []
  real = ui_state_module.messaging.log_from_bytes
  monkeypatch.setattr(ui_state_module.messaging, "log_from_bytes", lambda *a, **k: parses.append(1) or real(*a, **k))
  monkeypatch.setattr(ui_state_module, "get_cache", lambda params, key: None)
  for _ in range(3):
    state.update_params()
  assert state.CP is None and state.has_longitudinal_control is False
  assert parses == []
