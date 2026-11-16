from pathlib import Path

import pytest

from openpilot.common.hardware.usb import CHESTNUT_USB_PRODUCT
from openpilot.selfdrive.modeld import helpers


def _device(root: Path, name: str, vendor: str, product_id: str, product: str, bus: int, dev: int) -> None:
  d = root / name
  d.mkdir()
  for key, value in (("idVendor", vendor), ("idProduct", product_id), ("product", product), ("busnum", bus), ("devnum", dev)):
    (d / key).write_text(f"{value}\n")


@pytest.fixture
def sysfs(tmp_path, monkeypatch):
  monkeypatch.setattr(helpers, "USB_DEVICES_PATH", tmp_path)
  _device(tmp_path, "1-1", "2c7c", "6007", "EG916Q-GL", 1, 2)
  return tmp_path


def test_node_of_the_chestnut(sysfs):
  assert helpers.chestnut_node() is None and not helpers.chestnut_present()
  _device(sysfs, "4-1", "3801", "0001", CHESTNUT_USB_PRODUCT, 4, 2)
  assert helpers.chestnut_node() == Path("/dev/bus/usb/004/002")
  assert helpers.chestnut_present()


@pytest.fixture
def chmods(monkeypatch):
  calls = []
  monkeypatch.setattr(helpers.subprocess, "run", lambda cmd, **kw: calls.append(cmd))
  return calls


def test_accessible_node_is_left_alone(sysfs, monkeypatch, chmods):
  _device(sysfs, "4-1", "3801", "0001", CHESTNUT_USB_PRODUCT, 4, 2)
  monkeypatch.setattr(helpers.os, "access", lambda path, mode: True)
  helpers.wait_for_chestnut(timeout=10.)
  assert chmods == []


def test_root_only_node_is_opened_once(sysfs, monkeypatch, chmods):
  _device(sysfs, "4-1", "3801", "0001", CHESTNUT_USB_PRODUCT, 4, 2)
  checks = []
  monkeypatch.setattr(helpers.os, "access", lambda path, mode: checks.append(path) or len(checks) >= 4)
  monkeypatch.setattr(helpers.time, "sleep", lambda s: None)
  helpers.wait_for_chestnut(timeout=10.)
  assert checks == [Path("/dev/bus/usb/004/002")] * 4
  assert chmods == [["sudo", "-n", "chmod", "664", "/dev/bus/usb/004/002"]]


def test_wait_names_an_inaccessible_node(sysfs, monkeypatch, chmods):
  _device(sysfs, "4-1", "3801", "0001", CHESTNUT_USB_PRODUCT, 4, 2)
  monkeypatch.setattr(helpers.os, "access", lambda path, mode: False)
  with pytest.raises(TimeoutError, match="/dev/bus/usb/004/002 not accessible"):
    helpers.wait_for_chestnut(timeout=0.15)
  assert len(chmods) == 1


def test_wait_without_a_chestnut(sysfs, chmods):
  with pytest.raises(TimeoutError, match="did not enumerate"):
    helpers.wait_for_chestnut(timeout=0.15)
  assert chmods == []
