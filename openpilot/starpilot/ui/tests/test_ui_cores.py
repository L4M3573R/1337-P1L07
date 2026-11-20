import json

from openpilot.starpilot.ui.ui_cores import READ_PERIOD_S, UiCoreSwitch, parse_cores

STOCK = frozenset({0, 1, 2, 7})


def test_parse_accepts_only_distinct_in_range_core_lists():
  assert parse_cores({"ui_cores": [0, 1, 2, 6]}, 8) == frozenset({0, 1, 2, 6})
  assert parse_cores({"ui_cores": [4, 5, 6]}, 8) == frozenset({4, 5, 6})
  for bad in ([], [8], [-1], [1, 1], [True], [1.0], "0,1", None):
    assert parse_cores({"ui_cores": bad}, 8) is None, bad
  assert parse_cores({"mode": "off"}, 8) is None
  assert parse_cores(["ui_cores"], 8) is None


def test_missing_file_or_key_keeps_stock(tmp_path):
  path = tmp_path / "cpu_boost.json"
  switch = UiCoreSwitch(STOCK, path=path, cpu_count=8)
  assert switch.current(0.0) == STOCK
  path.write_text(json.dumps({"mode": "on", "big_mhz": 2200}))
  assert switch.current(READ_PERIOD_S + 1) == STOCK
  path.write_text("{torn")
  assert switch.current(2 * READ_PERIOD_S + 2) == STOCK


def test_switch_follows_the_file_on_its_read_period(tmp_path):
  path = tmp_path / "cpu_boost.json"
  events = []
  switch = UiCoreSwitch(STOCK, path=path, cpu_count=8, log=lambda name, **kw: events.append((name, kw)))
  assert switch.current(0.0) == STOCK
  path.write_text(json.dumps({"mode": "off", "ui_cores": [0, 1, 2, 6]}))
  assert switch.current(1.0) == STOCK  # not reread yet
  assert switch.current(READ_PERIOD_S + 0.1) == frozenset({0, 1, 2, 6})
  path.unlink()
  assert switch.current(2 * READ_PERIOD_S + 0.2) == STOCK
  assert events == [("ui_cores", {"cores": [0, 1, 2, 6], "stock": False}), ("ui_cores", {"cores": [0, 1, 2, 7], "stock": True})]


def test_clock_moving_back_rereads(tmp_path):
  path = tmp_path / "cpu_boost.json"
  switch = UiCoreSwitch(STOCK, path=path, cpu_count=8)
  switch.current(100.0)
  path.write_text(json.dumps({"ui_cores": [6]}))
  assert switch.current(50.0) == frozenset({6})
