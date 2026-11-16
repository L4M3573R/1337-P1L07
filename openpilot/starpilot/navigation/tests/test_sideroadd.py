"""The blinker-turn road search runs in sideroadd, never inside modeld: modeld's request and the answer round-trip
through two small files."""
import json

from openpilot.starpilot.navigation import blinker_turns as bt
from openpilot.starpilot.navigation import sideroadd
from openpilot.starpilot.navigation.side_road import SideRoad


class FakeFinder:
  def __init__(self, road=None, known=True, fail=False):
    self.road, self.known, self.fail, self.calls = road, known, fail, []

  def lookup_now(self, t, latitude, longitude, bearing, side):
    self.calls.append((t, latitude, longitude, bearing, side))
    if self.fail:
      raise ValueError('bad tile')
    return bt.Lookup(t, side, self.road, self.known)


def test_request_answer_and_read_back(tmp_path):
  req, res = tmp_path / 'req.json', tmp_path / 'res.json'
  now = [100.0]
  modeld = bt.MapTurnFinder(clock=lambda: now[0], remote=True, request_path=req, result_path=res)
  modeld.request(34.0, -117.0, 90.0, 'left')
  assert json.loads(req.read_text())['side'] == 'left' and modeld._thread is None  # no search in modeld
  road = SideRoad(42.0, 80.0, 'street')
  worker = FakeFinder(road)
  last = sideroadd.step(worker, 0.0, req, res)
  assert last == 100.0 and worker.calls == [(100.0, 34.0, -117.0, 90.0, 'left')]
  assert sideroadd.step(worker, last, req, res) == last and len(worker.calls) == 1  # an answered request is not redone
  found = modeld.latest('left')
  assert found == bt.Lookup(100.0, 'left', road, True)
  assert modeld.latest('right') is None   # a lookup for the other side does not count
  now[0] += bt.LOOKUP_MAX_AGE_S + 1
  assert modeld.latest('left') is None     # stale


def test_requests_are_rate_limited_and_failures_answered_safely(tmp_path):
  req, res = tmp_path / 'req.json', tmp_path / 'res.json'
  now = [5.0]
  modeld = bt.MapTurnFinder(clock=lambda: now[0], remote=True, request_path=req, result_path=res)
  modeld.request(1.0, 2.0, 3.0, 'right')
  modeld.request(9.0, 9.0, 9.0, 'right')  # within LOOKUP_PERIOD_S: dropped
  assert json.loads(req.read_text())['latitude'] == 1.0
  sideroadd.step(FakeFinder(fail=True), 0.0, req, res)
  found = modeld.latest('right')
  assert found is not None and found.road is None and not found.map_known


def test_missing_or_corrupt_files_mean_no_lookup(tmp_path):
  modeld = bt.MapTurnFinder(remote=True, request_path=tmp_path / 'req.json', result_path=tmp_path / 'res.json')
  assert modeld.latest('left') is None
  (tmp_path / 'res.json').write_text('{not json')
  modeld._last_read = -1e9
  assert modeld.latest('left') is None
  assert sideroadd.step(FakeFinder(), 0.0, tmp_path / 'req.json', tmp_path / 'res.json') == 0.0
