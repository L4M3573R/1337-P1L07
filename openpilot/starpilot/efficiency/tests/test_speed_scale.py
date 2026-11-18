"""Speed on the cluster's scale (2026-11-18): drives recorded on the bare wheel average are converted when read."""
import json
import re
from pathlib import Path

import pytest

from openpilot.starpilot.efficiency import experiment_learn as el
from openpilot.starpilot.efficiency import store

real = pytest.mark.real_speed_scale
K = 1.0223


def doc(factor=None):
  d = {'version': store.VERSION, 'drive': '20261117-120000-1',
       'total': {'distance': 1000.0, 'fuel_distance': 900.0, 'obd_distance': 800.0, 'fuel_gal': 0.02, 'propulsion': 5e5},
       'contexts': {'60+ mph|flat|straight|open road|chill|eco': {'distance': 400.0, 'obd_distance': 300.0, 'obd_fuel_gal': 0.01}},
       'stretches': [{'m': 290.0, 'dur': 10.0, 'v0': 29.0, 'v1': 29.2, 'obd_gal': 0.004, 'prop_j': 1e5}]}
  if factor is not None:
    d['speed_factor'] = factor
  return d


@real
def test_the_ledger_and_the_car_use_the_same_factor():
  src = (Path(__file__).resolve().parents[4] / 'opendbc_repo/opendbc/car/hyundai/interface.py').read_text()
  assert float(re.search(r'ret\.wheelSpeedFactor = ([0-9.]+)', src).group(1)) == store.SPEED_FACTOR == K


@real
def test_an_old_drive_is_brought_onto_todays_scale_once():
  d = store.rescale(doc())
  assert d['total']['distance'] == pytest.approx(1000.0 * K) and d['total']['obd_distance'] == pytest.approx(800.0 * K)
  assert d['total']['fuel_gal'] == 0.02 and d['total']['propulsion'] == 5e5          # fuel and energy as recorded
  assert d['contexts']['60+ mph|flat|straight|open road|chill|eco']['distance'] == pytest.approx(400.0 * K)
  st = d['stretches'][0]
  assert st['m'] == pytest.approx(290.0 * K) and st['v0'] == pytest.approx(29.0 * K) and st['dur'] == 10.0 and st['obd_gal'] == 0.004
  assert d['speed_factor'] == K
  again = json.loads(json.dumps(d))
  assert store.rescale(again) == d                                                      # already on today's scale


@real
def test_a_drive_recorded_on_todays_scale_is_left_alone_and_nonsense_factors_count_as_old():
  assert store.rescale(doc(K)) == doc(K)
  for bad in (0.0, 5.0, True, 'x', None):
    d = doc()
    d['speed_factor'] = bad
    assert store.rescale(d)['total']['distance'] == pytest.approx(1000.0 * K)
  assert store.rescale(None) is None and store.rescale({}) == {'speed_factor': K}


@real
def test_new_drives_record_their_factor_and_every_reader_converts_old_ones(tmp_path):
  from openpilot.starpilot.efficiency.ledger import Ledger
  assert store.drive_document('d', [], speed_factor=K)['speed_factor'] == K and store.drive_document('d', [])['speed_factor'] == 1.0
  assert Ledger() is not None
  (tmp_path / 'drives').mkdir()
  (tmp_path / 'drives' / 'old.json').write_text(json.dumps(doc()))
  (tmp_path / 'drives' / 'new.json').write_text(json.dumps(doc(K)))
  ms = sorted(st['m'] for d in el.load_drives(tmp_path) for st in d)
  assert ms == pytest.approx([290.0, 290.0 * K])
  total, _, n = store.load_totals(tmp_path)
  assert n == 2 and total.distance == pytest.approx(1000.0 + 1000.0 * K)


def test_the_other_fuel_tests_run_on_a_scale_of_one():
  assert store.SPEED_FACTOR == 1.0 and store.rescale(doc())['total']['distance'] == 1000.0
