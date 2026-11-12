import json

import pytest

from openpilot.common.params import Params
from openpilot.starpilot.ui import l4_custom as s


def test_nnlc_override_defaults_to_auto_and_round_trips(tmp_path):
  assert s.nnlc_override(tmp_path) == 'auto'
  for mode in ('off', 'on', 'auto'):
    s.set_nnlc_override(mode, tmp_path)
    assert s.nnlc_override(tmp_path) == mode
  (tmp_path / 'nnlc' / 'user.json').write_text('{"version": 1, "mode": "sideways"}')
  assert s.nnlc_override(tmp_path) == 'auto'
  with pytest.raises(ValueError):
    s.set_nnlc_override('sideways', tmp_path)


def test_curve_guard_is_off_until_turned_on(tmp_path):
  assert not s.curve_guard_enabled(tmp_path)
  s.set_curve_guard(True, tmp_path)
  assert s.curve_guard_enabled(tmp_path) and json.loads((tmp_path / 'curve_guard.json').read_text()) == {'enabled': True}
  s.set_curve_guard(False, tmp_path)
  assert not s.curve_guard_enabled(tmp_path)
  (tmp_path / 'curve_guard.json').write_text('not json')
  assert not s.curve_guard_enabled(tmp_path)


def test_pulse_glide_band_and_units(tmp_path):
  p = Params(str(tmp_path))
  assert s.pg_delta(p) == 5.0 and s.pg_delta_options(p) == s.PG_DELTAS_MPH
  s.set_pg_delta(p, 3.0)
  assert s.pg_delta(p) == 3.0
  with pytest.raises(ValueError):
    s.set_pg_delta(p, 0.0)
  p.put_bool('IsMetric', True, block=True)
  assert s.pg_delta_options(p) == s.PG_DELTAS_KPH


def test_lane_offset_options_keep_an_odd_current_value(tmp_path):
  p = Params(str(tmp_path))
  assert s.lane_offset(p) == 0.0 and s.lane_offset_options(p) == (-0.23, -0.12, 0.0)
  s.set_lane_offset(p, -0.05)
  assert s.lane_offset(p) == pytest.approx(-0.05) and -0.05 in s.lane_offset_options(p)
  with pytest.raises(ValueError):
    s.set_lane_offset(p, 0.5)

