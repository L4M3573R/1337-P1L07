"""Energy model and ledger: physics sanity and context bookkeeping."""
import math

import pytest

from openpilot.starpilot.efficiency.energy import REGEN_MAX_DECEL, resist_accel, step
from openpilot.starpilot.efficiency.ledger import STRETCH_M, Ledger, Sample, summarize

MASS = 1500.0
DT = 0.05


def drive(profile, pitch=0.0, **kw):
  """profile: list of (seconds, accel) segments starting from speed v0 (kw['v0'], default 20 m/s)."""
  v, t, ledger = kw.pop('v0', 20.0), 0.0, Ledger()
  for seconds, accel in profile:
    for _ in range(int(round(seconds / DT))):
      ledger.add(Sample(t=t, speed=v, accel=accel, pitch=pitch, mass=MASS, **kw))
      v = max(v + accel * DT, 0.0)
      t += DT
  return summarize(ledger.finish())


def test_steady_flat_cruise_costs_exactly_the_steady_baseline():
  total, _ = drive([(60, 0.0)])
  assert total.distance == pytest.approx(20.0 * 60, rel=0.01)
  assert total.propulsion == pytest.approx(total.steady, rel=1e-6)
  assert total.regen == total.friction == 0.0
  expected = MASS * resist_accel(20.0) * total.distance  # force x distance
  assert total.propulsion == pytest.approx(expected, rel=0.02)


def test_speed_up_and_brake_cycle_shows_excess_and_braking():
  total, _ = drive([(20, 0.0), (5, 1.0), (5, -1.0), (20, 0.0)], v0=12.0)
  assert total.excess > 0
  assert total.regen > 0 and total.friction == pytest.approx(0.0, abs=1.0)  # gentle slowdown: all within regen


def test_hard_braking_exceeds_regen_and_shows_friction():
  e = step(20.0, -(REGEN_MAX_DECEL + 3.0), 0.0, MASS, 1.0)
  assert e.friction_j > 0 and e.regen_j > 0
  assert e.propulsion_j == 0


def test_coasting_at_natural_decel_needs_no_energy():
  e = step(25.0, -resist_accel(25.0), 0.0, MASS, 1.0)
  assert e.propulsion_j == pytest.approx(0.0, abs=1e-6) and e.regen_j == pytest.approx(0.0, abs=1e-6)


def test_uphill_costs_more_and_downhill_coast_recovers():
  up = step(20.0, 0.0, math.atan(0.05), MASS, 1.0)
  flat = step(20.0, 0.0, 0.0, MASS, 1.0)
  down = step(20.0, 0.0, math.atan(-0.05), MASS, 1.0)
  assert up.propulsion_j > flat.propulsion_j
  assert down.propulsion_j == 0 and down.regen_j > 0  # holding speed downhill must brake/regen


def test_standstill_and_invalid_samples_add_nothing():
  assert step(0.0, 0.0, 0.0, MASS, 1.0) == step(float('nan'), 0.0, 0.0, MASS, 1.0)
  assert step(0.0, 0.0, 0.0, MASS, 1.0).propulsion_j == 0


def test_stretches_split_by_distance_and_mode_and_carry_context():
  total, by_context = drive([(60, 0.0)], v0=25.0, lead=True, long_active=True, experimental=True)
  ledger_stretches = sum(1 for _ in by_context)
  assert ledger_stretches == 1
  key, = by_context
  assert key == ("45-60 mph", "flat", "straight", "following", "experimental", "unknown")  # no drive mode in this test
  assert total.distance / STRETCH_M >= 4


def test_log_gap_starts_a_new_stretch():
  ledger = Ledger()
  for t in (0.0, 0.05, 0.10, 5.0, 5.05, 5.10):
    ledger.add(Sample(t=t, speed=10.0, accel=0.0, pitch=0.0, mass=MASS))
  stretches = ledger.finish()
  assert len(stretches) == 2
  assert all(s.duration <= 0.11 for s in stretches)


def test_next_drive_with_an_earlier_boot_clock_is_not_dropped(monkeypatch):
  from types import SimpleNamespace as NS
  from openpilot.starpilot.efficiency import report

  def event(which, t, **payload):
    return NS(which=lambda: which, logMonoTime=int(t * 1e9), **payload)

  def car_state(t):
    return event('carState', t, carState=NS(vEgo=20.0, aEgo=0.0, standstill=False))

  drive_a = [event('carParams', 100.0, carParams=NS(mass=1500.0))] + [car_state(100.0 + i * 0.1) for i in range(20)]
  drive_b = [event('carParams', 5.0, carParams=NS(mass=1500.0))] + [car_state(5.0 + i * 0.1) for i in range(20)]
  logs = {'a': drive_a, 'b': drive_b}
  monkeypatch.setattr('openpilot.tools.lib.logreader.LogReader', lambda name, sort_by_time=True: logs[name])
  produced = list(report.samples([__import__('pathlib').Path('a'), __import__('pathlib').Path('b')]))
  assert len(produced) == 40  # both drives' samples, despite drive b's clock starting earlier
