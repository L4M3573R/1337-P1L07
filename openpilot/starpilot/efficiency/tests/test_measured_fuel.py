"""Measured (BlueDriver) fuel in the ledger: complete stretches only, no bridging over link drops or gaps."""
import pytest

from openpilot.starpilot.efficiency.efficiencyd import Recorder
from openpilot.starpilot.efficiency.ledger import BATTERY_KWH, GAL_PER_BATTERY_KWH, Ledger, Sample, summarize

DT = 0.05
SPEED = 20.0


def run(fuel_at, soc_at=lambda t: None, seconds=120.0):
  ledger, t = Ledger(), 0.0
  while t < seconds:
    ledger.add(Sample(t=t, speed=SPEED, accel=0.0, pitch=0.0, mass=1500.0, long_active=True,
                      obd_fuel_gal=fuel_at(t), soc=soc_at(t)))
    t += DT
  return summarize(ledger.finish())[0]


def test_steady_burn_gives_the_expected_mpg():
  rate = 0.0004  # gal/s: 20 m/s is 44.7 mph, so 44.7 mph / 1.44 gal/h = 31.1 mpg
  total = run(lambda t: rate * t)
  assert total.obd_distance == pytest.approx(total.distance, rel=0.02)
  assert total.obd_mpg == pytest.approx(SPEED / 1609.344 / rate, rel=0.02)


def test_link_drop_excludes_only_the_affected_stretches():
  rate = 0.0004
  total = run(lambda t: None if 40.0 <= t < 50.0 else rate * t)
  assert 0 < total.obd_distance < total.distance
  assert total.obd_mpg == pytest.approx(SPEED / 1609.344 / rate, rel=0.02)  # no fuel lost or invented


def test_battery_drain_is_paid_back_in_fuel():
  rate = 0.0004
  total = run(lambda t: rate * t, soc_at=lambda t: 60.0 - t / 12.0)  # 10 points drained over 120 s
  assert total.obd_soc_delta == pytest.approx(-10.0, abs=0.2)
  fuel = total.obd_fuel_gal + 10.0 / 100 * BATTERY_KWH * GAL_PER_BATTERY_KWH
  assert total.obd_mpg_battery_adjusted == pytest.approx(total.obd_distance / 1609.344 / fuel, rel=0.02)
  assert total.obd_mpg_battery_adjusted < total.obd_mpg


def test_recorder_counter_survives_logger_restart_and_link_drop():
  r = Recorder(1500.0, 'd', root=None)
  r.observe_obd({'drive': 'a', 'connected': True, 'fuel_g': 10.0, 'soc': 55.0})
  r.observe_obd({'drive': 'a', 'connected': True, 'fuel_g': 38.4})
  assert r.obd_fuel_gal == pytest.approx(28.4 / 2840.0)
  r.observe_obd({'drive': 'a', 'connected': False, 'fuel_g': 38.4})
  assert r.obd_fuel_gal is None
  r.observe_obd({'drive': 'b', 'connected': True, 'fuel_g': 0.0})  # logger restarted: no jump
  r.observe_obd({'drive': 'b', 'connected': True, 'fuel_g': 28.4})
  assert r.obd_fuel_gal == pytest.approx(56.8 / 2840.0)
  r.observe_obd(None)
  assert r.obd_fuel_gal is None and r.soc is None
