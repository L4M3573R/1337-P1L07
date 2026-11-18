"""The fuel tests' drive files carry no speed factor, like every drive recorded before 2026-11-18. Read through
store.rescale they would all be stretched onto the cluster's speed scale, which is not what a test of the arithmetic is
about: here the scale is 1.0 unless a test asks for the real one (test_speed_scale.py)."""
import pytest

from openpilot.starpilot.efficiency import store


@pytest.fixture(autouse=True)
def bare_wheel_speed_scale(request, monkeypatch):
  if 'real_speed_scale' not in request.keywords:
    monkeypatch.setattr(store, 'SPEED_FACTOR', 1.0)


def pytest_configure(config):
  config.addinivalue_line('markers', "real_speed_scale: run with the car's real wheel-speed factor instead of 1.0")
