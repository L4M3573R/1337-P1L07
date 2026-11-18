"""The 2024 Elantra Hybrid's speed is on the instrument cluster's scale, so a set speed of 65 reads 65 on the dash."""
import pytest

from opendbc.car.hyundai.tests.test_elantra_hev_long_aol import params
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.values import CAR


def test_only_this_car_is_scaled():
  assert params().wheelSpeedFactor == pytest.approx(1.0223, abs=1e-6)          # stored as a 32-bit float
  assert params(lda=False, alpha_long=False).wheelSpeedFactor == pytest.approx(1.0223, abs=1e-6)
  petrol = CarInterface.get_params(CAR.HYUNDAI_ELANTRA_2024, gen_empty_fingerprint(), [], False, False, False)
  assert petrol.wheelSpeedFactor == 1.0     # measured on the hybrid only
