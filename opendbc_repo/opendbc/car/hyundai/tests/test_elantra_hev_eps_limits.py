"""The 2024 Elantra Hybrid asks its EPS for more than the Hyundai default (parking-lot test 2026-11-17), but only with
the safety word the panda raises its own limits for: the car must never ask for more than the panda lets through."""
from types import SimpleNamespace as NS

from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiSafetyFlags, elantra_hev_2024_eps

WORD = 0x8C0E   # the car's own word: refresh CAN, LDA button, CCNC, camera SCC, openpilot long, hybrid gas


def cp(fingerprint, word, flags=0):
  return NS(carFingerprint=fingerprint, flags=flags, safetyConfigs=[NS(safetyParam=word)])


def limits(CP):
  p = CarControllerParams(CP)
  return p.STEER_MAX, p.STEER_DELTA_UP, p.STEER_DELTA_DOWN


def test_the_cars_own_word_gets_the_measured_limits():
  assert limits(cp(CAR.HYUNDAI_ELANTRA_HEV_2024, WORD)) == (400, 10, 9)
  assert limits(cp(CAR.HYUNDAI_ELANTRA_HEV_2024, WORD & ~int(HyundaiSafetyFlags.LONG))) == (400, 10, 9)   # stock longitudinal too


def test_anything_the_panda_would_not_raise_keeps_the_default():
  need = (HyundaiSafetyFlags.CAN_REFRESH_MSGS, HyundaiSafetyFlags.CAMERA_SCC, HyundaiSafetyFlags.HYBRID_GAS)
  for missing in need:
    assert limits(cp(CAR.HYUNDAI_ELANTRA_HEV_2024, WORD & ~int(missing))) == (384, 3, 7)
  assert limits(cp(CAR.HYUNDAI_ELANTRA_2024, WORD)) == (384, 3, 7)           # the petrol Elantra was not tested
  assert limits(cp(CAR.HYUNDAI_SONATA, WORD)) == (384, 3, 7)
  assert not elantra_hev_2024_eps(NS(carFingerprint=CAR.HYUNDAI_ELANTRA_HEV_2024, flags=0, safetyConfigs=[]))
  assert not elantra_hev_2024_eps(NS(carFingerprint=CAR.HYUNDAI_ELANTRA_HEV_2024, flags=0))


def test_the_panda_side_selects_on_the_same_three_bits():
  import re
  from pathlib import Path
  src = (Path(__file__).resolve().parents[3] / 'safety' / 'modes' / 'hyundai.h').read_text()
  assert 'HYUNDAI_STEERING_LIMITS_ELANTRA_HEV_2024 = HYUNDAI_LIMITS_RT(400, 10, 9, 375)' in src
  assert re.search(r'hyundai_can_refresh_msgs && hyundai_camera_scc && hyundai_hybrid_gas_signal\) \?\s+HYUNDAI_STEERING_LIMITS_ELANTRA_HEV_2024', src)
