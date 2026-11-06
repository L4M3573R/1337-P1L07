import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai import classic_long_aol, long_buttons, ordinary_classic_startup
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.classic_scc_aol import ClassicSccLkasSources
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR


def params(*, lda=True, alpha_long=True):
  fp = gen_empty_fingerprint()
  if lda:
    fp[0][0x391] = 8
    fp[0][0x50c] = 8
  return CarInterface.get_params(CAR.HYUNDAI_ELANTRA_HEV_2024, fp, [], alpha_long, False, False)


def mark(cp, experience=32):
  cp.safetyConfigs[0].safetyParam = classic_long_aol.aol_word(cp)
  cp.alternativeExperience = experience
  return cp


@pytest.mark.parametrize('lda,word', [(True, 0x8C0E), (False, 0x840E)])
def test_exact_camera_long_words(lda, word):
  cp = params(lda=lda)
  assert cp.safetyConfigs[0].safetyParam == classic_long_aol.ordinary_word(cp) == 0x800E
  assert classic_long_aol.aol_word(cp) == word
  assert word in classic_long_aol.LONG_AOL_WORDS
  assert classic_long_aol.qualified(cp)
  assert not classic_long_aol.qualified(cp, marked_only=True)
  mark(cp)
  assert classic_long_aol.qualified(cp, marked_only=True)
  assert classic_long_aol.native_accepts(cp, int(cp.safetyConfigs[0].safetyModel.raw), word)


def test_no_radar_disable_startup_or_independent_buttons():
  # Camera SCC has no radar ECU to silence; the main and LDA buttons always toggle lateral.
  cp = params()
  assert CAR.HYUNDAI_ELANTRA_HEV_2024 not in classic_long_aol.CLASSIC_LONG_IDS
  assert not ordinary_classic_startup.required(cp)
  assert not CarInterface.startup_required(cp)
  assert not classic_long_aol.independent_buttons_qualified(cp, marked_only=False)
  assert not classic_long_aol.independent_buttons_qualified(mark(cp), marked_only=True)


def test_main_tracking_survives_aol_marking():
  cp = params()
  assert long_buttons.qualified(cp)
  assert long_buttons.qualified(mark(cp))
  assert not long_buttons.qualified(mark(params(), experience=0)), 'Marked word without the AOL experience'
  cs = CarState(mark(params()))
  assert cs.long_buttons is not None and cs.long_buttons.track_main
  assert isinstance(cs.forte_lkas_sources, ClassicSccLkasSources)


@pytest.mark.parametrize('experience', [1, 33])
def test_foreign_experience_is_rejected(experience):
  cp = mark(params(), experience)
  assert not classic_long_aol.qualified(cp)
  assert not long_buttons.qualified(cp)


def test_stock_and_other_camera_scc_cars_are_unchanged():
  assert not classic_long_aol.qualified(params(alpha_long=False))
  fp = gen_empty_fingerprint()
  fp[0][0x391] = 8
  elantra_gas = CarInterface.get_params(CAR.HYUNDAI_ELANTRA_2024, fp, [], True, False, False)
  assert not classic_long_aol.qualified(elantra_gas)
