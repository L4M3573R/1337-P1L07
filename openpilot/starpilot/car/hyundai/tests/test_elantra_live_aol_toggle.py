"""Elantra Hybrid camera-long: AOL is always armed at startup so the saved toggle works live."""
import tempfile
from dataclasses import replace

from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai import classic_long_aol
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from opendbc.car.structs import car
from openpilot.common.params import Params
from openpilot.selfdrive.monitoring.dmonitoringd import aol_runtime_armed
from openpilot.starpilot.aol.intent import AolSettings
from openpilot.starpilot.aol.vehicle import policy_for
from openpilot.starpilot.car.hyundai.classic_long_intent import ClassicLongCardIntent
from openpilot.starpilot.feature_runtime import enabled as feature_enabled

OFF = AolSettings(enabled=False, pause_brake_mps=0.0, lkas_action=0, main_action=0,
                  distance_actions=(0, 0, 0), cancel_actions=(0, 0, 0))
ON = replace(OFF, enabled=True)


def elantra_cp(marked=False):
  fp = gen_empty_fingerprint()
  fp[0][0x391] = fp[0][0x50c] = 8
  cp = CarInterface.get_params(CAR.HYUNDAI_ELANTRA_HEV_2024, fp, [], True, False, False)
  if marked:  # what card.py publishes after applying the vehicle policy
    cp.safetyConfigs[0].safetyParam = classic_long_aol.aol_word(cp)
    cp.alternativeExperience = classic_long_aol.AOL_EXPERIENCE
  return cp


def params_with_aol(value):
  params = Params(tempfile.mkdtemp())
  params.put_bool("AlwaysOnLateral", value, block=True)
  return params


def test_arms_at_startup_even_with_the_toggle_off():
  cp = elantra_cp()
  assert classic_long_aol.qualified(cp)
  assert policy_for(cp).full_axis_runtime_required
  assert feature_enabled(params_with_aol(False), cp, 'aol', {})
  assert feature_enabled(params_with_aol(False), elantra_cp(marked=True), 'aol', {})


def driving():
  cs = car.CarState(canValid=True, gearShifter='drive', vEgo=15.0)
  cs.cruiseState.available = True
  return cs


def press_main(owner, cs, now):
  cs.buttonEvents = [car.CarState.ButtonEvent(type='mainCruise', pressed=True)]
  owner.update(cs, now_ns=now)
  cs.buttonEvents = [car.CarState.ButtonEvent(type='mainCruise', pressed=False)]
  owner.update(cs, now_ns=now + 1)
  cs.buttonEvents = []


def test_toggle_applies_live_without_a_restart():
  cp, cs = elantra_cp(marked=True), driving()
  owner = ClassicLongCardIntent(cp, OFF)
  owner.update(cs, now_ns=1)
  press_main(owner, cs, 10)
  assert not owner.allowed_latch  # toggle off: cruise main does not engage steering

  owner.settings = ON  # card re-reads the saved toggle every 0.1 s
  owner.update(cs, now_ns=20)
  assert not owner.allowed_latch  # turning it on never starts steering by itself
  press_main(owner, cs, 30)
  assert owner.allowed_latch and owner.output(cs)[0]  # next cruise main press engages AOL

  owner.settings = OFF
  owner.update(cs, now_ns=40)
  assert not owner.allowed_latch and not owner.output(cs)[0]  # turning it off releases immediately


class _Params:
  def __init__(self, cp_bytes, aol):
    self.cp_bytes, self.inner = cp_bytes, params_with_aol(aol)

  def get(self, key, *args, **kwargs):
    return self.cp_bytes if key == "CarParams" else self.inner.get(key, *args, **kwargs)

  def __getattr__(self, name):
    return getattr(self.inner, name)


def test_driver_monitoring_follows_the_armed_runtime_not_the_startup_toggle():
  assert aol_runtime_armed(_Params(None, False)) is None  # waits for this drive's CarParams
  assert aol_runtime_armed(_Params(elantra_cp(marked=True).to_bytes(), False)) is True
