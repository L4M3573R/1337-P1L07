"""Elantra Hybrid AOL survives Park/Reverse/Neutral and back to Drive (route 3d: latch fell at D->N)."""
from dataclasses import replace

import pytest
from opendbc.car.structs import car
from openpilot.starpilot.car.hyundai.classic_long_intent import ClassicLongCardIntent
from openpilot.starpilot.car.hyundai.tests.test_elantra_live_aol_toggle import ON, elantra_cp


def state(gear, main=True, buttons=()):
  cs = car.CarState(canValid=True, gearShifter=gear, vEgo=0.0)
  cs.cruiseState.available = main
  cs.buttonEvents = [car.CarState.ButtonEvent(type=b, pressed=True) for b in buttons]
  return cs


def armed():
  owner = ClassicLongCardIntent(elantra_cp(marked=True), ON)
  owner.update(state('drive'), now_ns=1)
  owner.allowed_latch = True  # armed by an earlier cruise-main press
  return owner


def shift_out_and_back(owner, *, park_buttons=(), drive_fault=False, settings=ON):
  # Leaving Drive: the transport monitor reports a fault and the latch clears (as on route 3d).
  owner.update(state('neutral'), now_ns=10, fault_active=True)
  assert not owner.allowed_latch
  owner.update(state('reverse'), now_ns=20, fault_active=False)
  owner.update(state('park', main=False, buttons=park_buttons), now_ns=30, fault_active=False)  # Park: car reports cruise unavailable
  owner.settings = settings
  owner.update(state('neutral'), now_ns=40, fault_active=False)
  owner.update(state('drive'), now_ns=50, fault_active=drive_fault)


def test_aol_rearms_on_return_to_drive_with_main_still_on():
  owner = armed()
  shift_out_and_back(owner)
  assert owner.allowed_latch and owner.output(state('drive'))[0]


@pytest.mark.parametrize('button', ['mainCruise', 'cancel', 'lkas'])
def test_a_deliberate_button_while_parked_replaces_the_remembered_choice(button):
  owner = armed()
  owner.update(state('neutral'), now_ns=10, fault_active=True)
  assert owner.gear_rearm_pending
  owner.update(state('park', main=False, buttons=(button,)), now_ns=20, fault_active=False)
  assert not owner.gear_rearm_pending  # the press (handled by the normal gesture rules) decides, not the re-arm


def test_aol_turned_off_in_settings_does_not_rearm():
  owner = armed()
  shift_out_and_back(owner, settings=replace(ON, enabled=False))
  assert not owner.allowed_latch


def test_a_fault_still_active_in_drive_defers_the_rearm_until_it_clears():
  owner = armed()
  shift_out_and_back(owner, drive_fault=True)
  assert not owner.allowed_latch
  owner.update(state('drive'), now_ns=60, fault_active=False)
  assert owner.allowed_latch


def test_a_fault_while_driving_is_not_rearmed():
  owner = armed()
  owner.update(state('drive'), now_ns=10, fault_active=True)
  assert not owner.allowed_latch
  owner.update(state('drive'), now_ns=20, fault_active=False)
  assert not owner.allowed_latch and not owner.gear_rearm_pending


def test_main_off_on_return_to_drive_does_not_rearm():
  owner = armed()
  owner.update(state('neutral'), now_ns=10, fault_active=True)
  owner.update(state('drive', main=False), now_ns=20, fault_active=False)
  assert not owner.allowed_latch


def test_only_the_camera_long_elantra_gets_the_gear_rearm():
  other = elantra_cp(marked=True)
  other.carFingerprint = 'HYUNDAI_SONATA_HYBRID'
  assert ClassicLongCardIntent(elantra_cp(marked=True), ON).gear_rearm
  assert not ClassicLongCardIntent(other, ON).gear_rearm
