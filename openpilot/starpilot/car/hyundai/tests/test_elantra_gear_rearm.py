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


@pytest.mark.parametrize('button', ['cancel', 'lkas'])
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


SECOND = 1_000_000_000


def test_the_panda_arm_is_retained_while_out_of_drive_but_no_steering_is_allowed():
  # Route 45: the published arm fell in Park, pandad let the Panda's physical arm lapse, and the
  # re-armed request in Drive was refused 50 ms later. Keep publishing the arm through the shift.
  owner = armed()
  owner.update(state('neutral'), now_ns=10, fault_active=True)
  for gear, t in (('reverse', 2), ('park', 10), ('neutral', 30)):
    owner.update(state(gear), now_ns=t * SECOND, fault_active=False)
    assert owner.lateral_arm_retained()
    assert not owner.output(state(gear))[0]  # steering output stays off outside a driving gear
  owner.update(state('drive'), now_ns=31 * SECOND, fault_active=False)
  assert owner.allowed_latch and owner.lateral_arm_retained() and owner.output(state('drive'))[0]


def test_a_long_stop_in_park_still_resumes_in_drive_like_mads():
  owner = armed()
  owner.update(state('neutral'), now_ns=SECOND, fault_active=True)
  owner.update(state('park'), now_ns=20 * 60 * SECOND, fault_active=False)  # parked with the car on for 20 minutes
  assert owner.lateral_arm_retained()
  owner.update(state('drive'), now_ns=20 * 60 * SECOND + 1, fault_active=False)
  assert owner.allowed_latch


@pytest.mark.parametrize('fault', ['canValid', 'accFaulted', 'steerFaultPermanent'])
def test_a_car_fault_while_out_of_drive_drops_the_retained_arm(fault):
  owner = armed()
  owner.update(state('neutral'), now_ns=SECOND, fault_active=True)
  cs = state('park')
  if fault == 'canValid':
    cs.canValid = False
  else:
    setattr(cs, fault, True)
  owner.update(cs, now_ns=2 * SECOND, fault_active=False)
  assert not owner.lateral_arm_retained()


def test_a_disarmed_intent_does_not_publish_an_arm():
  owner = ClassicLongCardIntent(elantra_cp(marked=True), ON)
  owner.update(state('park'), now_ns=SECOND)
  assert not owner.lateral_arm_retained()


@pytest.mark.parametrize('main_in_drive', [True, False])
def test_main_pressed_in_park_follows_the_cars_main_state_in_drive(main_in_drive):
  # 2026-11-17: Main pressed in Park lit the car's main but AOL ignored it (paused out of Drive) and dropped the re-arm;
  # the next press turned main off. Now Drive follows the car's main state.
  for owner in (armed(), ClassicLongCardIntent(elantra_cp(marked=True), ON)):
    if owner.allowed_latch:
      owner.update(state('neutral'), now_ns=10, fault_active=True)
    owner.update(state('park', main=False), now_ns=20, fault_active=True)
    owner.update(state('park', main=False, buttons=('mainCruise',)), now_ns=30, fault_active=True)
    assert owner.gear_rearm_pending and not owner.allowed_latch
    owner.update(state('drive', main=main_in_drive), now_ns=40, fault_active=False)
    assert owner.allowed_latch == main_in_drive


def test_cancel_after_main_in_park_still_cancels():
  owner = ClassicLongCardIntent(elantra_cp(marked=True), ON)
  owner.update(state('park', main=False, buttons=('mainCruise',)), now_ns=10, fault_active=True)
  owner.update(state('park', main=False, buttons=('cancel',)), now_ns=20, fault_active=True)
  owner.update(state('drive'), now_ns=30, fault_active=False)
  assert not owner.allowed_latch


def press(owner, *, main, now_ns, fault_active=None, gear='drive'):
  owner.update(state(gear, main=main, buttons=('mainCruise',)), now_ns=now_ns, fault_active=fault_active)
  owner.update(state(gear, main=main), now_ns=now_ns + 1, fault_active=fault_active)  # release


def test_main_press_follows_the_cars_main_when_aol_was_armed_in_park_at_startup():
  # Drive 000000cd (2026-11-19): AOL came up armed in Park with main off; the first press turned main on
  # and the plain toggle turned AOL off, so every later press armed AOL as main went off.
  owner = ClassicLongCardIntent(elantra_cp(marked=True), ON)
  owner.update(state('drive', main=False), now_ns=1)
  owner.allowed_latch = True
  press(owner, main=True, now_ns=10)
  assert owner.allowed_latch
  press(owner, main=False, now_ns=20)
  assert not owner.allowed_latch and owner.latch_clear_reason in ('main_off', 'physical_toggle')
  press(owner, main=True, now_ns=30)
  assert owner.allowed_latch


def test_main_press_never_arms_aol_while_main_turns_off():
  owner = ClassicLongCardIntent(elantra_cp(marked=True), ON)
  owner.update(state('drive', main=True), now_ns=1)
  assert not owner.allowed_latch  # main already on, AOL off (e.g. LKAS turned it off)
  press(owner, main=False, now_ns=10)
  assert not owner.allowed_latch
  press(owner, main=True, now_ns=20)
  assert owner.allowed_latch


def test_a_main_on_press_during_a_lagging_fault_inhibit_arms_when_it_clears():
  owner = ClassicLongCardIntent(elantra_cp(marked=True), ON)
  owner.update(state('drive', main=False), now_ns=1, fault_active=True)  # stale inhibit from an earlier fault
  owner.update(state('drive', main=True, buttons=('mainCruise',)), now_ns=10)
  assert not owner.allowed_latch
  owner.update(state('drive', main=True), now_ns=11, fault_active=False)
  assert owner.allowed_latch


def test_a_fault_that_does_not_clear_soon_needs_a_fresh_press():
  owner = ClassicLongCardIntent(elantra_cp(marked=True), ON)
  owner.update(state('drive', main=False), now_ns=1, fault_active=True)
  owner.update(state('drive', main=True, buttons=('mainCruise',)), now_ns=10, fault_active=True)
  for i in range(60):
    owner.update(state('drive', main=True), now_ns=11 + i)  # inhibit stays set
  owner.update(state('drive', main=True), now_ns=100, fault_active=False)
  assert not owner.allowed_latch


def test_main_off_in_drive_drops_aol():
  owner = armed()
  press(owner, main=False, now_ns=10)
  assert not owner.allowed_latch
