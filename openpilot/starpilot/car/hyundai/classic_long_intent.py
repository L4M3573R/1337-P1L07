from opendbc.car.hyundai.classic_long_aol import CAMERA_LONG_IDS, independent_buttons_qualified
from opendbc.car.hyundai.blended_stock_aol import qualified_alpha
from opendbc.car.structs import car
from openpilot.starpilot.aol.intent import AOL_TOGGLE, AolCardIntent

GearShifter = car.CarState.GearShifter
ButtonType = car.CarState.ButtonEvent.Type
DRIVING_GEARS = (GearShifter.drive, GearShifter.sport, GearShifter.low, GearShifter.brake)
# Leaving Drive withdraws steering through the transport/fault path, which also clears the latch.
GEAR_REARM_REASONS = ('disarming_fault', 'native_rejection')
DRIVER_DECISION_BUTTONS = (ButtonType.mainCruise, ButtonType.cancel, ButtonType.lkas)
# A Main press that turned main on while the fault inhibit was still set arms once it clears, within 0.5 s.
MAIN_ON_WAIT_FRAMES = 50


class ClassicLongCardIntent(AolCardIntent):
  defer_full_engagement_on_gas = True

  def __init__(self, cp, settings):
    super().__init__(settings, explicit_latch=True, independent_cruise_buttons=(
      independent_buttons_qualified(cp, marked_only=False)),
      preserve_unassigned_cancel=qualified_alpha(cp))
    self.requires_fault_observation = True
    self.observe_stock_engagement = True
    # Camera-long Elantra Hybrid: like StarPilot AOL and sunnypilot MADS, P/R/N pauses steering and Drive
    # resumes it with no time limit, until the driver presses Main/Cancel/LKAS or the car reports a fault.
    self.gear_rearm = cp.carFingerprint in CAMERA_LONG_IDS
    self.gear_rearm_pending = False
    self._main_on_wait = 0

  def _arm_from_full_engagement(self, CS, *, source_ns):
    if not (CS.steerFaultTemporary or CS.gasPressed or CS.brakePressed):
      super()._arm_from_full_engagement(CS, source_ns=source_ns)

  def lateral_arm_retained(self) -> bool:
    """Published as lateralArmed so pandad keeps the Panda's physical arm while the car is out of Drive.

    Output stays gated: allowedLatch is False outside a driving gear, so no steering is commanded meanwhile.
    """
    return bool(self.allowed_latch or self.gear_rearm_pending)

  def _main_toggles_aol(self) -> bool:
    """Main is this car's AOL gesture (not reassigned to another action), as in AolCardIntent.update."""
    main_managed, lkas_managed = self.main_action == AOL_TOGGLE, self.lkas_action == AOL_TOGGLE
    independent = self.independent_cruise_buttons and (main_managed or lkas_managed)
    return (not independent or main_managed) and not (self.independent_cruise_buttons and self.settings.main_wheel_action)

  def update(self, CS, **kwargs) -> None:
    latched = self.allowed_latch
    super().update(CS, **kwargs)
    if not self.gear_rearm:
      return
    driving = CS.gearShifter in DRIVING_GEARS
    main_pressed = any(event.type == ButtonType.mainCruise and event.pressed for event in CS.buttonEvents)
    healthy = (CS.canValid and not CS.canTimeout and not CS.accFaulted and not CS.steerFaultPermanent and
               not self._fault_inhibit and not kwargs.get('fault_active'))
    self._follow_main(CS, driving=driving, main_pressed=main_pressed, healthy=healthy, now_ns=kwargs.get('now_ns', 0))
    if latched and not self.allowed_latch and not driving and self.latch_clear_reason in GEAR_REARM_REASONS:
      # The latch fell only because the car left Drive; the driver never turned AOL off.
      self.gear_rearm_pending = True
    if not driving and main_pressed and self.settings.enabled and self._main_toggles_aol():
      # Out of Drive AOL is paused and ignores the press, but the car's cruise main still toggles. Leaving the
      # choice to the car's main state in Drive keeps the two in step: a press in Park used to drop the re-arm
      # with main lit, and the next press (to arm) turned main off (2026-11-17).
      self.gear_rearm_pending = True
      return
    if not self.gear_rearm_pending:
      return
    if any(event.type in DRIVER_DECISION_BUTTONS and event.pressed for event in CS.buttonEvents) or not self.settings.enabled:
      self.gear_rearm_pending = False  # a deliberate driver action replaces the remembered choice
      return
    if not CS.canValid or CS.canTimeout or CS.accFaulted or CS.steerFaultPermanent:
      self.gear_rearm_pending = False  # no longer healthy: a fresh button press is required
      return
    if driving and healthy and CS.cruiseState.available and not self.allowed_latch:
      self._arm(now_ns=kwargs.get('now_ns', 0))
      self.gear_rearm_pending = False

  def _arm(self, *, now_ns: int) -> None:
    self.allowed_latch = True
    self.pause_lateral = False
    if now_ns:
      self.physical_arm_ns = now_ns
      self._last_latch_edge_ns = now_ns

  def _follow_main(self, CS, *, driving: bool, main_pressed: bool, healthy: bool, now_ns: int) -> None:
    """In Drive a Main press leaves AOL matching the car's own cruise main, never the opposite of it.

    The base gesture flips the latch on every press. The car flips main on every press too, but a press the
    gesture skips (stale fault inhibit, or AOL already armed in Park at startup) put the two out of step for
    good: each later press armed AOL as main turned off, arming without main is a hard transport fault, and
    the fault skipped the next press again (drive 000000cd, 2026-11-19: eight presses, never armed).
    """
    if not driving or not self.settings.enabled or not self._main_toggles_aol():
      self._main_on_wait = 0
      return
    main_on = bool(CS.cruiseState.available)
    if main_pressed:
      self._main_on_wait = 0
      if self.allowed_latch and not main_on:
        self._record_latch_clear('main_off')
        self.allowed_latch = False
      elif not self.allowed_latch and main_on:
        if healthy:
          self._arm(now_ns=now_ns)
        else:
          # The fault inhibit can lag the press by a frame or two; wait briefly, never indefinitely.
          self._main_on_wait = MAIN_ON_WAIT_FRAMES
      return
    if self._main_on_wait <= 0:
      return
    self._main_on_wait -= 1
    if (not main_on or self.allowed_latch or CS.steerFaultPermanent or CS.accFaulted or
        any(event.type in DRIVER_DECISION_BUTTONS and event.pressed for event in CS.buttonEvents)):
      self._main_on_wait = 0
    elif healthy:
      self._main_on_wait = 0
      self._arm(now_ns=now_ns)
