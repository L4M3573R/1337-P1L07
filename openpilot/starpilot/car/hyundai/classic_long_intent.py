from opendbc.car.hyundai.classic_long_aol import CAMERA_LONG_IDS, independent_buttons_qualified
from opendbc.car.hyundai.blended_stock_aol import qualified_alpha
from opendbc.car.structs import car
from openpilot.starpilot.aol.intent import AolCardIntent

GearShifter = car.CarState.GearShifter
ButtonType = car.CarState.ButtonEvent.Type
DRIVING_GEARS = (GearShifter.drive, GearShifter.sport, GearShifter.low, GearShifter.brake)
# Leaving Drive withdraws steering through the transport/fault path, which also clears the latch.
GEAR_REARM_REASONS = ('disarming_fault', 'native_rejection')
DRIVER_DECISION_BUTTONS = (ButtonType.mainCruise, ButtonType.cancel, ButtonType.lkas)


class ClassicLongCardIntent(AolCardIntent):
  defer_full_engagement_on_gas = True

  def __init__(self, cp, settings):
    super().__init__(settings, explicit_latch=True, independent_cruise_buttons=(
      independent_buttons_qualified(cp, marked_only=False)),
      preserve_unassigned_cancel=qualified_alpha(cp))
    self.requires_fault_observation = True
    self.observe_stock_engagement = True
    # Camera-long Elantra Hybrid: shifting P/R/N and back to Drive keeps the driver's AOL choice.
    self.gear_rearm = cp.carFingerprint in CAMERA_LONG_IDS
    self.gear_rearm_pending = False

  def _arm_from_full_engagement(self, CS, *, source_ns):
    if not (CS.steerFaultTemporary or CS.gasPressed or CS.brakePressed):
      super()._arm_from_full_engagement(CS, source_ns=source_ns)

  def update(self, CS, **kwargs) -> None:
    latched = self.allowed_latch
    super().update(CS, **kwargs)
    if not self.gear_rearm:
      return
    driving = CS.gearShifter in DRIVING_GEARS
    if latched and not self.allowed_latch and not driving and self.latch_clear_reason in GEAR_REARM_REASONS:
      # The latch fell only because the car left Drive; the driver never turned AOL off.
      self.gear_rearm_pending = True
    if not self.gear_rearm_pending:
      return
    if any(event.type in DRIVER_DECISION_BUTTONS and event.pressed for event in CS.buttonEvents) or not self.settings.enabled:
      self.gear_rearm_pending = False  # a deliberate driver action replaces the remembered choice
      return
    healthy = (CS.canValid and not CS.canTimeout and not CS.accFaulted and not CS.steerFaultPermanent and
               not self._fault_inhibit and not kwargs.get('fault_active'))
    if driving and healthy and CS.cruiseState.available and not self.allowed_latch:
      self.allowed_latch = True
      self.pause_lateral = False
      self.gear_rearm_pending = False
