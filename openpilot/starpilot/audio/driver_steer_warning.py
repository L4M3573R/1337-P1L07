"""Hide the temporary-steering warning while the driver is the one steering.

Holding the wheel at full lock makes the Elantra's EPS report steerFaultTemporary even when
the comma asks for no torque (drive 000000e1 seg 29, 2026-11-21 12:34: -450 deg at 2 mph,
driver torque maxed, comma torque 0, three chimes in 4 s). The onroad event is kept, since
AOL's intent retention reads it; only the banner and chime are dropped, and only while the
driver steers (or let go less than RELEASE_NS ago, as the fault clears a moment later).
"""

ALERT_TYPE = 'steerTempUnavailableSilent/warning'
RELEASE_NS = 1_000_000_000


class DriverSteerWarning:
  def __init__(self) -> None:
    self.pressed_ns: int | None = None

  def hide(self, steering_pressed: bool, now_ns: int) -> bool:
    if steering_pressed:
      self.pressed_ns = now_ns
    return self.pressed_ns is not None and 0 <= now_ns - self.pressed_ns <= RELEASE_NS
