from openpilot.starpilot.audio.driver_steer_warning import ALERT_TYPE, RELEASE_NS, DriverSteerWarning

S = 1_000_000_000


def test_hidden_while_the_driver_steers_and_just_after():
  w = DriverSteerWarning()
  assert w.hide(True, 10 * S)
  assert w.hide(True, 11 * S)
  assert w.hide(False, 11 * S + RELEASE_NS)  # the fault clears a moment after letting go
  assert not w.hide(False, 11 * S + RELEASE_NS + 1)


def test_shown_when_the_driver_never_touched_the_wheel():
  w = DriverSteerWarning()
  assert not w.hide(False, 10 * S)
  assert not w.hide(False, 20 * S)


def test_a_clock_step_backwards_never_hides():
  w = DriverSteerWarning()
  w.hide(True, 10 * S)
  assert not w.hide(False, 9 * S)


def test_drive_000000e1_full_lock_sequence():
  # 12:34 parking lot: driver at -450 deg with torque maxed through all three faults.
  w = DriverSteerWarning()
  pressed = [(t, True) for t in range(0, 7 * S, S // 100)] + [(7 * S + 300_000_000, False)]
  faults = (1_800_000_000, 3_400_000_000, 6_000_000_000, 7 * S + 300_000_000)
  shown = []
  for t, p in pressed:
    if w.hide(p, t) is False and t in faults:
      shown.append(t)
  assert shown == [] and ALERT_TYPE == 'steerTempUnavailableSilent/warning'
