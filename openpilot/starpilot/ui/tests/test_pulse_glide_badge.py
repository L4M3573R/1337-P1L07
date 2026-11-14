"""1337: the onroad Pulse and Glide badge (the mode had no on-screen state, so a toggle could not be seen)."""
from types import SimpleNamespace as NS

from openpilot.starpilot.ui.runtime_snapshot import pulse_glide_badge


def plan(enabled=True, phase='off', hill=False):
  return NS(pulseGlideEnabled=enabled, pulseGlidePhase=phase, pulseGlideHillPaused=hill)


def test_badge_follows_the_planner():
  assert pulse_glide_badge(None) is None
  assert pulse_glide_badge(plan(enabled=False)) is None
  assert pulse_glide_badge(plan(phase='pulse')) == 'pulse'
  assert pulse_glide_badge(plan(phase='glide')) == 'glide'
  assert pulse_glide_badge(plan(phase='off')) == 'waiting'       # on, but held off (lead, stop, below the band)
  assert pulse_glide_badge(plan(phase='glide', hill=True)) == 'hill'
  assert pulse_glide_badge(plan(enabled=False, phase='forceCoast')) == 'coast'
  assert pulse_glide_badge(NS()) is None                        # a malformed message never breaks the UI


def test_the_screen_state_carries_pulse_glide_from_the_planner():
  from openpilot.starpilot.ui.shell import ShellMode
  from openpilot.starpilot.ui.tests.test_runtime_snapshot import NOW, RuntimeSnapshotAdapter, ui_fake
  ui = ui_fake()
  ui.sm.put('starpilotLongitudinalPlan', plan(phase='glide'))
  assert RuntimeSnapshotAdapter(ui).build(ShellMode.ONROAD, now_ns=NOW).onroad.pulse_glide == 'glide'
  ui.sm.put('starpilotLongitudinalPlan', plan(phase='glide'), age_ns=2_000_000_000)   # stale: nothing shown
  assert RuntimeSnapshotAdapter(ui).build(ShellMode.ONROAD, now_ns=NOW).onroad.pulse_glide is None


def test_comma_4_small_screen_draws_the_badge():
  # the comma 4 uses the compact layout; the badge was first drawn only in the large one (2026-11-14)
  from unittest import mock
  from openpilot.starpilot.ui.onroad import OnroadView
  from openpilot.starpilot.ui.onroad_state import AlertSize
  from openpilot.starpilot.ui.presentation import Profile
  view = OnroadView.__new__(OnroadView)
  view.fonts = mock.Mock(profile=Profile.COMPACT)
  view.fonts.measure.return_value = NS(width=90, height=20)
  with mock.patch('openpilot.starpilot.ui.onroad.rl.draw_rectangle_rounded') as pill:
    view._pulse_glide_badge_compact(NS(pulse_glide='glide', alert=NS(size=AlertSize.NONE)))
    assert view.fonts.draw.call_args.args[0] == 'P&G: glide' and pill.called
    view.fonts.reset_mock()
    view._pulse_glide_badge_compact(NS(pulse_glide=None, alert=NS(size=AlertSize.NONE)))
    view._pulse_glide_badge_compact(NS(pulse_glide='pulse', alert=NS(size=AlertSize.SMALL)))   # an alert has the space
    view.fonts.draw.assert_not_called()
