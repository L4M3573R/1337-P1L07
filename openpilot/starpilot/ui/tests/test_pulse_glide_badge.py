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
