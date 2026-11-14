"""plannerd's fuel-experiment host: config, status for the ledger, Pulse and Glide hand-over, never raising."""
import json
from types import SimpleNamespace as NS

from openpilot.starpilot.efficiency import experiment as ex
from openpilot.starpilot.efficiency.experiment_host import ExperimentHost

MPH = ex.MPH


def sm(v=50 * MPH, set_mph=50, grade_pitch=0.0, lead=None, gas=False, brake=False, standstill=False):
  lead = lead or NS(present=False, dRel=0.0, vLead=0.0)
  return {'carState': NS(vEgo=v, aEgo=0.0, vCruise=set_mph * 1.609344, gasPressed=gas, brakePressed=brake, standstill=standstill),
          'carControl': NS(longActive=True, enabled=True, orientationNED=[0.0, grade_pitch, 0.0]),
          'radarState': NS(leadOne=lead), 'modelV2': NS(action=NS(shouldStop=False)), 'controlsState': NS(forceDecel=False)}


class Owner:
  def __init__(self):
    self.pulse_glide = False
    self.change_reason = 'start'


class Clock:
  def __init__(self):
    self.t = 100.0

  def __call__(self):
    self.t += 0.05
    return self.t


def host(tmp_path, mode='on', winners=None):
  if mode != 'on':
    (tmp_path / 'experiment.json').write_text(json.dumps({'mode': mode}))
  if winners:
    (tmp_path / 'experiments.json').write_text(json.dumps({'winners': winners}))
  return ExperimentHost(root=tmp_path, status_path=tmp_path / 'status.json', clock=Clock())


def test_off_mode_does_nothing(tmp_path):
  h = host(tmp_path, mode='off')
  assert h.sample(sm(), NS(owner=Owner()), drive=1, force_stop_forcing=False) is None


def test_status_reaches_the_ledger_reader(tmp_path):
  h = host(tmp_path, winners={'cruise|45-60': 'under2', 'pulse_glide|45-60': 'steady'})
  h._new_drive(1)
  h.exp.rng.random = lambda: 0.99   # no exploring
  plan = None
  for _ in range(5):
    plan = h.sample(sm(), NS(owner=Owner()), drive=1, force_stop_forcing=False)
  assert plan is not None and plan.tag
  assert ex.read_tag(h.clock.t, tmp_path / 'status.json') == plan.tag
  assert ex.read_tag(h.clock.t + 10, tmp_path / 'status.json') is None   # stale: no tag


def test_pulse_glide_experiment_hands_back_and_respects_the_driver(tmp_path):
  h = host(tmp_path, winners={'pulse_glide|45-60': 'pg'})
  h._new_drive(1)
  h.exp.rng.random = lambda: 0.99   # no exploring
  owner = NS(pulse_glide=False, change_reason='start')
  h.sample(sm(), NS(owner=owner), drive=1, force_stop_forcing=False)
  assert owner.pulse_glide and owner.change_reason == 'efficiency experiment'
  h.sample(sm(brake=True), NS(owner=owner), drive=1, force_stop_forcing=False)   # episode ends on the pedal
  assert not owner.pulse_glide
  # the driver turns it on themselves: from now on it is theirs
  owner.pulse_glide, owner.change_reason = True, 'driver toggled pulse and glide'
  for _ in range(5):
    h.sample(sm(), NS(owner=owner), drive=1, force_stop_forcing=False)
  assert owner.pulse_glide and owner.change_reason == 'driver toggled pulse and glide'


def test_a_new_drive_reloads_winners_and_forgets_the_driver_choice(tmp_path):
  h = host(tmp_path)
  h.driver_pulse_glide = True
  h.sample(sm(), NS(owner=Owner()), drive=2, force_stop_forcing=False)
  assert not h.driver_pulse_glide


def test_broken_input_never_raises(tmp_path):
  h = host(tmp_path)
  assert h.sample({}, None, drive=1, force_stop_forcing=False) is None



def test_a_new_veto_is_merged_with_the_file_not_written_over_it(tmp_path):
  h = host(tmp_path)
  h._new_drive(1)
  # something else recorded a veto after the drive began
  (tmp_path / 'vetoes.json').write_text(json.dumps({'vetoes': {'accelerate|30-45|gentle': 1}}))
  h.exp.vetoes['follow|30-45|smooth'] = 1
  h.sample(sm(), NS(owner=Owner()), drive=1, force_stop_forcing=False)
  saved = json.loads((tmp_path / 'vetoes.json').read_text())['vetoes']
  assert saved == {'accelerate|30-45|gentle': 1, 'follow|30-45|smooth': 1}
