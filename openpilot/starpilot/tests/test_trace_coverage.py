"""1337 tracing coverage: crashed or restarted programs, error lines, flicker-folded decision logs, rate-limited
errors, and the Force Stop decision record (what it follows, or what holds it back)."""
import json

from openpilot.starpilot.diagnostics import features, traced
from openpilot.starpilot.diagnostics.features import ChangeLog, log_error, log_event
from openpilot.starpilot.longitudinal.force_stop import ForceStop, vehicle_tune
from openpilot.starpilot.longitudinal.tests.test_force_stop import frame


class Events(list):
  def __call__(self, event, **fields):
    self.append(fields)


def test_change_log_folds_a_flickering_decision_into_one_event():
  events = Events()
  log = ChangeLog('demo', log=events, min_gap_s=1.0)
  assert log.observe({'state': 'a'}, now=0.0)
  for i, state in enumerate('babab'):          # five changes within the gap
    assert not log.observe({'state': state}, now=0.1 + i * 0.1)
  assert not log.observe({'state': 'b'}, now=0.9)
  assert log.observe({'state': 'b'}, now=1.2)  # the latest decision, once the gap has passed
  assert events[-1]['state'] == 'b' and events[-1]['flickers'] == 5
  assert not log.observe({'state': 'b'}, now=5.0)
  assert log.observe({'state': 'a'}, now=5.1) and 'flickers' not in events[-1]


def test_change_log_without_a_gap_is_unchanged():
  events = Events()
  log = ChangeLog('demo', log=events)
  assert log.observe({'x': 1}) and not log.observe({'x': 1}) and log.observe({'x': 2})
  assert len(events) == 2


def test_errors_are_logged_with_a_repeat_count_instead_of_hidden():
  features._event_last.clear()
  events = Events()
  assert log_error('demo', ValueError('bad'), where='loop', log=events, now=0.0)
  for t in (1.0, 2.0, 3.0):
    assert not log_error('demo', ValueError('bad'), where='loop', log=events, now=t)
  assert log_error('demo', 'other place', where='save', log=events, now=3.0)   # another place is separate
  assert log_error('demo', ValueError('bad'), where='loop', log=events, now=31.0)
  assert events[0]['error'] == 'ValueError: bad' and events[0]['what'] == 'error'
  assert events[-1]['repeats_since_last'] == 3
  assert log_event('demo_reset', 'camera', log=events, now=0.0, reason='camera')
  assert not log_event('demo_reset', 'camera', log=events, now=1.0, reason='camera')


def test_program_crash_restart_and_staying_down_are_reported():
  trace = traced.Trace()
  up = [('bluedriverd', True, True, 100, 0), ('edgemapd', True, True, 200, 0)]
  assert trace.processes(0.0, up) == []
  died = trace.processes(0.5, [('bluedriverd', False, True, 100, -9), ('edgemapd', True, True, 201, 0)])
  assert [(d['name'], d['what']) for d in died] == [('bluedriverd', 'died'), ('edgemapd', 'restarted')]
  assert 'SIGKILL' in died[0]['meaning']
  assert trace.processes(3.0, [('bluedriverd', False, True, 100, -9)]) == []
  down = trace.processes(6.0, [('bluedriverd', False, True, 100, -9)])
  assert [d['what'] for d in down] == ['still down']
  assert trace.processes(7.0, [('bluedriverd', False, True, 100, -9)]) == []   # reported once
  back = trace.processes(8.0, [('bluedriverd', True, True, 300, 0)])
  assert [d['what'] for d in back] == ['back up'] and back[0]['down_s'] == 7.5


def test_a_normal_shutdown_is_not_a_crash():
  trace = traced.Trace()
  trace.processes(0.0, [('curvemapd', True, True, 10, 0)])
  assert trace.processes(1.0, [('curvemapd', False, False, 10, 0)]) == []   # the manager stopped it (car off)
  assert trace.processes(9.0, [('curvemapd', False, False, 0, 0)]) == []


def test_exit_meanings():
  assert traced.exit_meaning(0) == 'clean exit'
  assert traced.exit_meaning(1) == 'exited with error 1'
  assert 'SIGSEGV' in traced.exit_meaning(-11)
  assert 'SIGKILL' in traced.exit_meaning(137)


def test_error_lines_carry_the_traceback_tail_and_skip_decision_events():
  crash = json.dumps({'msg': 'crash', 'ctx': {'daemon': 'efficiencyd'}, 'filename': 'process.py', 'lineno': 40,
                      'exc_info': 'Traceback (most recent call last):\n  File "x.py", line 1\nKeyError: \'fuel\''})
  record = traced.error_record(crash)
  assert record['daemon'] == 'efficiencyd' and record['where'] == 'process.py:40'
  assert record['exception'][-1] == "KeyError: 'fuel'"
  feature = json.dumps({'msg': {'event': 'starpilot_feature', 'feature': 'x', 'error': 'e'}})
  assert traced.error_record(feature) is None
  assert traced.error_record('not json') is None
  trace = traced.Trace()
  assert trace.error(0.0, record) is not None
  assert trace.error(5.0, dict(record)) is None
  assert trace.error(31.0, dict(record))['repeats_since_last'] == 1


def test_force_stop_says_what_it_follows_or_what_holds_it_back():
  tune = vehicle_tune('HYUNDAI_ELANTRA_HEV_2024')
  owner = ForceStop()
  owner.step(frame(0), tune)
  assert owner.decision['path'] == 'light' and owner.decision['light']
  for tick in range(1, 60):
    owner.step(frame(tick, horizon_m=max(80. - tick, 20.)), tune)
  assert owner.forcing and owner.decision['blocked_by'] == ''

  cases = {'lead car ahead (it does the stopping)': {'tracking_lead': True},
           'in a curve': {'driving_in_curve': True},
           'light too far (model horizon over 100 m)': {'horizon_m': 150.},
           'perception unavailable': {'perception_available': False}}
  for expected, change in cases.items():
    owner = ForceStop()
    owner.step(frame(0, **change), tune)
    assert owner.decision['path'] in ('none', 'slowing toward a light'), (change, owner.decision)
    assert owner.decision['blocked_by'] == expected and not owner.forcing, (change, owner.decision)

  owner = ForceStop()
  owner.step(frame(0, light=False), tune)
  assert owner.decision == {'path': 'none', 'blocked_by': '', 'light': False, 'sign': False}
  owner.step(frame(1, enabled=False), tune)
  assert owner.decision['blocked_by'] == 'not enabled'
