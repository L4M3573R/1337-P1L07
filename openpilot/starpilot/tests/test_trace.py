"""1337 tracing: lead-loss and slow-work decision logs, Pulse and Glide reasons, the on-comma trace file, the timeline."""
import json
from types import SimpleNamespace as NS

from openpilot.starpilot.controllers.coast import CoastOwner
from openpilot.starpilot.diagnostics import features, trace_report, traced
from openpilot.starpilot.diagnostics.features import LeadLossLog, log_if_slow


class Events(list):
  def __call__(self, event, **fields):
    self.append(fields)


def lead(d, v, prob=0.9, radar=True):
  return {'present': True, 'dRel': d, 'vRel': v, 'modelProb': prob, 'radar': radar}


NO_LEAD = {'present': False}


def test_closing_lead_lost_and_found_where_it_should_be():
  # 11-13 7:55pm drive: a car 75 m ahead, closing at 7.9 m/s, vanished for ~8 s and came back 38 m away
  log = Events()
  tracker = LeadLossLog(log)
  tracker.observe(0.0, lead(75.0, -7.9), 20.0)
  tracker.observe(0.05, NO_LEAD, 20.0)
  assert log[-1]['what'] == 'lost' and log[-1]['drel'] == 75.0 and log[-1]['time_to_reach_s'] == 9.5
  tracker.observe(4.0, NO_LEAD, 18.0)          # still gone: nothing more
  assert len(log) == 1
  tracker.observe(4.5, lead(40.0, -7.5), 18.0)  # 75 - 7.9 * 4.5 = 39.45: the same car all along
  assert log[-1]['what'] == 'returned' and log[-1]['likely_same_object'] and abs(log[-1]['expected_drel'] - 39.45) < 0.1


def test_far_or_pulling_away_leads_are_not_logged():
  log = Events()
  tracker = LeadLossLog(log)
  for d, v in ((120.0, -5.0), (50.0, +2.0)):
    tracker.observe(0.0, lead(d, v), 20.0)
    tracker.observe(0.05, NO_LEAD, 20.0)
  assert log == []
  tracker.observe(1.0, lead(20.0, +1.0), 5.0)   # but anything within 30 m always counts
  tracker.observe(1.05, NO_LEAD, 5.0)
  assert log[-1]['what'] == 'lost'
  tracker.observe(30.0, lead(60.0, 0.0), 5.0)   # far later: a new lead, not a return
  assert len(log) == 1


def test_slow_work_is_logged_at_most_every_ten_seconds():
  log = Events()
  features._slow_last.clear()
  assert not log_if_slow('lookup_x', 0.2, 0.3, log=log, now=100.0)
  assert log_if_slow('lookup_x', 0.9, 0.3, log=log, now=100.0, branches=3)
  assert not log_if_slow('lookup_x', 0.9, 0.3, log=log, now=105.0)
  assert log_if_slow('lookup_x', 0.9, 0.3, log=log, now=111.0)
  assert log[0] == {'feature': 'lookup_x', 'slow_s': 0.9, 'threshold_s': 0.3, 'branches': 3}


def test_pulse_and_glide_says_why_it_changed():
  owner = CoastOwner()
  owner.toggle(14, long_active=True)
  assert owner.pulse_glide and owner.change_reason == 'driver toggled pulse and glide'
  args = {'active': True, 'gas': False, 'brake': False, 'speed_mps': 20.0, 'target_mps': 20.0, 'delta_mps': 2.0,
          'pitch': 0.0, 'lead_relevant': False, 'stop_context': False}
  owner.sample(admitted=False, **args)
  assert not owner.pulse_glide and owner.change_reason == 'not admitted'
  owner.toggle(14, long_active=True)
  owner.sample(admitted=True, **{**args, 'speed_mps': -3.0})
  assert owner.change_reason.startswith('bad input (speed -3.0')
  owner.toggle(2, long_active=True)
  owner.sample(admitted=True, **{**args, 'brake': True})
  assert not owner.force_coast and owner.change_reason == 'force coast ended by the pedals'


def test_trace_collects_decision_events_from_the_app_log():
  raw = json.dumps({'msg': {'event': 'starpilot_feature', 'feature': 'lead_loss', 'what': 'lost', 'drel': 75.0},
                    'ctx': {'daemon': 'radard'}})
  assert traced.feature_record(raw) == {'feature': 'lead_loss', 'what': 'lost', 'drel': 75.0, 'kind': 'feature', 'daemon': 'radard'}
  assert traced.feature_record(json.dumps({'msg': 'plain text'})) is None
  assert traced.feature_record('{broken') is None


def test_alert_snapshot_names_the_busy_processes_and_cores():
  trace = traced.Trace()
  for i in range(4):
    trace.device(i, [50.0, 60.0, 70.0, 80.0, 100.0, 90.0, 60.0, 70.0], 69.0, 69)
  trace.proc(0.0, {1: (10.0, 'card'), 2: (5.0, 'controlsd')})
  trace.proc(2.0, {1: (11.9, 'card'), 2: (6.6, 'controlsd')})
  ev = lambda name, soft=False: NS(name=name, immediateDisable=False, softDisable=soft)  # noqa: E731
  line = trace.events(2.0, traced.alert_names([ev('selfdrivedLagging', True), ev('preLaneChangeLeft')]))
  assert line['events'] == ['selfdrivedLagging'] and line['cores_now'][4] == 100.0
  assert line['busiest_last_4s'] == [('card', 95.0), ('controlsd', 80.0)]
  assert trace.events(2.5, {'selfdrivedLagging'}) is None   # the same alert still showing: one line only


def test_per_minute_load_line_and_name_order_pruning(tmp_path):
  trace = traced.Trace()
  trace.device(0, [10.0, 20.0], 50.0, 40)
  assert trace.proc(0.0, {1: (0.0, 'modeld')}) is None
  trace.device(30, [30.0, 40.0], 51.0, 40)
  line = trace.proc(61.0, {1: (30.5, 'modeld')})
  assert line['kind'] == 'load' and line['cores_avg'] == [20.0, 30.0] and line['busiest'] == [('modeld', 50.0)]
  for day in ('2026-11-01', '2026-11-02', '2026-11-03'):
    (tmp_path / f'{day}.jsonl').write_text('{}\n')
  traced.Writer(tmp_path).prune(keep_days=2)
  assert sorted(p.name for p in tmp_path.iterdir()) == ['2026-11-02.jsonl', '2026-11-03.jsonl']


def test_timeline_counts_only_whole_slow_seconds():
  t = trace_report.Timeline()
  t.start = 0.0
  t.counts = {10: 100, 11: 80, 12: 100, 20: 30}   # 20 is a segment edge (no neighbours): not a slow second
  t.second_speed = {11: 7.0}
  t.finish()
  assert [line for _, line in t.lines] == [' 15.7 mph  safety loop slow: 80/100 per second']
  assert trace_report.parse_clock('25:02.5') == 1502.5 and trace_report.clock(1502.5) == '25:02.5'
