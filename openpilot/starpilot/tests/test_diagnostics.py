"""Per-feature decision logs record each change once, explain it, and never break their producer."""
from types import SimpleNamespace as NS

from openpilot.starpilot.diagnostics import features
from openpilot.starpilot.diagnostics.features import ChangeLog, aol_decision, radar_context, radar_decision, turn_decision


def recorder():
  records = []
  return records, (lambda event, **fields: records.append((event, fields)))


def test_change_log_records_each_decision_change_once_with_context():
  records, log = recorder()
  turns = ChangeLog('turn_intent', log=log)
  assert turns.observe({'desire': 2, 'source': 'blinker'}, {'v_ego': 6.0})
  assert not turns.observe({'desire': 2, 'source': 'blinker'}, {'v_ego': 6.5})  # context alone is not a change
  assert turns.observe({'desire': 0, 'source': ''}, {'v_ego': 7.0})
  assert records == [(features.EVENT, {'feature': 'turn_intent', 'desire': 2, 'source': 'blinker', 'v_ego': 6.0}),
                     (features.EVENT, {'feature': 'turn_intent', 'desire': 0, 'source': '', 'v_ego': 7.0})]


def test_a_failing_logger_drops_the_record_instead_of_raising():
  def broken(*_, **__):
    raise RuntimeError('log unavailable')
  assert not ChangeLog('aol', log=broken).observe({'allowed_latch': True})


def test_turn_decision_names_the_source_or_the_gate():
  sent = turn_decision(NS(source='blinker', blocked_by='', signal_stop_hold=False, stop_hold=False), 2)
  assert sent == {'desire': 2, 'source': 'blinker', 'blocked_by': '', 'signal_stop_hold': False, 'nav_stop_hold': False}
  held = turn_decision(NS(source='', blocked_by='stop predicted', signal_stop_hold=True, stop_hold=False), 0)
  assert held['blocked_by'] == 'stop predicted' and held['signal_stop_hold']
  assert turn_decision(NS(), object())['desire'] == -1


def test_radar_decision_and_context():
  tandem = NS(last_action='hold', held=NS(track_id=4, frames_without_camera=7), last_pickup_id=-1, last_lead_prob=0.3)
  assert radar_decision(tandem) == {'action': 'hold', 'held_track': 4, 'pickup_track': -1}
  assert radar_context(tandem, {'present': True, 'dRel': 31.5}) == {'lead_prob': 0.3, 'frames_without_camera': 7,
                                                                      'lead_drel': 31.5}
  assert 'lead_drel' not in radar_context(tandem, NS(present=False))


def test_aol_decision_reports_latch_arm_and_reason():
  intent = NS(allowed_latch=False, lateral_arm_retained=lambda: True, latch_clear_reason='disarming_fault',
              latch_clear_count=3, gear_rearm_pending=True, _fault_inhibit=False, settings=NS(enabled=True))
  assert aol_decision(intent) == {'allowed_latch': False, 'lateral_armed': True, 'latch_clear_reason': 'disarming_fault',
                                  'latch_clear_count': 3, 'gear_rearm_pending': True, 'fault_inhibit': False,
                                  'settings_enabled': True}


def test_cem_stop_fields():
  from openpilot.starpilot.conditional_mode.stop import StopObservation, stop_log_fields
  detector = NS(light_filter=NS(x=0.7), committed=True)
  fields = stop_log_fields(StopObservation(True, False, None, True), detector)
  assert fields['cemStopKnown'] and fields['cemStopLightDetected'] and fields['cemStopModelStopping']
  assert fields['cemLightFilter'] == 0.7 and fields['cemStopCommitted']
  withheld = stop_log_fields(StopObservation(None, None, None, None), detector)
  assert not withheld['cemStopKnown'] and withheld['cemStopCommitted']
  assert stop_log_fields(None, None) == {'cemStopKnown': False}
