import math

from openpilot.starpilot.stop_line.stop_lined import Shadow


def test_rest_event_scores_ours_and_chestnut_against_where_the_car_stopped():
  events = []
  shadow = Shadow(log=lambda event, **fields: events.append(fields))
  dt, t, v = 0.05, 0.0, 15.0
  stop_at = 100.0  # the car will come to rest at odometer 100 m
  while v > 0:
    left = stop_at - shadow.odometer_m
    v = min(15.0, math.sqrt(max(2 * 1.2 * left, 0.0))) if left > 0.05 else 0.0
    # ours says 2 m short of the real point, chestnut 8 m long
    shadow.inference((5.0, (left - 2.0) / 120.0, math.log(1.0 / 120.0)), shadow.odometer_m, t)
    shadow.tick(t, v, model_stop_m=left + 8.0, model_big=True, model_should_stop=True, ours_available=True)
    t += dt
  for _ in range(40):
    shadow.tick(t, 0.0, model_stop_m=0.0, model_big=True, model_should_stop=True, ours_available=True)
    t += dt
  rests = [e for e in events if e.get('what') == 'rest']
  assert len(rests) == 1
  rest = rests[0]
  assert not rest['queue']
  assert abs(rest['ours_err_30'] + 2.0) < 1.5
  assert abs(rest['chestnut_err_30'] - 8.0) < 1.5
  assert rest['source_30'] == 'chestnut'  # its plan stops, so it is chosen; ours is still scored
  choices = [e for e in events if e.get('source')]
  assert choices and choices[0]['source'] == 'chestnut'


def test_chestnut_input_matches_training():
  from openpilot.starpilot.stop_line.stop_lined import chestnut_input
  assert chestnut_input(None).tolist() == [[0.0, 0.0]]
  assert chestnut_input(float('nan')).tolist() == [[0.0, 0.0]]
  assert chestnut_input(200.0).tolist() == [[0.0, 0.0]]
  assert chestnut_input(30.0).tolist() == [[1.0, 0.25]]


# Scan rate: look hard only where a stop can matter (2026-11-19, the finder ran at its 40 % cap everywhere).
from openpilot.starpilot.stop_line import stop_lined as _sl  # noqa: E402


def _rate(speed=15.0, model_stop=None, ours=False, seen=0.0, accel=0.0, full=0.3):
  return _sl.scan_interval(speed, model_stop, ours, seen, accel, full)


def test_scan_pauses_while_stopped():
  assert _rate(speed=0.0) is None
  assert _rate(speed=0.3, model_stop=20.0) is None


def test_scan_runs_full_rate_near_a_possible_stop():
  assert _rate(model_stop=120.0) == 0.3        # the driving model plans a stop within 150 m
  assert _rate(ours=True) == 0.3               # our own estimate is tracking a stop
  assert _rate(seen=0.5) == 0.3                # our latest frame saw something stop-like
  assert _rate(accel=-1.2) == 0.3              # braking like an approach
  assert _rate(speed=31.0, model_stop=90.0) == 0.3  # on the highway too


def test_scan_slows_when_nothing_is_near():
  assert _rate(speed=15.0) == _sl.CRUISE_INTERVAL_S
  assert _rate(speed=31.0) == _sl.HIGHWAY_INTERVAL_S
  assert _rate(model_stop=400.0) == _sl.CRUISE_INTERVAL_S


def test_scan_never_beats_the_busy_cap():
  assert _rate(model_stop=50.0, full=0.6) == 0.6
  assert _rate(speed=15.0, full=1.5) == 1.5


def test_scan_handles_bad_inputs():
  assert _rate(speed=float('nan')) is None
  assert _rate(model_stop=float('nan')) == _sl.CRUISE_INTERVAL_S
  assert _rate(accel=float('nan')) == _sl.CRUISE_INTERVAL_S
