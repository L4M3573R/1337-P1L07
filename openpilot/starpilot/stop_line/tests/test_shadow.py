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
