"""Lane centering v2: keeps the lane through brief doubt, and its shadow never touches what steers."""
import json


from openpilot.starpilot.lateral.lane_centering import LaneCenteringController, LaneCenteringSettings
from openpilot.starpilot.lateral.lane_centering_v2 import HOLD_S, LaneCenteringControllerV2, LaneShadow, read_mode
from openpilot.starpilot.lateral.tests.test_lane_centering import model, observation


def drive(controller, frames):
  """frames: [(model, seconds)] fed at 100 Hz with a new model id every 5 ticks (20 Hz)."""
  results, sample = [], 0
  for m, seconds in frames:
    for tick in range(int(round(seconds / 0.01))):
      if tick % 5 == 0:
        sample += 1
      results.append(controller.update(observation(m, model_sample_id=sample)))
  return results


def test_a_trusted_line_that_dips_under_60_percent_is_kept():
  strong, dipped = model(right=2.3, prob=0.9), model(right=2.3, prob=0.5)
  cur, v2 = drive(LaneCenteringController(), [(strong, 2.0), (dipped, 1.0)]), drive(LaneCenteringControllerV2(), [(strong, 2.0), (dipped, 1.0)])
  assert cur[-1].reason == 'unqualified_model'
  assert v2[-1].reason == 'qualified' and v2[-1].correction != 0.0


def test_a_line_never_trusted_still_needs_60_percent():
  v2 = drive(LaneCenteringControllerV2(), [(model(right=2.3, prob=0.5), 2.0)])
  assert v2[-1].reason == 'unqualified_model'


def test_both_lines_gone_for_a_moment_it_keeps_the_lane_then_lets_go():
  strong, gone = model(right=2.3, prob=0.9), model(right=2.3, prob=0.05)
  v2 = drive(LaneCenteringControllerV2(), [(strong, 2.0), (gone, 0.5)])
  assert v2[-1].reason == 'qualified' and v2[-1].correction != 0.0   # held from memory
  v2 = drive(LaneCenteringControllerV2(), [(strong, 2.0), (gone, HOLD_S + 0.5)])
  assert v2[-1].reason == 'unqualified_model'                           # and lets go once the memory is spent


def test_it_pushes_the_same_way_as_the_current_controller():
  off_center = model(left=-1.2, right=2.4)   # the lane center is to the right of the path
  cur = drive(LaneCenteringController(), [(off_center, 3.0)])[-1].correction
  v2 = drive(LaneCenteringControllerV2(), [(off_center, 3.0)])[-1].correction
  assert cur > 0 and v2 > 0


def test_the_shadow_never_changes_what_steers_and_logs_a_summary():
  logged = []
  shadow = LaneShadow(LaneCenteringControllerV2(), 'v2', 'current', log=logged.append)
  steering = LaneCenteringController()
  sample = 0
  for m, seconds in ((model(right=2.3), 30.0), (model(right=2.3, prob=0.5), 0.5), (model(right=2.3), 31.0)):
    for tick in range(int(round(seconds / 0.01))):
      if tick % 5 == 0:
        sample += 1
      obs = observation(m, model_sample_id=sample)
      result = steering.update(obs)
      before = (result.candidate_curvature, result.correction, result.reason)
      shadow.observe(obs, result)
      assert (result.candidate_curvature, result.correction, result.reason) == before
  assert logged, 'a summary once a minute'
  doc = logged[0]
  assert doc['current_dropouts'] >= 1 and doc['v2_dropouts'] == 0   # the dip: the current one let go, v2 held
  assert doc['feature'] == 'lane_centering_shadow' and doc['minutes'] >= 1.0


def test_a_broken_shadow_is_silent():
  class Broken(LaneCenteringControllerV2):
    def update(self, observation):
      raise RuntimeError('boom')
  shadow = LaneShadow(Broken(), 'v2', 'current', log=lambda d: None)
  steering = LaneCenteringController()
  obs = observation(model())
  shadow.observe(obs, steering.update(obs))   # no exception


def test_the_mode_file(tmp_path):
  path = tmp_path / 'lane_centering_v2.json'
  assert read_mode(path) == 'on'   # v2 steers by default (the driver, 2026-11-17)
  for mode in ('on', 'off', 'shadow'):
    path.write_text(json.dumps({'mode': mode}))
    assert read_mode(path) == mode
  path.write_text('{"mode": "bogus"}')
  assert read_mode(path) == 'on'


def test_it_never_pushes_toward_a_line_the_body_is_already_close_to():
  # lens 0.23 m left of the car center, car 1.83 m wide: a right line 1.30 m right of the lens leaves 0.155 m of room;
  # the path sits left of the lane center, so the plain reading would push right, toward that close line
  close_right = model(left=-2.1, right=1.30, path=-0.6)
  cur = drive(LaneCenteringController(), [(close_right, 3.0)])[-1]
  v2 = LaneCenteringControllerV2()
  results = []
  sample = 0
  for tick in range(300):
    if tick % 5 == 0:
      sample += 1
    results.append(v2.update(observation(close_right, model_sample_id=sample,
                                         settings=LaneCenteringSettings(True, -0.23, 0.0, True))))
  assert cur.correction > 0                     # the current controller pushes toward the close right line
  assert results[-1].correction <= 1e-9        # v2 does not


def test_away_from_a_close_line_it_still_pushes():
  close_right = model(left=-2.1, right=1.30, path=0.6)   # the path is right of center: push left, away from the line
  v2 = LaneCenteringControllerV2()
  sample, result = 0, None
  for tick in range(300):
    if tick % 5 == 0:
      sample += 1
    result = v2.update(observation(close_right, model_sample_id=sample, settings=LaneCenteringSettings(True, -0.23, 0.0, True)))
  assert result.correction < 0


def test_the_remembered_lane_never_survives_a_turn():
  # good lines, then a 6 s signalled turn with no lines, then the blinker off on a road with no lines:
  # the lane remembered before the turn belongs to the old road and must not steer the new one
  strong, gone = model(right=2.3, prob=0.9), model(right=2.3, prob=0.05)
  v2, sample, results = LaneCenteringControllerV2(), 0, []
  for m, seconds, blinker in ((strong, 2.0, False), (gone, 6.0, True), (gone, 0.5, False)):
    for tick in range(int(round(seconds / 0.01))):
      if tick % 5 == 0:
        sample += 1
      results.append(v2.update(observation(m, model_sample_id=sample, turn_signal_active=blinker)))
  assert results[-1].reason == 'unqualified_model' and abs(results[-1].correction) < 1e-6
  assert v2.preview.memory is None and v2.sticky.trusted == [False, False]


def test_a_faint_close_line_is_still_kept_off():
  close_right = model(left=-2.1, right=1.30, path=-0.6, prob=0.3)   # a faint right line, the body 0.155 m from it
  v2 = LaneCenteringControllerV2()
  assert v2._keep_off_lines(close_right, -0.23, 0.001) == 0.0
