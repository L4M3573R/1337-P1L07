import unittest

from openpilot.starpilot.models.step_timing import STEPS, StepTiming


class TestStepTiming(unittest.TestCase):
  def setUp(self):
    self.now = 0.0
    self.reports: list[dict] = []
    self.timing = StepTiming(lambda **packet: self.reports.append(packet), period_s=30.0, clock=lambda: self.now)

  def frame(self, ms=(1, 2, 3, 20, 4), total_ms=33.0, model="lightspeed2", chestnut=True, timed=True):
    self.timing.add(tuple(int(m * 1e6) for m in ms) if timed else None, total_ms / 1e3, model, chestnut)

  def test_one_report_per_period_with_percentiles_in_ms(self):
    for k in range(100):
      self.now = k * 0.05
      self.frame(ms=(1, 2, 3, 20 + (k == 99) * 30, 4), total_ms=30 + k * 0.1)
    self.assertEqual(self.reports, [])   # 5 s in: nothing logged per frame
    self.now = 30.0
    self.frame()
    self.assertEqual(len(self.reports), 1)
    r = self.reports[0]
    self.assertEqual((r["modelId"], r["chestnut"], r["frames"], r["timedFrames"]), ("lightspeed2", True, 101, 101))
    self.assertEqual(r["wait"], {"p50": 20.0, "p90": 20.0, "max": 50.0})
    self.assertEqual(r["prepare"]["p50"], 1.0)
    self.assertEqual(set(STEPS) <= r.keys(), True)
    self.assertAlmostEqual(r["total"]["max"], 39.9)
    self.now = 31.0
    self.frame()
    self.assertEqual(len(self.reports), 1)   # the next window has only started

  def test_model_change_closes_the_window(self):
    self.frame()
    self.frame(model="small", chestnut=False, total_ms=12.0)
    self.assertEqual(len(self.reports), 1)
    self.assertEqual((self.reports[0]["modelId"], self.reports[0]["frames"]), ("lightspeed2", 1))
    self.timing.flush()
    self.assertEqual((self.reports[1]["modelId"], self.reports[1]["chestnut"], self.reports[1]["total"]["p50"]), ("small", False, 12.0))

  def test_frames_without_step_times_count_in_total_only(self):
    self.frame(timed=False)
    self.frame(ms=(1, 2, 3))   # wrong length: ignored, never raises in the model loop
    self.timing.flush()
    r = self.reports[0]
    self.assertEqual((r["frames"], r["timedFrames"]), (2, 0))
    self.assertNotIn("wait", r)

  def test_empty_flush_reports_nothing(self):
    self.timing.flush()
    self.assertEqual(self.reports, [])


if __name__ == "__main__":
  unittest.main()
