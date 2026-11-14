"""The real longitudinal planner with fuel-experiment plans: a cap or a lower target only ever makes it gentler, and a
car ahead still wins."""
import unittest

from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.starpilot.efficiency.experiment import Plan
from openpilot.starpilot.longitudinal.tests.test_cruise_ceiling import V_EGO, messages

FRAMES = 60


class ExperimentPlannerTests(unittest.TestCase):
  def setUp(self):
    self.cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)

  def run_pair(self, plan, **conditions):
    plain, tested = LongitudinalPlanner(self.cp, init_v=V_EGO), LongitudinalPlanner(self.cp, init_v=V_EGO)
    out = []
    for _ in range(FRAMES):
      sm, _ = messages(**conditions)
      plain.update(sm)
      sm, _ = messages(**conditions)
      tested.update(sm, efficiency_plan=plan)
      out.append((float(plain.output_a_target), float(tested.output_a_target)))
    return out

  def test_accel_cap_limits_speeding_up(self):
    out = self.run_pair(Plan(accel_max=0.3))   # set speed 100 km/h, car at 20 m/s: it wants to speed up
    self.assertTrue(any(a > 0.35 for a, _ in out))
    self.assertTrue(all(b <= 0.3 + 1e-6 for _, b in out))

  def test_lower_target_never_more_aggressive(self):
    out = self.run_pair(Plan(speed_ceiling_mps=V_EGO))
    self.assertTrue(all(b <= a + 1e-6 for a, b in out))
    self.assertLess(out[-1][1], out[-1][0])

  def test_car_ahead_still_wins(self):
    out = self.run_pair(Plan(speed_ceiling_mps=V_EGO + 5.0, accel_max=1.0), lead=True)
    self.assertTrue(all(b <= a + 1e-6 for a, b in out))   # the lead's braking is kept

  def test_downhill_carry_is_at_most_2_mph(self):
    asked_too_much = self.run_pair(Plan(speed_bonus_mps=10.0))
    allowed = self.run_pair(Plan(speed_bonus_mps=2 * 0.44704))
    self.assertEqual([b for _, b in asked_too_much], [b for _, b in allowed])   # clipped to the 2 mph allowance
