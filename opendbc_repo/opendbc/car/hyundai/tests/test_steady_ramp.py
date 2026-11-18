"""The Elantra Hybrid's fast torque ramp (10 up / 9 down) is only for a demand that keeps going one way: after the
torque reverses, the ramp is the old 3 / 7 for half a second, so steering that hunts back and forth gets exactly the
old ramp (first drive on the flat fast ramp, 2026-11-18: 5 deg of wheel shake at 8-25 mph against 2 deg before)."""
import json
from pathlib import Path
from types import SimpleNamespace as NS

from opendbc.car.hyundai import steady_ramp as sr
from opendbc.car.hyundai.steady_ramp import SteadyRamp
from opendbc.car.lateral import apply_driver_steer_torque_limits


def limits(up=10, down=9):
  return NS(STEER_MAX=400, STEER_DELTA_UP=up, STEER_DELTA_DOWN=down, STEER_DRIVER_ALLOWANCE=50, STEER_DRIVER_MULTIPLIER=2,
            STEER_DRIVER_FACTOR=1, STEER_THRESHOLD=150)


def drive(wanted, ramp=None, flat=None):
  """The torque sent for each asked torque; a ramp carries on from where its last call ended."""
  out, last = [], getattr(ramp, 'test_last', 0)
  for want in wanted:
    last = apply_driver_steer_torque_limits(want, last, 0.0, ramp.limits() if ramp is not None else flat)
    if ramp is not None:
      ramp.update(last)
      ramp.test_last = last
    out.append(last)
  return out


def test_a_demand_from_rest_gets_the_full_ramp_either_way():
  for sign in (1, -1):
    out = drive([sign * 400] * 45, SteadyRamp(limits()))
    assert abs(out[2]) >= 29 and out[40] == sign * 400     # 0.4 s, as on the flat fast ramp
    assert out == drive([sign * 400] * 45, flat=limits())


def test_after_a_reversal_the_ramp_is_the_old_one_then_grows_back():
  ramp = SteadyRamp(limits())
  drive([400] * 40, ramp)
  drive([380] * 5, ramp)                                  # comes back 20: a reversal
  assert ramp.steps() == (3, 7)
  drive([380] * (sr.HOLD_FRAMES - 5), ramp)
  assert ramp.steps() == (3, 7)                           # still the old ramp half a second on
  drive([380] * (sr.GROW_FRAMES + 5), ramp)                # the reversal is seen a couple of frames after it starts
  assert ramp.steps() == (10, 9)
  mid = SteadyRamp(limits())
  drive([400] * 40 + [380] * (sr.HOLD_FRAMES + sr.GROW_FRAMES // 2), mid)
  assert (3, 7) < mid.steps() < (10, 9)


def test_small_ripple_is_not_a_reversal():
  ramp = SteadyRamp(limits())
  drive([100] * 20 + [100 - sr.REVERSAL + 1, 100] * 30, ramp)
  assert ramp.steps() == (10, 9)


def every_step_is_an_old_ramp_step(wanted, out, start):
  return all(out[n] == apply_driver_steer_torque_limits(wanted[n], out[n - 1], 0.0, limits(3, 7)) for n in range(start, len(out)))


def test_hunting_at_two_hertz_gets_exactly_the_old_ramp():
  # what the first drive did after a turn: the asked torque swinging +-200 every quarter second
  wanted = ([200] * 25 + [-200] * 25) * 8
  steady, old, fast = drive(wanted, SteadyRamp(limits())), drive(wanted, flat=limits(3, 7)), drive(wanted, flat=limits())
  swing = lambda out: max(out[200:]) - min(out[200:])
  assert swing(fast) > 2 * swing(old)
  # the first swing from rest is free; a few frames into the first swing back, every step is an old-ramp step for good
  assert not every_step_is_an_old_ramp_step(wanted, steady, 1)
  assert every_step_is_an_old_ramp_step(wanted, steady, 30) and swing(steady) <= swing(old) + 2


def test_hunting_a_little_above_one_hertz_is_still_held_to_the_old_ramp():
  wanted = ([300] * 45 + [-300] * 45) * 5                 # 1.1 Hz
  assert every_step_is_an_old_ramp_step(wanted, drive(wanted, SteadyRamp(limits())), 50)


def test_slow_swings_do_get_the_fast_ramp_back():
  wanted = ([300] * 150 + [-300] * 150) * 2               # a third of a hertz: a turn one way, then the other
  steady = drive(wanted, SteadyRamp(limits()))
  assert not every_step_is_an_old_ramp_step(wanted, steady, 160)
  assert max(abs(b - a) for a, b in zip(steady[160:], steady[161:])) == 10


def test_it_never_allows_more_than_it_was_given_and_passes_the_rest_through():
  ramp = SteadyRamp(limits(2, 3))                         # a car with smaller limits than the base ramp
  assert ramp.steps() == (2, 3)
  drive([400] * 30 + [0] * 30, ramp)
  assert ramp.steps() == (2, 3)
  view = SteadyRamp(limits()).limits()
  assert (view.STEER_MAX, view.STEER_DRIVER_ALLOWANCE, view.STEER_DRIVER_MULTIPLIER, view.STEER_DRIVER_FACTOR) == (400, 50, 2, 1)
  import random
  rng, ramp, last = random.Random(3), SteadyRamp(limits()), 0
  for _ in range(4000):
    tq = apply_driver_steer_torque_limits(rng.randint(-400, 400), last, 0.0, ramp.limits())
    step = tq - last
    rising = abs(tq) > abs(last) or tq * last < 0
    assert abs(step) <= (10 if rising else 9) + (9 if tq * last < 0 else 0) and abs(tq) <= 400
    ramp.update(tq)
    last = tq


def test_the_file_switches_the_ramp_and_bad_values_mean_the_default(tmp_path):
  path = tmp_path / 'steer_ramp.json'
  default = sr.from_file(limits(), str(path))             # no file
  assert isinstance(default, SteadyRamp) and (default.hold_frames, default.grow_frames) == (sr.HOLD_FRAMES, sr.GROW_FRAMES)
  path.write_text(json.dumps({'ramp': 'fast'}))
  assert sr.from_file(limits(), str(path)) is None        # the flat 10 / 9
  path.write_text(json.dumps({'ramp': 'slow'}))
  slow = sr.from_file(limits(), str(path))
  assert drive([400] * 50, slow) == drive([400] * 50, flat=limits(3, 7))
  path.write_text(json.dumps({'ramp': 'steady', 'hold_frames': 30, 'grow_frames': 10}))
  tuned = sr.from_file(limits(), str(path))
  assert (tuned.hold_frames, tuned.grow_frames) == (30, 10)
  for bad in ('{"ramp": "ste', '[1]', json.dumps({'hold_frames': -5, 'grow_frames': 'x'}), json.dumps({'hold_frames': 9000, 'grow_frames': True})):
    path.write_text(bad)
    ramp = sr.from_file(limits(), str(path))
    assert (ramp.hold_frames, ramp.grow_frames) == (sr.HOLD_FRAMES, sr.GROW_FRAMES)


def test_only_the_elantra_hybrid_with_its_raised_limits_gets_it():
  src = (Path(__file__).resolve().parents[1] / 'carcontroller.py').read_text()
  assert 'self.steady_ramp = steady_ramp.from_file(self.params) if elantra_hev_2024_eps(CP) else None' in src
  assert 'torque_limits = self.params if self.steady_ramp is None else self.steady_ramp.limits()' in src
  assert 'apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, torque_limits)' in src
