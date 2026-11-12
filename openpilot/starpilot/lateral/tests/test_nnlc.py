import hashlib
import json
import math
import os
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import log
from opendbc.car.car_helpers import interfaces
from opendbc.car.hyundai.values import CAR
from opendbc.car.structs import car
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.starpilot.lateral.nnlc import trial
from openpilot.starpilot.lateral.nnlc.feedforward import MAX_DEVIATION, NnlcFeedforward
from openpilot.starpilot.lateral.nnlc.model import INPUT_SIZE, ModelError, NnlcModel


def model_doc(scale=0.3, bias=0.0, hidden=4, seed=0):
  """A small valid network whose output is roughly scale * lateral accel (input 1)."""
  rng = np.random.default_rng(seed)
  W1 = rng.normal(0, 0.01, (INPUT_SIZE, hidden))
  W1[1, :] = 1.0
  W2 = np.full((hidden, 1), scale / (hidden * 0.25))
  return {
    'input_size': INPUT_SIZE, 'output_size': 1,
    'input_mean': [0.0] * INPUT_SIZE, 'input_std': [1.0] * INPUT_SIZE,
    'layers': [
      {'dense_0_W': W1.T.tolist(), 'dense_0_b': [0.0] * hidden, 'activation': 'σ'},
      {'dense_1_W': W2.T.tolist(), 'dense_1_b': [bias - scale * 2.0], 'activation': 'identity'},
    ],
  }


PLAN_LEN = len(ModelConstants.T_IDXS)


def plan(acc=0.0, n=PLAN_LEN):
  return SimpleNamespace(acceleration=SimpleNamespace(y=[acc] * n), orientation=SimpleNamespace(x=[0.0] * n))


class TestModel:
  def test_evaluates_and_is_monotonic_in_lateral_accel(self):
    m = NnlcModel(model_doc())
    lo, hi = m.evaluate([20, -1] + [0] * 16), m.evaluate([20, 1] + [0] * 16)
    assert math.isfinite(lo) and math.isfinite(hi) and hi > lo

  @pytest.mark.parametrize('mutate', [
    lambda d: d.update(input_size=17),
    lambda d: d.update(output_size=2),
    lambda d: d['input_std'].__setitem__(3, 0.0),
    lambda d: d['input_mean'].__setitem__(0, float('nan')),
    lambda d: d['layers'][0].update(activation='relu'),
    lambda d: d['layers'][1].update(activation='σ'),
    lambda d: d['layers'][0].update(dense_0_b=[0.0]),
    lambda d: d.update(layers=[]),
  ])
  def test_rejects_malformed_files(self, mutate):
    d = model_doc()
    mutate(d)
    with pytest.raises(ModelError):
      NnlcModel(d)

  def test_load_reports_checksum(self, tmp_path):
    p = tmp_path / 'm.json'
    p.write_text(json.dumps(model_doc()))
    _, sha = NnlcModel.load(p)
    assert sha == hashlib.sha256(p.read_bytes()).hexdigest()


class TestFeedforward:
  def make(self, **kw):
    ff = NnlcFeedforward(NnlcModel(model_doc(**kw)), DT_CTRL)
    for _ in range(40):
      ff.observe(0.0, 0.0)
    return ff

  def test_none_without_plan_or_with_bad_plan(self):
    ff = self.make()
    assert ff.torque(20, 0, 0.5, 0, 0, 0.2, 0.1) is None
    ff.set_plan(plan(n=5))
    assert ff.torque(20, 0, 0.5, 0, 0, 0.2, 0.1) is None
    ff.set_plan(plan())
    assert ff.torque(20, 0, 0.5, 0, 0, 0.2, math.nan) is None
    assert ff.torque(20, 0, math.nan, 0, 0, 0.2, 0.1) is None

  def test_bounded_near_baseline_and_full_scale(self):
    ff = self.make(scale=5.0)  # a deliberately wild network
    ff.set_plan(plan(2.0))
    out = ff.torque(20, 0, 2.0, 0, 0, 0.2, 0.1)
    assert out == pytest.approx(0.1 + MAX_DEVIATION)
    out = ff.torque(20, 0, 2.0, 0, 0, 0.2, 0.95)
    assert out == pytest.approx(1.0)
    ff.set_plan(plan(-2.0))
    assert ff.torque(20, 0, -2.0, 0, 0, 0.2, -0.1) == pytest.approx(-0.1 - MAX_DEVIATION)

  def test_never_weaker_than_baseline_in_tight_curves(self):
    ff = self.make()
    ff.model = SimpleNamespace(evaluate=lambda x: 0.2)  # a flattened network
    ff.set_plan(plan(2.5))
    assert ff.torque(20, 0, 2.5, 0, 0, 0.2, 0.45) == pytest.approx(0.45)   # fully back to the baseline
    assert ff.torque(20, 0, 1.85, 0, 0, 0.2, 0.45) == pytest.approx(0.325)  # halfway
    assert ff.torque(20, 0, 1.0, 0, 0, 0.2, 0.45) == pytest.approx(0.2)    # moderate curve: network as is
    assert ff.torque(20, 0, 2.5, 0, 0, 0.2, 0.1) == pytest.approx(0.2)     # stronger than baseline: kept
    ff.model = SimpleNamespace(evaluate=lambda x: -0.2)
    assert ff.torque(20, 0, -2.5, 0, 0, 0.2, -0.45) == pytest.approx(-0.45)  # mirrored for left curves

  def test_past_inputs_come_from_history(self):
    seen = []
    ff = self.make()
    ff.model = SimpleNamespace(evaluate=lambda x: seen.append(list(x)) or 0.0)
    for i in range(30):
      ff.observe(float(i), 0.0)
    ff.set_plan(plan())
    ff.torque(20, 0, 29.0, 0, 0, 0.2, 0.0)
    assert seen[0][4:7] == [0.0, 10.0, 20.0]  # 0.3, 0.2, 0.1 s ago at 100 Hz


class TestTrial:
  def write(self, root, mode='trial', doc=None, sha=None):
    (root / 'models').mkdir(parents=True, exist_ok=True)
    p = root / 'models' / 'personal.json'
    p.write_text(json.dumps(doc or model_doc()))
    real = hashlib.sha256(p.read_bytes()).hexdigest()
    (root / 'config.json').write_text(json.dumps({'version': 1, 'mode': mode, 'model': 'models/personal.json',
                                                  'model_sha256': sha or real}))
    return real

  def test_missing_or_off_is_standard(self, tmp_path):
    assert trial.choose_for_drive(tmp_path).mode == trial.STANDARD
    self.write(tmp_path, mode='off')
    assert trial.choose_for_drive(tmp_path).mode == trial.STANDARD

  def test_trial_alternates_starting_with_nnlc(self, tmp_path):
    self.write(tmp_path)
    modes = [trial.choose_for_drive(tmp_path).mode for _ in range(4)]
    assert modes == [trial.NNLC, trial.STANDARD, trial.NNLC, trial.STANDARD]

  def test_new_model_starts_its_trial_on_nnlc(self, tmp_path):
    self.write(tmp_path)
    assert trial.choose_for_drive(tmp_path).mode == trial.NNLC
    self.write(tmp_path, doc=model_doc(seed=3))
    assert trial.choose_for_drive(tmp_path).mode == trial.NNLC

  def test_on_always_nnlc(self, tmp_path):
    self.write(tmp_path, mode='on')
    assert {trial.choose_for_drive(tmp_path).mode for _ in range(3)} == {trial.NNLC}

  def test_checksum_mismatch_or_bad_reference_is_standard(self, tmp_path):
    self.write(tmp_path, sha='0' * 64)
    assert trial.choose_for_drive(tmp_path).reason == 'model checksum mismatch'
    (tmp_path / 'config.json').write_text(json.dumps({'version': 1, 'mode': 'on', 'model': '../x.json', 'model_sha256': '0' * 64}))
    assert trial.choose_for_drive(tmp_path).reason == 'bad model reference'

  def test_driver_override_wins_and_leaves_the_trial_alone(self, tmp_path):
    self.write(tmp_path)
    assert trial.choose_for_drive(tmp_path).mode == trial.NNLC                     # trial drive 1
    (tmp_path / 'user.json').write_text(json.dumps({'version': 1, 'mode': 'off'}))
    assert trial.choose_for_drive(tmp_path).reason == 'driver chose off'
    (tmp_path / 'user.json').write_text(json.dumps({'version': 1, 'mode': 'on'}))
    assert {trial.choose_for_drive(tmp_path).mode for _ in range(3)} == {trial.NNLC}
    (tmp_path / 'user.json').write_text(json.dumps({'version': 1, 'mode': 'auto'}))
    assert trial.choose_for_drive(tmp_path).mode == trial.STANDARD                 # the trial resumes at drive 2
    self.write(tmp_path, mode='off')
    (tmp_path / 'user.json').write_text(json.dumps({'version': 1, 'mode': 'on'}))
    assert trial.choose_for_drive(tmp_path).mode == trial.NNLC                     # on overrides a trainer "off"
    self.write(tmp_path, sha='0' * 64)
    assert trial.choose_for_drive(tmp_path).mode == trial.STANDARD                 # but never a bad model

  @pytest.mark.skipif(os.geteuid() == 0, reason='root ignores directory permissions')
  def test_unwritable_state_is_standard(self, tmp_path):
    self.write(tmp_path)
    os.chmod(tmp_path, 0o500)
    try:
      assert trial.choose_for_drive(tmp_path).mode == trial.STANDARD
    finally:
      os.chmod(tmp_path, 0o700)


class TestController:
  def setup(self):
    CarInterface = interfaces[CAR.HYUNDAI_ELANTRA_HEV_2024]
    CP = CarInterface.get_non_essential_params(CAR.HYUNDAI_ELANTRA_HEV_2024)
    self.CI, self.VM = CarInterface(CP), VehicleModel(CP)
    self.lac = LatControlTorque(CP.as_reader(), self.CI, DT_CTRL)
    self.CS = car.CarState.new_message(vEgo=25.0)
    self.params = log.VehicleParameters.new_message()

  def run(self, active=True, curvature=0.002, n=200):
    out = None
    for _ in range(n):
      out = self.lac.update(active, self.CS, self.VM, self.params, False, curvature, False, 0.2)
    return out

  def test_nnlc_replaces_only_the_feedforward(self):
    self.setup()
    _, _, base = self.run()
    self.setup()
    ff = NnlcFeedforward(NnlcModel(model_doc()), DT_CTRL)
    ff.set_plan(plan(0.002 * 25.0 ** 2))
    self.lac.nnlc = ff
    _, _, log_nn = self.run()
    assert ff.last_used
    assert log_nn.f == pytest.approx(self.lac.lateral_accel_from_torque(ff.last_torque, self.lac.torque_params), abs=1e-4)
    assert log_nn.f != pytest.approx(base.f, abs=1e-4)

  def test_inactive_outputs_nothing_but_keeps_history(self):
    self.setup()
    ff = NnlcFeedforward(NnlcModel(model_doc()), DT_CTRL)
    ff.set_plan(plan())
    self.lac.nnlc = ff
    steer, _, _ = self.run(active=False, n=50)
    assert steer == 0.0 and not ff.last_used and len(ff.lat_accel_history) == ff.history_len

  def test_falls_back_when_network_cannot_run(self):
    self.setup()
    _, _, base = self.run()
    self.setup()
    self.lac.nnlc = NnlcFeedforward(NnlcModel(model_doc()), DT_CTRL)  # no plan: falls back every tick
    _, _, log_nn = self.run()
    assert log_nn.f == pytest.approx(base.f, abs=1e-6)
