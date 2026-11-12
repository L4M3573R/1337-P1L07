"""Small dense network evaluator for NNLC model files.

File format (JSON, compatible with the common NNLC layout): input_size, output_size, input_mean,
input_std and layers, each layer holding one weight matrix stored output-major (key ending in `_W`),
one bias vector (key ending in `_b`) and an activation ('σ'/'sigmoid' or 'identity').
"""
from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Sequence
from pathlib import Path

import numpy as np

INPUT_SIZE = 18
MAX_FILE_BYTES = 1_000_000
MAX_UNITS = 256
ACTIVATIONS = {'σ': 'sigmoid', 'sigmoid': 'sigmoid', 'identity': 'identity'}


class ModelError(ValueError):
  pass


def _array(value, name: str) -> np.ndarray:
  try:
    arr = np.asarray(value, dtype=np.float64)
  except (TypeError, ValueError) as e:
    raise ModelError(f'{name} is not numeric') from e
  if not np.all(np.isfinite(arr)):
    raise ModelError(f'{name} is not finite')
  return arr


class NnlcModel:
  def __init__(self, doc: dict):
    if not isinstance(doc, dict):
      raise ModelError('model document must be an object')
    if doc.get('input_size') != INPUT_SIZE or doc.get('output_size') != 1:
      raise ModelError('unexpected input or output size')
    self.mean = _array(doc.get('input_mean'), 'input_mean').reshape(-1)
    self.std = _array(doc.get('input_std'), 'input_std').reshape(-1)
    if self.mean.shape != (INPUT_SIZE,) or self.std.shape != (INPUT_SIZE,) or np.any(self.std <= 0):
      raise ModelError('bad input normalization')
    layers = doc.get('layers')
    if not isinstance(layers, list) or not 1 <= len(layers) <= 6:
      raise ModelError('bad layer list')
    self.layers: list[tuple[np.ndarray, np.ndarray, str]] = []
    width = INPUT_SIZE
    for i, layer in enumerate(layers):
      if not isinstance(layer, dict):
        raise ModelError(f'layer {i} must be an object')
      w_keys = [k for k in layer if k.endswith('_W')]
      b_keys = [k for k in layer if k.endswith('_b')]
      if len(w_keys) != 1 or len(b_keys) != 1:
        raise ModelError(f'layer {i} needs one weight and one bias')
      W = _array(layer[w_keys[0]], f'layer {i} weights').T
      b = _array(layer[b_keys[0]], f'layer {i} bias').reshape(-1)
      activation = ACTIVATIONS.get(layer.get('activation'))
      if W.ndim != 2 or W.shape[0] != width or b.shape != (W.shape[1],) or W.shape[1] > MAX_UNITS or activation is None:
        raise ModelError(f'layer {i} has a bad shape or activation')
      self.layers.append((W, b, activation))
      width = W.shape[1]
    if width != 1 or self.layers[-1][2] != 'identity':
      raise ModelError('the last layer must be a single identity output')

  @classmethod
  def load(cls, path: Path) -> tuple[NnlcModel, str]:
    """Returns the model and the SHA-256 of the exact bytes loaded."""
    raw = Path(path).read_bytes()
    if len(raw) > MAX_FILE_BYTES:
      raise ModelError('model file too large')
    try:
      doc = json.loads(raw)
    except (UnicodeDecodeError, json.JSONDecodeError) as e:
      raise ModelError('model file is not JSON') from e
    return cls(doc), hashlib.sha256(raw).hexdigest()

  def evaluate(self, x: Sequence[float]) -> float:
    v = (np.asarray(x, dtype=np.float64) - self.mean) / self.std
    for W, b, activation in self.layers:
      v = v @ W + b
      if activation == 'sigmoid':
        v = 1.0 / (1.0 + np.exp(-np.clip(v, -50.0, 50.0)))
    out = float(v[0])
    return out if math.isfinite(out) else math.nan
