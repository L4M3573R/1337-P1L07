#!/usr/bin/env python3
"""Accuracy table for a cross-validation run of the stop-line finder: ours against chestnut on the same frames.

  python openpilot/starpilot/stop_line/report.py <data_dir> <run_dir>

Reads <data_dir>/index.npz and the run's fold*.npz predictions (train.py). "Accurate" is: says a stop is coming and
puts it within 5 m of the stop point, out of every approach frame in that range; chestnut counts only when its plan
reaches a stop. Rows split lights and signs and each kind of stop; the last line is false alarms on no-stop frames.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

LABEL_RANGE_M = 120.0
RANGES = ((0, 30), (30, 60), (60, 121), (0, 121))


def predictions(data_dir: Path, run_dir: Path):
  ix = dict(np.load(data_dir / 'index.npz'))
  folds = sorted(run_dir.glob('fold*.npz'))
  out = np.concatenate([np.load(f)['out'] for f in folds])
  keys = np.concatenate([np.load(f)['keys'] for f in folds])
  return ix, folds, out, keys


def gate_metrics(data_dir: Path, run_dir: Path) -> dict:
  """What the nightly gate compares: first-in-line stops placed within 5 m by chestnut-first with ours filling in (what
  stop_lined chooses), the same for ours alone and chestnut alone, and ours' false alarms on no-stop frames."""
  ix, _, out, keys = predictions(data_dir, run_dir)
  p = 1 / (1 + np.exp(-out[:, 0]))
  ours = np.where(p > 0.5, out[:, 1] * LABEL_RANGE_M, np.nan)
  dist, chestnut, target = ix['dist'][keys], ix['model_stop'][keys], ix['target'][keys]
  kind = ix['kind'][keys] if 'kind' in ix else np.full(keys.size, 'full')
  chestnut = np.where(np.isfinite(chestnut) & (chestnut <= LABEL_RANGE_M), chestnut, np.nan)
  hybrid = np.where(np.isfinite(chestnut), chestnut, ours)
  first = (target == 1) & (kind == 'full')
  ok = lambda x: float(np.mean((np.isfinite(x) & (np.abs(x - dist) < 5))[first]))
  return dict(hybrid_within5=ok(hybrid), ours_within5=ok(ours), chestnut_within5=ok(chestnut),
              false_alarms=float(np.mean(p[target == 0] > 0.5)), first_in_line_frames=int(first.sum()),
              frames=int(keys.size))


def main() -> None:
  data_dir, run_dir = Path(sys.argv[1]), Path(sys.argv[2])
  ix, folds, out, keys = predictions(data_dir, run_dir)
  p = 1 / (1 + np.exp(-out[:, 0]))
  ours = out[:, 1] * LABEL_RANGE_M
  target, dist, chestnut = ix['target'][keys], ix['dist'][keys], ix['model_stop'][keys]
  tag, kind = ix['tag'][keys], ix['kind'][keys] if 'kind' in ix else np.full(keys.size, 'full')
  ours_ok = (p > 0.5) & (np.abs(ours - dist) < 5)
  chestnut_ok = np.isfinite(chestnut) & (np.abs(chestnut - dist) < 5)
  print(f'{run_dir.name}: {len(folds)} folds, within 5 m as ours / chestnut')
  print(f"{'':16}{'frames':>8}" + ''.join(f'{f"{lo}-{min(hi, 120)} m":>14}' for lo, hi in RANGES[:-1]) + f"{'all':>14}")
  pos = target == 1
  rows = [('lights', tag == 'light'), ('signs', tag == 'sign')] + \
         [(f'  {k} stops', kind == k) for k in ('full', 'queue', 'rolling') if np.any(pos & (kind == k))] + [('everything', pos)]
  for name, m in rows:
    m = m & pos
    cells = []
    for lo, hi in RANGES:
      mm = m & (dist >= lo) & (dist < hi)
      cells.append(f'{ours_ok[mm].mean():.0%} / {chestnut_ok[mm].mean():.0%}' if mm.any() else '-')
    print(f'{name:16}{int(m.sum()):>8}' + ''.join(f'{c:>14}' for c in cells))
  neg = target == 0
  print(f'false alarms on no-stop frames: ours {np.mean(p[neg] > 0.5):.1%}, '
        f'chestnut {np.mean(np.isfinite(chestnut[neg]) & (chestnut[neg] < LABEL_RANGE_M)):.1%}')
  g = gate_metrics(data_dir, run_dir)
  print(f"first-in-line within 5 m: chestnut-first with ours filling in {g['hybrid_within5']:.0%}, "
        f"ours {g['ours_within5']:.0%}, chestnut {g['chestnut_within5']:.0%}")


if __name__ == '__main__':
  main()
