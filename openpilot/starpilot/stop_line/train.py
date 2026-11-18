#!/usr/bin/env python3
"""Train the stop-line finder on the PC's GPU (5070 Ti) from frames.py's dataset, and export it for the comma.

  /mnt/Projects/stopline-train/.venv/bin/python openpilot/starpilot/stop_line/train.py <data_dir> <out_dir>
      [--folds 4] [--epochs 12] [--final]

One road-camera frame in (RGB, INPUT_H x INPUT_W), three numbers out: a stop-ahead logit (a light or sign stop
within LABEL_RANGE_M), the distance to the stop point as a fraction of LABEL_RANGE_M, and the log of that
distance's Laplace scale (how sure it is, used to fuse frames and to decide when to trust chestnut instead).
Validation holds out whole places (intersections), and every fold reports the same frames' error for chestnut's
planned stop next to ours. --final trains on everything and writes stop_line.onnx (opset 13, loads in cv2.dnn).
Needs only torch, torchvision, numpy and pillow; no openpilot imports.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from PIL import Image
from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler
from torchvision import models
from torchvision.transforms import v2 as T

INPUT_W, INPUT_H = 448, 256
LABEL_RANGE_M = 120.0
HIDE_CHESTNUT = 0.2  # corrector training hides chestnut's plan this often, so it still works without chestnut
MEAN = (0.485, 0.456, 0.406)
STD = (0.229, 0.224, 0.225)


class StopLineNet(nn.Module):
  """MobileNetV3-small trunk; takes RGB in 0..1 and normalizes inside so the comma feeds plain pixels."""

  def __init__(self, pretrained: bool = True, aux: int = 0):
    super().__init__()
    self.aux = aux  # extra inputs next to the image: chestnut's plan (has a stop, its distance fraction) for the corrector
    trunk = models.mobilenet_v3_small(weights=models.MobileNet_V3_Small_Weights.DEFAULT if pretrained else None)
    self.features = trunk.features
    self.register_buffer('mean', torch.tensor(MEAN).view(1, 3, 1, 1))
    self.register_buffer('std', torch.tensor(STD).view(1, 3, 1, 1))
    self.head = nn.Sequential(nn.Linear(576 + aux, 256), nn.ReLU(), nn.Dropout(0.3), nn.Linear(256, 3))

  def forward(self, x, aux=None):
    x = (x - self.mean) / self.std
    x = self.features(x)
    x = x.mean(dim=(2, 3))
    if self.aux:
      x = torch.cat((x, aux), dim=1)
    out = self.head(x)
    # logit, distance fraction (0..1), log scale of the distance fraction
    return torch.stack((out[:, 0], torch.sigmoid(out[:, 1]), out[:, 2].clamp(-6.0, 1.0)), dim=1)


class Frames(Dataset):
  def __init__(self, data_dir: Path, index: dict, rows: np.ndarray, train: bool):
    self.data_dir, self.index, self.rows, self.train = data_dir, index, rows, train
    self.aug = T.Compose([
      T.RandomResizedCrop((INPUT_H, INPUT_W), scale=(0.85, 1.0), ratio=(INPUT_W / INPUT_H * 0.95, INPUT_W / INPUT_H * 1.05)),
      T.ColorJitter(brightness=0.4, contrast=0.3, saturation=0.3, hue=0.03),
    ]) if train else None

  def __len__(self):
    return self.rows.size

  def __getitem__(self, i):
    k = self.rows[i]
    ix = self.index
    path = self.data_dir / 'frames' / f"{ix['route'][k]}--{ix['seg'][k]}" / f"{ix['frame'][k]}.jpg"
    img = torch.from_numpy(np.array(Image.open(path).convert('RGB'))).permute(2, 0, 1)
    if self.aug is not None:
      img = self.aug(img)
    img = img.float() / 255.0
    dist = ix['dist'][k] if ix['target'][k] == 1 else 0.0
    return img, chestnut_aux(ix['model_stop'][k], hide=self.train and np.random.rand() < HIDE_CHESTNUT), \
      torch.tensor(float(ix['target'][k])), torch.tensor(float(dist) / LABEL_RANGE_M), torch.tensor(int(k))


def chestnut_aux(model_stop: float, hide: bool = False) -> torch.Tensor:
  """Chestnut's planned stop as the corrector's extra input: (has one within range, its distance / LABEL_RANGE_M)."""
  has = not hide and math.isfinite(model_stop) and 0.0 <= model_stop <= LABEL_RANGE_M
  return torch.tensor([1.0 if has else 0.0, float(model_stop) / LABEL_RANGE_M if has else 0.0], dtype=torch.float32)


def loss_fn(out, target, dist):
  cls = F.binary_cross_entropy_with_logits(out[:, 0], target)
  pos = target > 0.5
  if pos.any():
    mu, log_b = out[pos, 1], out[pos, 2]
    reg = (torch.abs(dist[pos] - mu) / log_b.exp() + log_b).mean()  # Laplace negative log-likelihood
  else:
    reg = out.sum() * 0.0
  return cls + reg, cls, reg


def split(index: dict, fold: int, folds: int, seed: int = 0, by: str = 'place') -> tuple[np.ndarray, np.ndarray]:
  """by='place': whole intersections held out (how it does where it has never been). by='drive': whole drives held out,
  so other passes of the same intersection can be in training (how it does on roads driven before)."""
  rng = np.random.default_rng(seed)
  if by == 'drive':
    routes = np.unique(index['route'])
    route_fold = dict(zip(rng.permutation(routes), np.arange(routes.size) % folds, strict=True))
    fold_of = np.array([route_fold[r] for r in index['route']])
    return np.flatnonzero(fold_of != fold), np.flatnonzero(fold_of == fold)
  places = np.unique(index['place'][index['place'] >= 0])
  routes = np.unique(index['route'])
  place_fold = dict(zip(rng.permutation(places), np.arange(places.size) % folds, strict=True))
  route_fold = dict(zip(rng.permutation(routes), np.arange(routes.size) % folds, strict=True))
  fold_of = np.array([place_fold[p] if p >= 0 else route_fold[r] for p, r in zip(index['place'], index['route'], strict=True)])
  return np.flatnonzero(fold_of != fold), np.flatnonzero(fold_of == fold)


def loader(data_dir, index, rows, train, batch, workers=4):
  ds = Frames(data_dir, index, rows, train)
  if train:
    pos = index['target'][rows] == 1
    w = np.where(pos, 0.5 / max(pos.sum(), 1), 0.5 / max((~pos).sum(), 1))
    sampler = WeightedRandomSampler(torch.as_tensor(w, dtype=torch.double), num_samples=min(rows.size, 2 * 12000), replacement=True)
    return DataLoader(ds, batch_size=batch, sampler=sampler, num_workers=workers, pin_memory=True, drop_last=True,
                      persistent_workers=workers > 0, prefetch_factor=2 if workers else None)
  return DataLoader(ds, batch_size=batch, shuffle=False, num_workers=workers, pin_memory=True)


@torch.no_grad()
def predict(model, dl, device):
  model.eval()
  outs, keys = [], []
  for img, aux, _, _, k in dl:
    with torch.autocast('cuda', dtype=torch.bfloat16):
      out = model(img.to(device, non_blocking=True), aux.to(device))
    outs.append(out.float().cpu())
    keys.append(k)
  return torch.cat(outs).numpy(), torch.cat(keys).numpy()


def evaluate(out: np.ndarray, keys: np.ndarray, index: dict) -> dict:
  target = index['target'][keys]
  p = 1 / (1 + np.exp(-out[:, 0]))
  pos, neg = target == 1, target == 0
  res: dict = {'frames_pos': int(pos.sum()), 'frames_neg': int(neg.sum())}
  if pos.any() and neg.any():
    order = np.argsort(-p)
    t = target[order] == 1
    tp, fp = np.cumsum(t), np.cumsum(~t)
    res['auc'] = float(np.trapezoid(tp / t.sum(), fp / (~t).sum()))
    res['recall_at_p50'] = float((p[pos] > 0.5).mean())
    res['false_alarm_at_p50'] = float((p[neg] > 0.5).mean())
  if pos.any():
    d = index['dist'][keys][pos]
    ours = out[pos, 1] * LABEL_RANGE_M
    chestnut = index['model_stop'][keys][pos]
    for name, lo, hi in (('all', 0, 121), ('0-30m', 0, 30), ('30-60m', 30, 60), ('60-120m', 60, 121)):
      m = (d >= lo) & (d < hi)
      if not m.any():
        continue
      c = m & np.isfinite(chestnut)
      found = p[pos] > 0.5
      for tol in (5, 10):
        # accurate = says a stop is coming and puts it within tol metres, out of every approach frame in the range
        res[f'ours_within{tol}_{name}'] = float(np.mean((found & (np.abs(ours - d) < tol))[m]))
        res[f'chestnut_within{tol}_{name}'] = float(np.mean((np.isfinite(chestnut) & (np.abs(chestnut - d) < tol))[m]))
      res[f'ours_mae_{name}'] = float(np.median(np.abs(ours[m] - d[m])))
      res[f'chestnut_mae_{name}'] = float(np.median(np.abs(chestnut[c] - d[c]))) if c.any() else None
      res[f'chestnut_has_stop_{name}'] = float(c.sum() / m.sum())
  return res


def train_one(data_dir, index, tr, va, args, device, log):
  model = StopLineNet(aux=2 if args.corrector else 0).to(device).to(memory_format=torch.channels_last)
  opt = torch.optim.AdamW([{'params': model.features.parameters(), 'lr': args.lr * 0.3},
                           {'params': model.head.parameters(), 'lr': args.lr}], weight_decay=1e-4)
  dl = loader(data_dir, index, tr, True, args.batch, args.workers)
  steps = args.epochs * len(dl)
  sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=[args.lr * 0.3, args.lr], total_steps=steps, pct_start=0.15)
  for epoch in range(args.epochs):
    model.train()
    t0, tot = time.time(), np.zeros(3)
    for img, aux, target, dist, _ in dl:
      img = img.to(device, non_blocking=True).to(memory_format=torch.channels_last)
      target, dist = target.to(device), dist.to(device)
      with torch.autocast('cuda', dtype=torch.bfloat16):
        out = model(img, aux.to(device))
      loss, cls, reg = loss_fn(out.float(), target, dist)
      opt.zero_grad(set_to_none=True)
      loss.backward()
      opt.step()
      sched.step()
      tot += (loss.item(), cls.item(), reg.item())
    msg = f'  epoch {epoch + 1}/{args.epochs} loss {tot[0] / len(dl):.3f} cls {tot[1] / len(dl):.3f} reg {tot[2] / len(dl):.3f} {time.time() - t0:.0f}s'
    if va is not None and (epoch + 1 == args.epochs or (epoch + 1) % 4 == 0):
      res = evaluate(*predict(model, loader(data_dir, index, va, False, args.batch * 2, args.workers), device), index)
      msg += f" | val auc {res.get('auc', float('nan')):.3f} ours {res.get('ours_mae_all', float('nan')):.1f}m chestnut {res.get('chestnut_mae_all') or float('nan'):.1f}m"
    log(msg)
  return model


def export(model, out_dir: Path):
  model = model.float().cpu().eval().to(memory_format=torch.contiguous_format)
  dummy = (torch.zeros(1, 3, INPUT_H, INPUT_W), torch.zeros(1, model.aux)) if model.aux else torch.zeros(1, 3, INPUT_H, INPUT_W)
  path = out_dir / 'stop_line.onnx'
  torch.onnx.export(model, dummy, path, input_names=['image', 'chestnut'][:2 if model.aux else 1], output_names=['out'], opset_version=13, dynamo=False)
  return path


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument('data_dir', type=Path)
  ap.add_argument('out_dir', type=Path)
  ap.add_argument('--folds', type=int, default=4)
  ap.add_argument('--epochs', type=int, default=12)
  ap.add_argument('--batch', type=int, default=64)
  ap.add_argument('--lr', type=float, default=1e-3)
  ap.add_argument('--workers', type=int, default=4, help='JPEG loader processes (~1 GB RAM each); keep low while gaming')
  ap.add_argument('--split', choices=('place', 'drive'), default='place')
  ap.add_argument('--corrector', action='store_true', help="also feed chestnut's planned stop: learn to correct it")
  ap.add_argument('--final', action='store_true')
  args = ap.parse_args()
  args.out_dir.mkdir(parents=True, exist_ok=True)
  index = dict(np.load(args.data_dir / 'index.npz'))
  device = torch.device('cuda')
  torch.backends.cudnn.benchmark = True
  log_file = open(args.out_dir / 'train.log', 'a')

  def log(msg):
    print(msg, flush=True)
    log_file.write(msg + '\n')
    log_file.flush()

  log(f"{time.strftime('%Y-%m-%d %H:%M')} frames {index['target'].size} (positive {(index['target'] == 1).sum()}), "
      f"places {np.unique(index['place'][index['place'] >= 0]).size}")
  if args.final:
    model = train_one(args.data_dir, index, np.arange(index['target'].size), None, args, device, log)
    torch.save(model.state_dict(), args.out_dir / 'stop_line.pt')
    log(f'exported {export(model, args.out_dir)}')
    return
  results = []
  for fold in range(args.folds):
    tr, va = split(index, fold, args.folds, by=args.split)
    log(f'fold {fold + 1}/{args.folds}: train {tr.size} val {va.size}')
    model = train_one(args.data_dir, index, tr, va, args, device, log)
    out, keys = predict(model, loader(args.data_dir, index, va, False, args.batch * 2, args.workers), device)
    np.savez(args.out_dir / f'fold{fold + 1}.npz', out=out, keys=keys)
    res = evaluate(out, keys, index)
    log('  ' + json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in res.items()}))
    results.append(res)
  keys = [k for k in results[0] if isinstance(results[0][k], float)]
  mean = {k: float(np.mean([r[k] for r in results if r.get(k) is not None])) for k in keys}
  log('cross-validation mean ' + json.dumps({k: round(v, 3) for k, v in mean.items()}))
  (args.out_dir / 'cv.json').write_text(json.dumps({'folds': results, 'mean': mean}, indent=2))


if __name__ == '__main__':
  main()
