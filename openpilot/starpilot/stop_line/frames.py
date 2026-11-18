#!/usr/bin/env python3
"""Decode the training frames for the stop-line finder from the drives' road video on DR4L (PC side).

  python -m openpilot.starpilot.stop_line.frames <data_dir> [--host root@192.168.1.3] [--src /mnt/user/comma] [--jobs 8]

Reads <data_dir>/routes/*.npz (extract.py), <data_dir>/review.json (each front stop tagged light, sign or other by
looking at it) and <data_dir>/lines_review.json (lines.py's painted lines, checked by eye; rolling stops tagged).
Each stop's label point is where to stop by its line when there is one (stop_events). Approaches to light and sign
stops are kept every POSITIVE_STRIDE frames, 'no stop ahead' frames
every NEGATIVE_STRIDE frames; approaches to 'other' stops (lot exits, driveways) are left out. Frames are the narrow
road camera (fcamera.hevc) scaled to INPUT_W x INPUT_H and stored as JPEG under <data_dir>/frames/, listed in
<data_dir>/index.npz with their labels, the chestnut stop point and the place each stop is at (stops within
PLACE_M of each other are one place, so validation can hold out whole intersections).
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

INPUT_W, INPUT_H = 448, 256
SRC_W, SRC_H = 1344, 760
POSITIVE_STRIDE = 2
NEGATIVE_STRIDE = 20
PLACE_M = 40.0
TAGS = ('light', 'sign')
LABEL_RANGE_M = 120.0


def places(points: list[tuple[float, float]]) -> list[int]:
  ids = [-1] * len(points)
  centers: list[tuple[float, float]] = []
  for i, (lat, lon) in enumerate(points):
    if not (np.isfinite(lat) and np.isfinite(lon)):
      ids[i] = len(centers)
      centers.append((np.nan, np.nan))
      continue
    for k, (clat, clon) in enumerate(centers):
      if np.isfinite(clat) and np.hypot((lat - clat) * 111_000, (lon - clon) * 111_000 * np.cos(np.radians(lat))) < PLACE_M:
        ids[i] = k
        break
    else:
      ids[i] = len(centers)
      centers.append((lat, lon))
  return ids


def stop_events(data_dir: Path) -> tuple[list[dict], float | None]:
  """Every light or sign stop to learn from, with its stop point in odometer metres.

  The point is where to stop: CAMERA_BEFORE_LINE before the painted line when lines.py found it and a person checked
  it (lines_review.json) or autolabel.py accepted it (auto_labels.json; a person's review always wins), else where the
  car came to rest (a worn or missing line). Rolling stops count only with a checked line, since where the car was
  slowest says little about where it should have stopped."""
  review = {(r['route'], r['stop']): r['tag'] for r in json.loads((data_dir / 'review.json').read_text())}
  lines_path = data_dir / 'lines_review.json'
  checked = json.loads(lines_path.read_text()) if lines_path.exists() else []
  line_at = {(r['route'], r['kind'], r['index']): r for r in checked if r['line_ok']}
  reviewed = {(r['route'], r['kind'], r['index']) for r in checked}
  auto_path = data_dir / 'auto_labels.json'
  auto = [r for r in (json.loads(auto_path.read_text()) if auto_path.exists() else [])
          if r['accepted'] and (r['route'], r['kind'], r['index']) not in reviewed]
  auto_tag = {(r['route'], r['index']): r['tag'] for r in auto if r['kind'] == 'full'}
  line_at.update({(r['route'], r['kind'], r['index']): r for r in auto})
  rest_offsets = [r['line_ahead_m'] for r in checked if r['kind'] == 'full' and r['line_ok']]
  camera_before_line = float(np.median(rest_offsets)) if rest_offsets else None
  events = []
  for path in sorted((data_dir / 'routes').glob('*.npz')):
    route = path.stem
    d = np.load(path)
    s = d['s']
    for n, st in enumerate(json.loads(str(d['stops']))):
      tag = review.get((route, n)) or auto_tag.get((route, st['index']))
      if st['front'] and tag in TAGS:
        line = line_at.get((route, 'full', st['index']))
        point = line['line_odometer'] - camera_before_line if line and camera_before_line is not None else float(s[st['index']])
        events.append(dict(route=route, kind='full', index=st['index'], tag=tag, point=point, lat=st['lat'], lon=st['lon'],
                           source='line' if line else 'rest'))
      elif not st['front'] and not st['last']:
        # a queue stop: the line was hidden by the car ahead and seen when the queue moved on
        line = line_at.get((route, 'queue', st['index']))
        if line is not None and line['tag'] in TAGS and camera_before_line is not None:
          events.append(dict(route=route, kind='queue', index=st['index'], tag=line['tag'], lat=st['lat'], lon=st['lon'],
                             point=line['line_odometer'] - camera_before_line, source='line', auto=bool(line.get('rule'))))
    for r in json.loads(str(d['rolls'])):
      line = line_at.get((route, 'rolling', r['index']))
      if line is None or line['tag'] not in TAGS or camera_before_line is None:
        continue
      events.append(dict(route=route, kind='rolling', index=r['index'], tag=line['tag'], lat=r['lat'], lon=r['lon'],
                         point=line['line_odometer'] - camera_before_line, source='line'))
    for e in events:
      if e['route'] == route:
        # where the stop point is on the map (a queue stop is well short of it): move along the heading
        ahead = e['point'] - float(s[e['index']])
        bearing = np.radians(float(d['bearing'][e['index']]))
        if np.isfinite(bearing) and np.isfinite(e['lat']):
          e['map_lat'] = e['lat'] + ahead * np.cos(bearing) / 111_000
          e['map_lon'] = e['lon'] + ahead * np.sin(bearing) / (111_000 * np.cos(np.radians(e['lat'])))
  for e, place in zip(events, places([(e.get('map_lat', e['lat']), e.get('map_lon', e['lon'])) for e in events]), strict=True):
    e['place'] = place
  return events, camera_before_line


def plan(data_dir: Path, video: set[str]) -> tuple[dict[tuple[str, int], list[dict]], dict]:
  events, camera_before_line = stop_events(data_dir)
  by_route: dict[str, list[dict]] = defaultdict(list)
  for n, e in enumerate(events):
    by_route[e['route']].append(dict(e, id=n))
  wanted: dict[tuple[str, int], list[dict]] = defaultdict(list)
  for path in sorted((data_dir / 'routes').glob('*.npz')):
    route = path.stem
    d = dict(np.load(path))  # load each array once; indexing the lazy NpzFile re-reads the array every time
    s, v, target = d['s'], np.nan_to_num(d['v']), d['target'].astype(np.int8)
    # the approach to a stop starts after the previous stop or slow-down of any kind: never label across one
    every = sorted([st['index'] for st in json.loads(str(d['stops']))] + [r['index'] for r in json.loads(str(d['rolls']))])
    dist = np.full(s.size, np.nan)
    owner = np.full(s.size, -1)
    for e in by_route.get(route, []):
      lo = max([i for i in every if i < e['index']], default=-1) + 1
      # a queue stop's approach goes on past where the car waited, up to the line or the next stop
      hi = min([i for i in every if i > e['index']], default=s.size) if e['kind'] == 'queue' else e['index'] + 1
      k = np.arange(lo, min(hi, s.size))
      k = k[(e['point'] - s[k] > 0.0) & (e['point'] - s[k] <= LABEL_RANGE_M) & (v[k] >= 0.3)]
      dist[k], owner[k] = e['point'] - s[k], e['id']
    for k in range(s.size):
      seg, frame = int(d['seg'][k]), int(d['seg_frame'][k])
      if frame < 0 or f'{route}--{seg}' not in video:
        continue
      if owner[k] >= 0:
        if frame % POSITIVE_STRIDE:
          continue
        e = events[owner[k]]
        row = dict(target=1, dist=float(dist[k]), tag=e['tag'], place=e['place'], stop=int(owner[k]), kind=e['kind'],
                   source=e['source'])
      elif target[k] == 0:
        if frame % NEGATIVE_STRIDE:
          continue
        row = dict(target=0, dist=float('nan'), tag='none', place=-1, stop=-1, kind='none', source='none')
      else:
        continue
      wanted[(route, seg)].append(dict(row, frame=frame, v=float(d['v'][k]), model_stop=float(d['model_stop'][k]),
                                       big=bool(d['big'][k]), route=route, seg=seg))
  meta = dict(stops=len(events), places=len({e['place'] for e in events}), camera_before_line=camera_before_line,
              rolling=sum(e['kind'] == 'rolling' for e in events), queue=sum(e['kind'] == 'queue' for e in events),
              from_line=sum(e['source'] == 'line' for e in events))
  return wanted, meta


def decode_segment(host: str, src: str, route: str, seg: int, frames: list[int], out_dir: Path) -> list[int]:
  from PIL import Image
  seg_dir = out_dir / f'{route}--{seg}'
  seg_dir.mkdir(parents=True, exist_ok=True)
  want = set(frames)
  if all((seg_dir / f'{f}.jpg').exists() for f in want):
    return sorted(want)
  cmd = (f'ssh -o BatchMode=yes {host} cat {src}/{route}--{seg}/fcamera.hevc | '
         f'ffmpeg -v error -i - -vf scale={INPUT_W}:{INPUT_H} -f rawvideo -pix_fmt rgb24 - 2>/dev/null')  # closed early on purpose
  proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
  size = INPUT_W * INPUT_H * 3
  done, n = [], 0
  assert proc.stdout is not None
  while want:
    raw = proc.stdout.read(size)
    if len(raw) < size:
      break
    if n in want:
      Image.frombytes('RGB', (INPUT_W, INPUT_H), raw).save(seg_dir / f'{n}.jpg', quality=92)
      done.append(n)
      want.discard(n)
    n += 1
  proc.kill()
  proc.wait()
  return done


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument('data_dir', type=Path)
  ap.add_argument('--host', default='root@192.168.1.3')
  ap.add_argument('--src', default='/mnt/user/comma')
  ap.add_argument('--jobs', type=int, default=8)
  args = ap.parse_args()
  listing = subprocess.run(['ssh', '-o', 'BatchMode=yes', args.host, f'cd {args.src} && ls -d */fcamera.hevc'],
                           check=True, capture_output=True, text=True).stdout.split()
  video = {p.split('/')[0] for p in listing}
  wanted, meta = plan(args.data_dir, video)
  rows = [r for rs in wanted.values() for r in rs]
  print(f"{meta['stops']} light/sign stops ({meta['rolling']} rolling, {meta['queue']} queue, {meta['from_line']} placed by their line, camera "
        f"{meta['camera_before_line']} m before it) at {meta['places']} places; {sum(r['target'] == 1 for r in rows)} positive and "
        f"{sum(r['target'] == 0 for r in rows)} negative frames from {len(wanted)} segments", flush=True)
  out_dir = args.data_dir / 'frames'
  got: set[tuple[str, int, int]] = set()
  with ProcessPoolExecutor(args.jobs) as pool:
    futures = {pool.submit(decode_segment, args.host, args.src, route, seg, [r['frame'] for r in rs], out_dir): (route, seg)
               for (route, seg), rs in wanted.items()}
    for n, fut in enumerate(as_completed(futures), 1):
      route, seg = futures[fut]
      try:
        got.update((route, seg, f) for f in fut.result())
      except Exception as e:
        print(f'{route}--{seg}: {e}', file=sys.stderr, flush=True)
      if n % 50 == 0:
        print(f'[{n}/{len(futures)}] segments', flush=True)
  rows = [r for r in rows if (r['route'], r['seg'], r['frame']) in got]
  cols = {k: np.asarray([r[k] for r in rows]) for k in rows[0]}
  np.savez(args.data_dir / 'index.npz', **cols)
  print(f'{len(rows)} frames indexed', flush=True)


if __name__ == '__main__':
  main()
