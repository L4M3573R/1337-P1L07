#!/usr/bin/env python3
"""Pull the drives' rlogs from DR4L and write per-drive stop-line label tables (PC side).

  python -m openpilot.starpilot.stop_line.extract <out_dir> [--host root@192.168.1.3] [--src /mnt/user/comma]
                                                  [--jobs 12] [--route ROUTE ...]

One <out_dir>/routes/<route>.npz per drive, at the 20 Hz model rate: the frame ids that locate each frame in
that segment's fcamera.hevc, speed, brake, radar lead, the driving model's planned stop distance (chestnut's
stop point when modelV2.big), the sign detector's stop-sign flag, GPS, and the labels from labels.py.
<out_dir>/summary.json counts the stops by kind. A drive is extracted again when the puller has brought segments its
table doesn't have yet (old drives arrive piece by piece); otherwise it is skipped.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

from openpilot.starpilot.stop_line.labels import find_rolls, find_stops, label_frames, model_stop_distance, travel

GAP_S = 1.0
FIELDS = ('t', 'frame_id', 'seg', 'seg_frame', 'v', 'standstill', 'brake', 'gas', 'long_active', 'lead_d',
          'model_stop', 'model_should_stop', 'big', 'sign', 'lat', 'lon', 'bearing', 'cal_roll', 'cal_pitch', 'cal_yaw',
          'cal_height')


def list_segments(host: str, src: str) -> dict[str, list[int]]:
  out = subprocess.run(['ssh', '-o', 'BatchMode=yes', host, f'cd {src} && ls -d */rlog.zst'],
                       check=True, capture_output=True, text=True).stdout.split()
  routes: dict[str, list[int]] = defaultdict(list)
  for entry in out:
    name = entry.split('/')[0]
    route, _, seg = name.rpartition('--')
    if seg.isdigit():
      routes[route].append(int(seg))
  return {r: sorted(s) for r, s in routes.items()}


def read_segment(path: str, seg: int) -> dict[str, np.ndarray] | None:
  from openpilot.tools.lib.logreader import LogReader
  rows: dict[str, list] = {k: [] for k in FIELDS}
  car = dict(v=np.nan, standstill=False, brake=False, gas=False)
  long_active, lead_d, sign = False, np.nan, False
  gps = (np.nan, np.nan, np.nan)
  cal = (np.nan, np.nan, np.nan, np.nan)  # camera roll, pitch, yaw (rad) and height (m) once calibrated
  encode: dict[int, int] = {}
  for m in LogReader(path, sort_by_time=True):
    w = m.which()
    if w == 'carState':
      cs = m.carState
      car = dict(v=cs.vEgo, standstill=cs.standstill, brake=cs.brakePressed, gas=cs.gasPressed)
    elif w == 'carControl':
      long_active = m.carControl.longActive
    elif w == 'radarState':
      lead = m.radarState.leadOne
      lead_d = lead.dRel if lead.present else np.nan
    elif w == 'slcVisionObservation':
      stop_signs = m.slcVisionObservation.vision.stopSigns
      sign = str(stop_signs.status) == 'valid' and len(stop_signs.detections) > 0
    elif w == 'gpsLocationExternal':
      g = m.gpsLocationExternal
      gps = (g.latitude, g.longitude, g.bearingDeg) if g.hasFix else (np.nan, np.nan, np.nan)
    elif w == 'extrinsicsCalibration':
      c = m.extrinsicsCalibration
      if str(c.calStatus) == 'calibrated' and len(c.rpyCalib) == 3:
        cal = (*c.rpyCalib, c.height[0] if len(c.height) else 1.22)
    elif w == 'narrowRoadEncodeIdx':
      e = m.narrowRoadEncodeIdx
      if e.segmentNum == seg:
        encode[e.frameId] = e.segmentId  # position in this segment's fcamera.hevc (segmentIdEncode is unused here)
    elif w == 'modelV2':
      md = m.modelV2
      rows['t'].append(m.logMonoTime * 1e-9)
      rows['frame_id'].append(md.frameId)
      rows['seg'].append(seg)
      rows['model_stop'].append(model_stop_distance(md.position.x, md.velocity.x))
      rows['model_should_stop'].append(md.action.shouldStop)
      rows['big'].append(md.big)
      for k, val in car.items():
        rows[k].append(val)
      rows['long_active'].append(long_active)
      rows['lead_d'].append(lead_d)
      rows['sign'].append(sign)
      rows['lat'].append(gps[0])
      rows['lon'].append(gps[1])
      rows['bearing'].append(gps[2])
      for name, val in zip(('cal_roll', 'cal_pitch', 'cal_yaw', 'cal_height'), cal, strict=True):
        rows[name].append(val)
  if not rows['t']:
    return None
  rows['seg_frame'] = [encode.get(f, -1) for f in rows['frame_id']]
  return {k: np.asarray(v) for k, v in rows.items()}


def extract_route(route: str, segs: list[int], host: str, src: str, out_dir: Path) -> dict:
  parts, unreadable = [], []
  with tempfile.TemporaryDirectory(dir='/tmp') as tmp:
    for seg in segs:
      local = os.path.join(tmp, f'{seg}.zst')
      got = subprocess.run(['scp', '-q', '-o', 'BatchMode=yes', f'{host}:{src}/{route}--{seg}/rlog.zst', local])
      if got.returncode != 0:
        unreadable.append(seg)
        continue
      try:
        table = read_segment(local, seg)
      except Exception as e:  # a truncated or half-pulled rlog: skip the segment, the gap splits the drive
        print(f'{route}--{seg}: {e}', file=sys.stderr)
        table = None
      os.remove(local)
      if table is not None:
        parts.append(table)
      else:
        unreadable.append(seg)
  if not parts:
    return {'route': route, 'stops': []}
  data = {k: np.concatenate([p[k] for p in parts]) for k in FIELDS}
  # split into pieces at missing segments so no distance is integrated across a gap
  t = data['t']
  breaks = np.flatnonzero(np.diff(t) > GAP_S) + 1
  dist = np.full(t.size, np.nan)
  target = np.full(t.size, -1, dtype=np.int8)
  stop_id = np.full(t.size, -1, dtype=np.int32)
  s_all = np.zeros(t.size)
  stops_out, rolls_out = [], []
  for lo, hi in zip(np.r_[0, breaks], np.r_[breaks, t.size], strict=True):
    sl = slice(lo, hi)
    v = np.nan_to_num(data['v'][sl])
    s = travel(t[sl], v)
    s_all[sl] = s
    stops = find_stops(t[sl], v, s, data['lead_d'][sl], data['brake'][sl], data['model_stop'][sl], data['sign'][sl])
    rolls = find_rolls(t[sl], v, s, data['lead_d'][sl], data['sign'][sl])
    d, tg, sid = label_frames(t[sl], v, s, stops, rolls)
    for r in rolls:
      k = lo + r.slow_index
      rolls_out.append(dict(index=int(k), t=float(t[k]), seg=int(data['seg'][k]), lat=float(data['lat'][k]),
                            lon=float(data['lon'][k]), min_speed=r.min_speed, sign_seen=r.sign_seen))
    dist[sl], target[sl] = d, tg
    stop_id[sl] = np.where(sid >= 0, sid + len(stops_out), -1)
    for st in stops:
      r = lo + st.rest_index
      stops_out.append(dict(index=int(r), t=float(t[r]), seg=int(data['seg'][r]), lat=float(data['lat'][r]),
                            lon=float(data['lon'][r]), front=st.front, last=st.last, driver_braked=st.driver_braked,
                            model_agreed=st.model_agreed, sign_seen=st.sign_seen))
  np.savez_compressed(out_dir / 'routes' / f'{route}.npz', dist=dist, target=target, stop_id=stop_id, s=s_all, **data,
                      stops=json.dumps(stops_out), rolls=json.dumps(rolls_out), unreadable=np.asarray(unreadable, dtype=np.int32))
  return {'route': route, 'stops': stops_out, 'rolls': rolls_out, 'frames': int(t.size), 'big_frames': int(np.count_nonzero(data['big'])),
          'labeled': int(np.count_nonzero(target == 1)), 'negatives': int(np.count_nonzero(target == 0))}


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument('out_dir', type=Path)
  ap.add_argument('--host', default='root@192.168.1.3')
  ap.add_argument('--src', default='/mnt/user/comma')
  ap.add_argument('--jobs', type=int, default=12)
  ap.add_argument('--route', nargs='*')
  args = ap.parse_args()
  (args.out_dir / 'routes').mkdir(parents=True, exist_ok=True)
  routes = list_segments(args.host, args.src)
  if args.route:
    routes = {r: routes[r] for r in args.route}
  todo = {r: s for r, s in routes.items() if stale(args.out_dir / 'routes' / f'{r}.npz', s)}
  print(f'{len(routes)} drives, {len(todo)} to extract', flush=True)
  results_path = args.out_dir / 'results.jsonl'
  with ProcessPoolExecutor(args.jobs) as pool, open(results_path, 'a') as results:
    futures = {pool.submit(extract_route, r, s, args.host, args.src, args.out_dir): r for r, s in todo.items()}
    for n, fut in enumerate(as_completed(futures), 1):
      try:
        res = fut.result()
      except Exception as e:
        print(f'{futures[fut]}: failed {e}', file=sys.stderr, flush=True)
        continue
      results.write(json.dumps(res) + '\n')
      results.flush()
      print(f'[{n}/{len(todo)}] {res["route"]} stops={len(res["stops"])} labeled={res.get("labeled", 0)}', flush=True)
  summarize(args.out_dir)


def stale(path: Path, segments: list[int]) -> bool:
  """Not extracted yet, or the puller has since brought segments this drive's table doesn't have."""
  if not path.exists():
    return True
  d = np.load(path)
  have = set(np.unique(d['seg']).tolist()) | set(d['unreadable'].tolist() if 'unreadable' in d else [])
  return bool(set(segments) - have)


def summarize(out_dir: Path) -> None:
  latest = {}
  for line in open(out_dir / 'results.jsonl'):
    r = json.loads(line)
    latest[r['route']] = r  # a re-extracted drive's newest row replaces the older one
  results = list(latest.values())
  stops = [s for r in results for s in r['stops']]
  front = [s for s in stops if s['front'] and not s['last']]
  summary = dict(drives=len(results), frames=sum(r.get('frames', 0) for r in results),
                 chestnut_frames=sum(r.get('big_frames', 0) for r in results), stops=len(stops),
                 front_stops=len(front), queue_stops=sum(1 for s in stops if not s['front'] and not s['last']),
                 parking=sum(1 for s in stops if s['last']),
                 front_driver_braked=sum(s['driver_braked'] for s in front),
                 front_model_agreed=sum(s['model_agreed'] for s in front),
                 front_sign_seen=sum(s['sign_seen'] for s in front),
                 labeled_frames=sum(r.get('labeled', 0) for r in results),
                 negative_frames=sum(r.get('negatives', 0) for r in results),
                 rolling_stops=sum(len(r.get('rolls', [])) for r in results))
  (out_dir / 'summary.json').write_text(json.dumps(summary, indent=2))
  print(json.dumps(summary, indent=2))


if __name__ == '__main__':
  main()
