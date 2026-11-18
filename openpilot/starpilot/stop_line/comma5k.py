#!/usr/bin/env python3
"""comma5k (comma.ai's public dataset, on DR4L's comma share) -> stop-line tables like extract.py's (PC side).

  python -m openpilot.starpilot.stop_line.comma5k <data_dir> [--host root@192.168.1.3] [--src /mnt/user/comma/comma5k]
                                                  [--jobs 12]

Each comma5k segment is a lone minute from someone else's car (comma three, 3X or four), with stripped rlogs: carState,
raw radar tracks, calibration, camera frame indices and video, but no controls, no driving model and no processed
radar. Usable for labels: US/Canada (roads like the user's) with working radar tracks (else a stop behind another car
looks first in line). One <data_dir>/comma5k/routes/c5k-<id>.npz per segment, one row per road-camera frame, with the same fields
as extract.py's tables where they exist: no chestnut plan (model_stop NaN) and no sign detections. The car ahead
comes from radar tracks (within LEAD_HALF_WIDTH_M of the lane centre); positions and heading from the
segment's offline localizer (frame_states, ECEF). Each table also records the camera's size and focal length.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import struct
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

from openpilot.starpilot.stop_line.labels import find_rolls, find_stops, label_frames, travel

LEAD_HALF_WIDTH_M = 1.0
# narrow road camera per device (common/transformations/camera.py)
CAMERAS = {'os04c10': (1344, 760, 1522.0 * 3 / 4), 'ar0231': (1928, 1208, 2648.0), 'ox03c10': (1928, 1208, 2648.0)}
FIELDS = ('t', 'frame_id', 'seg', 'seg_frame', 'v', 'standstill', 'brake', 'gas', 'long_active', 'lead_d', 'model_stop',
          'model_should_stop', 'big', 'sign', 'lat', 'lon', 'bearing', 'cal_roll', 'cal_pitch', 'cal_yaw', 'cal_height')


def safetensors(path: str) -> dict[str, np.ndarray]:
  raw = open(path, 'rb').read()
  n = struct.unpack('<Q', raw[:8])[0]
  head = json.loads(raw[8:8 + n])
  out = {}
  for k, v in head.items():
    if k == '__metadata__':
      continue
    a, b = v['data_offsets']
    out[k] = np.frombuffer(raw[8 + n + a:8 + n + b], dtype={'F64': np.float64, 'F32': np.float32}[v['dtype']]).reshape(v['shape'])
  return out


def geodetic(ecef: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
  """ECEF (N, 3) -> latitude, longitude (degrees), and the east/north unit vectors there (WGS84, Bowring)."""
  a, e2 = 6378137.0, 6.69437999014e-3
  b = a * math.sqrt(1 - e2)
  ep2 = (a * a - b * b) / (b * b)
  x, y, z = ecef.T
  p = np.hypot(x, y)
  th = np.arctan2(z * a, p * b)
  lat = np.arctan2(z + ep2 * b * np.sin(th) ** 3, p - e2 * a * np.cos(th) ** 3)
  lon = np.arctan2(y, x)
  east = np.stack([-np.sin(lon), np.cos(lon), np.zeros_like(lon)], axis=1)
  north = np.stack([-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)], axis=1)
  return np.degrees(lat), np.degrees(lon), np.stack([east, north], axis=1)


def read_segment(rlog: str, localizer: str) -> tuple[dict[str, np.ndarray], dict] | None:
  from openpilot.tools.lib.logreader import LogReader
  frames, car, lead = [], [], []
  radar_points = 0  # many cars log no radar tracks at all: then a queue stop cannot be told from a first-in-line one
  cal = (np.nan,) * 4
  device = sensor = None
  for m in LogReader(rlog, sort_by_time=True):
    w = m.which()
    if w == 'initData':
      device = str(m.initData.deviceType)
    elif w == 'narrowRoadCameraState' and sensor is None:
      sensor = str(m.narrowRoadCameraState.sensor)
    elif w == 'extrinsicsCalibration':
      c = m.extrinsicsCalibration
      if str(c.calStatus) == 'calibrated' and len(c.rpyCalib) == 3:
        cal = (*c.rpyCalib, c.height[0] if len(c.height) else 1.22)
    elif w == 'carState':
      cs = m.carState
      car.append((m.logMonoTime, cs.vEgo, cs.standstill, cs.brakePressed, cs.gasPressed))
    elif w == 'radarTracks':
      radar_points += len(m.radarTracks.points)
      near = [p.dRel for p in m.radarTracks.points if abs(p.yRel) < LEAD_HALF_WIDTH_M and p.dRel > 0]  # 'measured' is deprecated here
      lead.append((m.logMonoTime, min(near) if near else np.nan))
    elif w == 'narrowRoadEncodeIdx':
      e = m.narrowRoadEncodeIdx
      frames.append((m.logMonoTime, e.frameId, e.segmentId, *cal))
  if not frames or not car:
    return None
  if sensor in (None, 'unknown'):
    sensor = 'os04c10' if device == 'mici' else 'ar0231'  # tici before the sensor field: AR0231
  if sensor not in CAMERAS:
    return None
  f = np.array(frames, dtype=np.float64)
  c = np.array(car, dtype=np.float64)
  t = f[:, 0] * 1e-9
  idx = np.clip(np.searchsorted(c[:, 0], f[:, 0]) - 1, 0, len(c) - 1)  # latest carState before each frame
  data = {'t': t, 'frame_id': f[:, 1], 'seg': np.zeros(len(f)), 'seg_frame': f[:, 2], 'v': c[idx, 1],
          'standstill': c[idx, 2].astype(bool), 'brake': c[idx, 3].astype(bool), 'gas': c[idx, 4].astype(bool),
          'long_active': np.zeros(len(f), bool), 'model_stop': np.full(len(f), np.nan),
          'model_should_stop': np.zeros(len(f), bool), 'big': np.zeros(len(f), bool), 'sign': np.zeros(len(f), bool),
          'cal_roll': f[:, 3], 'cal_pitch': f[:, 4], 'cal_yaw': f[:, 5], 'cal_height': f[:, 6]}
  if lead:
    ld = np.array(lead, dtype=np.float64)
    j = np.clip(np.searchsorted(ld[:, 0], f[:, 0]) - 1, 0, len(ld) - 1)
    data['lead_d'] = ld[j, 1]
  else:
    data['lead_d'] = np.full(len(f), np.nan)
  data['lat'] = data['lon'] = data['bearing'] = np.full(len(f), np.nan)
  try:
    loc = safetensors(localizer)
    fs = loc['frame_states']
    if len(fs) == len(f):
      lat, lon, en = geodetic(fs[:, :3])
      vel = fs[:, 7:10]
      ve, vn = np.einsum('nij,nj->ni', en, vel).T
      data['lat'], data['lon'] = lat, lon
      data['bearing'] = np.where(np.hypot(ve, vn) > 0.5, (np.degrees(np.arctan2(ve, vn)) + 360) % 360, np.nan)
  except (OSError, KeyError, ValueError):
    pass
  w, h, focal = CAMERAS[sensor]
  lat = data['lat'][np.isfinite(data['lat'])]
  lon = data['lon'][np.isfinite(data['lon'])]
  # US/Canada: stop lines, signs and signal placement like the user's roads
  north_america = bool(lat.size and 24 < np.median(lat) < 60 and -170 < np.median(lon) < -50)
  return data, dict(device=device, sensor=sensor, width=w, height=h, focal=focal, radar_points=int(radar_points),
                    north_america=north_america)


def fetch(host: str, remote: str, local: str, tries: int = 4) -> bool:
  """scp with retries: DR4L's sshd drops some connections when many arrive at once."""
  for attempt in range(tries):
    if subprocess.run(['scp', '-q', '-o', 'BatchMode=yes', f'{host}:{remote}', local]).returncode == 0:
      return True
    time.sleep(2 + 3 * attempt)
  return False


def extract_one(seg_id: str, host: str, src: str, out_dir: Path) -> dict:
  with tempfile.TemporaryDirectory(dir='/tmp') as tmp:
    for name in ('rlog.zst', 'localizer.safetensors'):
      if not fetch(host, f'{src}/data/{seg_id}/{name}', os.path.join(tmp, name)):
        return dict(id=seg_id, error=f'no {name}')
    got = read_segment(os.path.join(tmp, 'rlog.zst'), os.path.join(tmp, 'localizer.safetensors'))
  if got is None:
    return dict(id=seg_id, error='unreadable or unknown camera')
  data, camera = got
  t, v = data['t'], np.nan_to_num(data['v'])
  s = travel(t, v)
  stops = find_stops(t, v, s, data['lead_d'], data['brake'], data['model_stop'], data['sign'])
  rolls = find_rolls(t, v, s, data['lead_d'], data['sign'])
  dist, target, stop_id = label_frames(t, v, s, stops, rolls)
  stops_out = [dict(index=int(st.rest_index), t=float(t[st.rest_index]), seg=0, lat=float(data['lat'][st.rest_index]),
                    lon=float(data['lon'][st.rest_index]), front=st.front, last=st.last, driver_braked=st.driver_braked,
                    model_agreed=False, sign_seen=False) for st in stops]
  rolls_out = [dict(index=int(r.slow_index), t=float(t[r.slow_index]), seg=0, lat=float(data['lat'][r.slow_index]),
                    lon=float(data['lon'][r.slow_index]), min_speed=r.min_speed, sign_seen=False) for r in rolls]
  route = f'c5k-{seg_id}'
  np.savez_compressed(out_dir / f'{route}.npz', dist=dist, target=target, stop_id=stop_id, s=s, **data,
                      stops=json.dumps(stops_out), rolls=json.dumps(rolls_out), camera=json.dumps(camera))
  return dict(id=seg_id, route=route, **camera, usable=camera['radar_points'] > 0 and camera['north_america'],
              front=sum(st.front and not st.last for st in stops),
              queue=sum(not st.front and not st.last for st in stops), rolling=len(rolls))


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument('data_dir', type=Path)
  ap.add_argument('--host', default='root@192.168.1.3')
  ap.add_argument('--src', default='/mnt/user/comma/comma5k')
  ap.add_argument('--jobs', type=int, default=6)  # more parallel copies than this trips DR4L's ssh connection limit
  args = ap.parse_args()
  out_dir = args.data_dir / 'comma5k' / 'routes'
  out_dir.mkdir(parents=True, exist_ok=True)
  # a segment is ready once its rlog, localizer and road video are all there (the download is still running)
  ready = subprocess.run(['ssh', '-o', 'BatchMode=yes', args.host,
                          f'cd {args.src}/data && for d in *; do [ -f $d/rlog.zst ] && [ -f $d/localizer.safetensors ] && '
                          f'[ -f $d/fcamera.hevc ] && echo $d; done; true'], capture_output=True, text=True).stdout.split()
  todo = [s for s in ready if not (out_dir / f'c5k-{s}.npz').exists()]
  print(f'{len(ready)} comma5k segments downloaded, {len(todo)} new', flush=True)
  results = args.data_dir / 'comma5k' / 'results.jsonl'
  with ProcessPoolExecutor(args.jobs) as pool, open(results, 'a') as log:
    futs = [pool.submit(extract_one, s, args.host, args.src, out_dir) for s in todo]
    for n, fut in enumerate(as_completed(futs), 1):
      try:
        r = fut.result()
      except Exception as e:
        r = dict(error=str(e)[:200])
      log.write(json.dumps(r) + '\n')
      if n % 100 == 0:
        print(f'[{n}/{len(todo)}]', flush=True)
  rows = [json.loads(line) for line in open(results)]
  ok = [r for r in rows if 'error' not in r]
  use = [r for r in ok if r.get('usable')]
  print(json.dumps(dict(segments=len(ok), errors=len(rows) - len(ok), usable=len(use),
                        usable_front_stops=sum(r['front'] for r in use), usable_queue_stops=sum(r['queue'] for r in use),
                        usable_rolling=sum(r['rolling'] for r in use),
                        cameras={s: sum(r['sensor'] == s for r in ok) for s in CAMERAS}), indent=1))


if __name__ == '__main__':
  main()
