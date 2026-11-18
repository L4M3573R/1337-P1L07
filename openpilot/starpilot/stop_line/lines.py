#!/usr/bin/env python3
"""Find the painted stop line in the road video at each stop, so labels say where the line is (PC side).

  python openpilot/starpilot/stop_line/lines.py <data_dir> [--jobs 8]     (with the training venv: needs cv2)

For every first-in-line full stop and every rolling stop that has road video, the approach's full-resolution frames
are projected onto the road (flat ground, this frame's camera calibration from extract.py) into a top-down view of
the lane ahead. A stop line is a bright band across most of the lane, possibly at an angle; each frame that sees one
4-16 m ahead gives the line's position in odometer metres. A real line gives the same position frame after frame, so
positions are clustered and a line needs MIN_SUPPORT frames. Writes <data_dir>/lines.json: per stop the chosen line
(odometer metres of the camera-to-line point, how many frames saw it) and every candidate, for review and labels.

Positions are of the camera: a line at odometer L is under the camera when the car has driven to L. Full stops
measure where drivers stop relative to the line (camera-to-line at rest); that offset turns a rolling stop's line
into where to stop.
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

SRC_W, SRC_H = 1344, 760
FOCAL = 1522.0 * 3 / 4  # narrow road camera (os04c10, half resolution), common/transformations/camera.py
COMMA_FOUR = (SRC_W, SRC_H, FOCAL)  # comma5k's comma three/3X segments carry their own (1928, 1208, 2648.0)
RES_M = 0.05
X_NEAR, X_FAR = 3.5, 20.0
HALF_WIDTH_M = 1.5
SEE_NEAR, SEE_FAR = 4.0, 16.0  # distances a sighting counts at (close enough that braking pitch matters little)
ANGLES_DEG = tuple(range(-28, 29, 4))
MIN_CONTRAST = 10.0
MIN_RELATIVE = 0.2
MIN_COVERAGE = 0.6
CLUSTER_M = 1.0
MIN_SUPPORT = 5
WINDOW_BEFORE_M = 40.0
WINDOW_AFTER_ROLL_M = 8.0
WINDOW_AFTER_QUEUE_M = 50.0  # behind a car the line is hidden; it is seen once the queue moves and the car drives on
QUEUE_LINE_PAST_LEAD_M = 3.0  # the line is at least this far past the stopped car ahead's rear
QUEUE_LINE_MAX_M = 60.0


def rot_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
  """openpilot's orientation.rot_from_euler: R = Rz(yaw) Ry(pitch) Rx(roll)."""
  cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
  rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
  ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
  rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
  return rz @ ry @ rx


def view_from_road(roll: float, pitch: float, yaw: float, height: float) -> np.ndarray:
  """camera.get_view_frame_from_road_frame (road: x forward, y left, z up; origin on the road under the camera)."""
  view_from_device = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])
  device_from_road = rot_from_euler(roll, pitch, yaw) @ np.diag([1., -1., -1.])
  return np.hstack((view_from_device @ device_from_road, [[0.], [height], [0.]]))


def road_maps(roll: float, pitch: float, yaw: float, height: float, camera=COMMA_FOUR) -> tuple[np.ndarray, np.ndarray]:
  """cv2.remap maps for a top-down view: row 0 is X_FAR, each row RES_M nearer; columns span the lane, left first."""
  width, height_px, focal = camera
  intrinsics = np.array([[focal, 0, width / 2], [0, focal, height_px / 2], [0, 0, 1]])
  xs = np.arange(X_FAR, X_NEAR, -RES_M)
  ys = np.arange(HALF_WIDTH_M, -HALF_WIDTH_M, -RES_M)
  x, y = np.meshgrid(xs, ys, indexing='ij')
  pts = np.stack([x, y, np.zeros_like(x), np.ones_like(x)], axis=-1) @ view_from_road(roll, pitch, yaw, height).T @ intrinsics.T
  return (pts[..., 0] / pts[..., 2]).astype(np.float32), (pts[..., 1] / pts[..., 2]).astype(np.float32)


def row_distance(row: float) -> float:
  return X_FAR - row * RES_M


def find_bands(top: np.ndarray) -> list[tuple[float, float, int]]:
  """Bright bands across the lane in a top-down gray view: (metres ahead, contrast, angle deg)."""
  import cv2
  h, w = top.shape
  found: list[tuple[float, float, int]] = []
  for angle in ANGLES_DEG:
    # shear so a line at this angle becomes a row: column c shifts by (c - centre) * tan(angle) rows
    shift = math.tan(math.radians(angle))
    m = np.float32([[1, 0, 0], [shift, 1, -shift * w / 2]])
    sheared = cv2.warpAffine(top, m, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    profile = sheared.astype(np.float32).mean(axis=1)
    bg = np.array([np.median(profile[max(0, r - 20):r + 21]) for r in range(h)])
    score = np.convolve(profile - bg, np.ones(5) / 5, mode='same')
    for r in range(3, h - 3):
      s = score[r]
      if s < MIN_CONTRAST or s < MIN_RELATIVE * (bg[r] + 10.0) or s < score[r - 3:r + 4].max():
        continue
      half = score > s / 2
      lo, hi = r, r
      while lo > 0 and half[lo - 1]:
        lo -= 1
      while hi < h - 1 and half[hi + 1]:
        hi += 1
      thickness = (hi - lo + 1) * RES_M
      if not 0.1 <= thickness <= 0.9:
        continue
      coverage = float(np.mean(sheared[r].astype(np.float32) - bg[r] > s / 2))
      if coverage >= MIN_COVERAGE:
        found.append((row_distance(r), float(s), angle))
  # one sighting per band: keep the strongest angle within 0.5 m
  found.sort(key=lambda f: -f[1])
  kept: list[tuple[float, float, int]] = []
  for f in found:
    if all(abs(f[0] - k[0]) > 0.5 for k in kept):
      kept.append(f)
  return kept


def clusters(sightings: list[tuple[int, float]]) -> list[dict]:
  """(frame, odometer of the line) sightings to clusters of one line each."""
  pts = sorted(sightings, key=lambda p: p[1])
  out, cur = [], []
  for p in pts:
    if cur and p[1] - cur[-1][1] > CLUSTER_M:
      out.append(cur)
      cur = []
    cur.append(p)
  if cur:
    out.append(cur)
  res = []
  for c in out:
    odo = np.array([p[1] for p in c])
    res.append(dict(odometer=float(np.median(odo)), support=len({p[0] for p in c}), spread=float(odo.std())))
  return [c for c in res if c['support'] >= MIN_SUPPORT]


def decode(host: str, video: str, frames: set[int], camera=COMMA_FOUR) -> dict[int, np.ndarray]:
  width, height, _ = camera
  cmd = f'ssh -o BatchMode=yes {host} cat {video} | ffmpeg -v error -i - -f rawvideo -pix_fmt gray - 2>/dev/null'
  proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
  assert proc.stdout is not None
  got, n, last = {}, 0, max(frames)
  while n <= last:
    raw = proc.stdout.read(width * height)
    if len(raw) < width * height:
      break
    if n in frames:
      got[n] = np.frombuffer(raw, dtype=np.uint8).reshape(height, width)
    n += 1
  proc.kill()
  proc.wait()
  return got


def locate(job: dict, host: str, src: str, data_dir: Path) -> dict:
  import cv2
  d = np.load(job.get('table') or data_dir / 'routes' / f"{job['route']}.npz")
  camera = tuple(job.get('camera') or COMMA_FOUR)
  s, seg, frame, v = d['s'], d['seg'], d['seg_frame'], d['v']
  cal = np.stack([d['cal_roll'], d['cal_pitch'], d['cal_yaw'], d['cal_height']], axis=1)
  k = job['index']
  after = {'rolling': WINDOW_AFTER_ROLL_M, 'queue': WINDOW_AFTER_QUEUE_M}.get(job['kind'], 0.0)
  window = np.flatnonzero((s >= s[k] - WINDOW_BEFORE_M) & (s <= s[k] + after) & (frame >= 0) & np.isfinite(cal).all(axis=1))
  window = window[(np.nan_to_num(v[window]) > 0.3) | (window == k)][::2]
  if job['kind'] == 'queue' and job.get('table'):
    window = window[::2]  # comma5k queue windows are long and the video is 1928x1208: every 4th frame is plenty
  result = dict(job, line=None, candidates=[], frames=int(window.size))
  if window.size < MIN_SUPPORT:
    result['why'] = 'no calibrated video'
    return result
  maps = road_maps(*cal[window[0]], camera=camera)
  sightings = []
  for segment_num in np.unique(seg[window]):
    idx = window[seg[window] == segment_num]
    video = job.get('video') or f"{src}/{job['route']}--{segment_num}/fcamera.hevc"
    images = decode(host, video, {int(frame[i]) for i in idx}, camera)
    for i in idx:
      img = images.get(int(frame[i]))
      if img is None:
        continue
      top = cv2.remap(img, *maps, cv2.INTER_LINEAR)
      for ahead, _, _ in find_bands(top):
        if SEE_NEAR <= ahead <= SEE_FAR:
          sightings.append((int(i), float(s[i] + ahead)))
  cands = clusters(sightings)
  for c in cands:
    c['ahead_of_event_m'] = round(c['odometer'] - float(s[k]), 2)
  result['candidates'] = cands
  result['event_odometer'] = float(s[k])
  return result


def jobs_for(data_dir: Path, video: set[str]) -> list[dict]:
  jobs = []
  for path in sorted((data_dir / 'routes').glob('*.npz')):
    d = np.load(path)
    route, seg = path.stem, d['seg']
    for n, st in enumerate(json.loads(str(d['stops']))):
      if st['last'] or f"{route}--{st['seg']}" not in video:
        continue
      if st['front']:
        jobs.append(dict(route=route, kind='full', stop=n, index=st['index']))
      else:
        lead = d['lead_d'][st['index']:st['index'] + 20]
        lead = lead[np.isfinite(lead)]
        jobs.append(dict(route=route, kind='queue', stop=n, index=st['index'], lead_m=float(lead.min()) if lead.size else None))
    for n, r in enumerate(json.loads(str(d['rolls']))):
      if f"{route}--{int(seg[r['index']])}" in video:
        jobs.append(dict(route=route, kind='rolling', roll=n, index=r['index'], sign_seen=r['sign_seen']))
  return jobs


def comma5k_jobs(data_dir: Path, src: str) -> list[dict]:
  """Stops in comma5k segments usable for labels (comma5k.py: US/Canada with working radar), each with its camera."""
  jobs = []
  for path in sorted((data_dir / 'comma5k' / 'routes').glob('*.npz')):
    d = np.load(path)
    cam = json.loads(str(d['camera']))
    if not (cam.get('radar_points', 0) > 0 and cam.get('north_america')):
      continue
    seg_id = path.stem.removeprefix('c5k-')
    common = dict(route=path.stem, table=str(path), video=f'{src}/comma5k/data/{seg_id}/fcamera.hevc',
                  camera=[cam['width'], cam['height'], cam['focal']])
    for n, st in enumerate(json.loads(str(d['stops']))):
      if st['last']:
        continue
      lead = d['lead_d'][st['index']:st['index'] + 20]
      lead = lead[np.isfinite(lead)]
      jobs.append(dict(common, kind='full' if st['front'] else 'queue', stop=n, index=st['index'],
                       lead_m=float(lead.min()) if lead.size else None))
    for n, r in enumerate(json.loads(str(d['rolls']))):
      jobs.append(dict(common, kind='rolling', roll=n, index=r['index'], sign_seen=False))
  return jobs


def choose(results: list[dict]) -> float | None:
  """Nearest line ahead of each full stop; their median camera-to-line distance at rest picks the rolling stops' lines."""
  offsets = []
  for r in results:
    if r['kind'] != 'full':
      continue
    ahead = [c for c in r['candidates'] if -1.0 <= c['ahead_of_event_m'] <= 15.0]
    if ahead:
      r['line'] = min(ahead, key=lambda c: c['ahead_of_event_m'])
      offsets.append(r['line']['ahead_of_event_m'])
  offset = float(np.median(offsets)) if offsets else None
  for r in results:
    if r['kind'] != 'queue' or r.get('lead_m') is None:
      continue
    # the first line past the car that was stopped ahead
    past = [c for c in r['candidates'] if r['lead_m'] + QUEUE_LINE_PAST_LEAD_M <= c['ahead_of_event_m'] <= QUEUE_LINE_MAX_M]
    if past:
      r['line'] = min(past, key=lambda c: c['ahead_of_event_m'])
  for r in results:
    if r['kind'] != 'rolling' or offset is None:
      continue
    # a rolling stop's slowest point is near where a full stop would rest, give or take rolling past the line
    near = [c for c in r['candidates'] if -6.0 <= c['ahead_of_event_m'] <= 15.0]
    if near:
      r['line'] = min(near, key=lambda c: abs(c['ahead_of_event_m'] - offset))
  return offset


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument('data_dir', type=Path)
  ap.add_argument('--host', default='root@192.168.1.3')
  ap.add_argument('--src', default='/mnt/user/comma')
  ap.add_argument('--jobs', type=int, default=8)
  ap.add_argument('--comma5k', action='store_true', help='comma5k segments instead of your drives (comma5k/lines.json)')
  args = ap.parse_args()
  if args.comma5k:
    jobs = comma5k_jobs(args.data_dir, args.src)
  else:
    listing = subprocess.run(['ssh', '-o', 'BatchMode=yes', args.host, f'cd {args.src} && ls -d */fcamera.hevc'],
                             check=True, capture_output=True, text=True).stdout.split()
    jobs = jobs_for(args.data_dir, {p.split('/')[0] for p in listing})
  print(f"{sum(j['kind'] == 'full' for j in jobs)} full, {sum(j['kind'] == 'queue' for j in jobs)} queue and "
        f"{sum(j['kind'] == 'rolling' for j in jobs)} rolling stops with video",
        flush=True)
  results = []
  with ProcessPoolExecutor(args.jobs) as pool:
    futures = [pool.submit(locate, j, args.host, args.src, args.data_dir) for j in jobs]
    for n, fut in enumerate(as_completed(futures), 1):
      try:
        results.append(fut.result())
      except Exception as e:
        print(f'failed: {e}', file=sys.stderr, flush=True)
      if n % 20 == 0:
        print(f'[{n}/{len(jobs)}]', flush=True)
  offset = choose(results)
  out = args.data_dir / 'comma5k' / 'lines.json' if args.comma5k else args.data_dir / 'lines.json'
  out.write_text(json.dumps({'rest_offset_m': offset, 'stops': results}, indent=1))
  full = [r for r in results if r['kind'] == 'full']
  rolling = [r for r in results if r['kind'] == 'rolling']
  queue = [r for r in results if r['kind'] == 'queue']
  print(f"line found at {sum(r['line'] is not None for r in full)}/{len(full)} full stops, "
        f"{sum(r['line'] is not None for r in queue)}/{len(queue)} queue stops, "
        f"{sum(r['line'] is not None for r in rolling)}/{len(rolling)} rolling stops; "
        f"drivers rest with the camera {offset if offset is None else round(offset, 1)} m before the line (median)")


if __name__ == '__main__':
  main()
