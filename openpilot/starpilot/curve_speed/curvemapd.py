#!/usr/bin/env python3
"""Map curve speed, shadow mode: plans curve speeds from the road map and the curve memory, learns from every curve
driven (above all from the driver taking over), and logs what it would have done. It never publishes control.
Ancillary: it cannot block engagement.

Config <storage>/curve_map.json: {"mode": "shadow"} (default) or {"mode": "off"}.
Logs <storage>/curve_map/shadow/<date>.jsonl, one line per curve driven; memory <storage>/curve_map/curves.json.
"""
from __future__ import annotations

import json
import math
import os
import signal
import threading
import time
from collections import OrderedDict, deque
from dataclasses import asdict
from pathlib import Path

import numpy as np

from openpilot.starpilot.curve_speed import camera, map_curves, map_target
from openpilot.starpilot.navigation.road_tiles import LINK
from openpilot.starpilot.curve_speed.curve_memory import CURVE_K, MIN_SPEED as CURVE_MIN_SPEED, CurveMemory, CurvePass, PassTracker, Sample

SERVICES = ['carState', 'carControl', 'controlsState', 'gpsLocationExternal', 'selfdriveState', 'radarState', 'modelV2']
LOOKUP_PERIOD_S = 2.0
LOOKUP_MOVE_M = 20.0      # no new map lookup until the car has moved this far (parked, crawling)
GPS_MAX_AGE_S = 1.5
GPS_MAX_ACCURACY_M = 15.0
TILE_RADIUS_M = 1100.0
TILE_CACHE = 64
SAVE_PERIOD_S = 60.0
PREDICTION_KEEP_S = 120.0
PREDICTION_MATCH_M = 60.0
LIVE_TIGHTER = 1.15        # the curve measuring this much tighter than predicted would lower the rest of the plan now
LIVE_TORQUE = 0.9          # ...and so would needing this share of the comma's full steering torque
LIVE_WINDOW = 20           # samples (1 s at 20 Hz) in the rolling median of measured/predicted tightness


def storage() -> Path:
  from openpilot.starpilot.storage import starpilot_storage_root
  return starpilot_storage_root()


def mode(root: Path) -> str:
  try:
    value = json.loads((root / 'curve_map.json').read_text()).get('mode')
    return value if value in ('shadow', 'off') else 'shadow'
  except (OSError, ValueError, AttributeError):
    return 'shadow'


def offset_lat_lon(latitude: float, longitude: float, east: float, north: float) -> tuple[float, float]:
  return latitude + north / 111_320.0, longitude + east / (111_320.0 * max(math.cos(math.radians(latitude)), 1e-6))


class Shadow:
  """Pure core: the daemon feeds samples and map curves; this decides, learns and produces log records."""

  def __init__(self, memory: CurveMemory):
    self.memory = memory
    self.tracker = PassTracker()
    self.predictions: deque[dict] = deque()
    self.live_plan: map_target.Plan | None = None  # the newest plan, measured from where it was made
    self.live_t = self.traveled = 0.0
    self.live: dict = {}                          # this curve's live prediction check so far
    self.ratios: deque[float] = deque(maxlen=LIVE_WINDOW)

  def plan(self, t: float, latitude: float, longitude: float, profile: map_curves.RoadProfile, speed: float,
           path: str = 'straight') -> dict | None:
    """Plan the road ahead chunk by chunk and remember what would have been done (for the shadow log)."""
    def memory_of(curve):
      lat, lon = offset_lat_lon(latitude, longitude, curve.apex_east, curve.apex_north)
      return self.memory.remembered(lat, lon, curve.apex_bearing)
    adjust = map_target.adjustments(profile.distances, profile.curves, memory_of, profile.curvature)
    p = map_target.plan(profile.distances, profile.curvature, self.memory.comfort.comfort, adjust,
                        cap_of=self.memory.capability.cap)
    self.live_plan, self.live_t, self.traveled = p, t, 0.0
    b = p.binding
    if b is None:
      return None
    d = float(profile.distances[b])
    curve = next((c for c in profile.curves if c.start_m <= d <= c.end_m), None)
    lat, lon = offset_lat_lon(latitude, longitude, float(profile.points[b, 0]), float(profile.points[b, 1]))
    every_50m = [round(float(v), 1) if math.isfinite(v) else None
                 for v in p.speed[(profile.distances % 50.0 < map_curves.STEP_M / 2) & (profile.distances <= 500.0)]]
    prediction = {'t': t, 'apex_lat': lat, 'apex_lon': lon, 'map_k': round(abs(float(profile.curvature[b])), 5),
                  'used_k': round(float(p.curvature[b]), 5), 'apex_speed': round(float(p.limit[b]), 2),
                  'limit': p.reason[b], 'allowed_now': round(p.now, 2), 'distance_m': round(d, 1),
                  'speed': round(speed, 2), 'would_slow': p.now < speed - 1.0,
                  'known': curve is not None and memory_of(curve) is not None, 'plan_every_50m': every_50m,
                  'path': path}
    self.predictions.append(prediction)
    while self.predictions and t - self.predictions[0]['t'] > PREDICTION_KEEP_S:
      self.predictions.popleft()
    return prediction

  def add(self, s: Sample, experimental: bool) -> dict | None:
    self._live(s)
    p = self.tracker.add(s)
    return None if p is None else self.finish(p, experimental)

  def _live(self, s: Sample) -> None:
    """While driving: learn the steering capability, and check the plan's prediction for the spot the car is on."""
    lateral = abs(s.curvature) * s.v * s.v
    if s.lat_active and not s.steering_pressed and not s.saturated:
      self.memory.capability.observe(s.v, lateral, s.torque)
    if self.live_plan is not None and s.t > self.live_t:
      self.traveled += s.v * min(s.t - self.live_t, 0.2)
      self.live_t = s.t
    if abs(s.curvature) < CURVE_K or s.v < CURVE_MIN_SPEED or self.live_plan is None or not len(self.live_plan.distances):
      return
    predicted = float(np.interp(self.traveled, self.live_plan.distances, self.live_plan.curvature))
    # only where the plan predicts a real curve, and as a 1 s rolling median: at a curve's edges a few metres of
    # position offset would otherwise read as many times tighter than predicted
    if predicted >= CURVE_K:
      self.ratios.append(abs(s.curvature) / predicted)
    tighter = float(np.median(self.ratios)) if len(self.ratios) >= LIVE_WINDOW else None
    need = lateral / max(self.memory.capability.cap(s.v), 1e-3)
    live = self.live
    live['max_tighter'] = max(live.get('max_tighter', 0.0), tighter or 0.0)
    live['max_need'] = max(live.get('max_need', 0.0), need)
    live['max_torque'] = max(live.get('max_torque', 0.0), abs(s.torque) if s.lat_active else 0.0)
    if (tighter is not None and tighter > LIVE_TIGHTER) or need > LIVE_TORQUE:
      # what the rest of this curve would be planned at, knowing what the car measures right now
      fix = map_target.apex_speed(abs(s.curvature), self.memory.comfort.comfort(abs(s.curvature)),
                                  cap_of=self.memory.capability.cap).apex_speed
      if fix < s.v - 0.5 and fix < live.get('lowered_to', math.inf):
        live['lowered_to'] = round(fix, 2)
        live['lowered_at_speed'] = round(s.v, 2)
        live['why'] = 'tighter than predicted' if tighter is not None and tighter > LIVE_TIGHTER else 'near the steering limit'

  def finish(self, p: CurvePass, experimental: bool) -> dict:
    planned = [x for x in self.predictions if x['t'] <= p.start_t + p.duration and
               math.hypot((x['apex_lat'] - p.latitude) * 111_320.0,
                          (x['apex_lon'] - p.longitude) * 111_320.0 * math.cos(math.radians(p.latitude))) <= PREDICTION_MATCH_M]
    record = self.memory.learn(p)
    live, self.live = self.live, {}
    self.ratios.clear()
    line = {'curve': asdict(p), 'experimental': experimental, 'learned_comfort': round(record.comfort, 2),
            'curve_passes': record.passes, 'map_seen': bool(planned),
            'live': {k: round(v, 3) if isinstance(v, float) else v for k, v in live.items()},
            'would_lower_mid_curve': 'lowered_to' in live}
    if planned:
      first, slowest = planned[0], min(planned, key=lambda x: x['allowed_now'])
      line.update({'seen_from_m': first['distance_m'], 'map_k': first['map_k'], 'planned_apex_speed': slowest['apex_speed'],
                   'planned_limit': slowest['limit'], 'would_have_slowed': any(x['would_slow'] for x in planned),
                   'slowest_allowed': slowest['allowed_now'], 'speed_then': slowest['speed']})
    return line


class MapLookup:
  """Background road lookups from the stored tiles, at most once per LOOKUP_PERIOD_S."""

  def __init__(self):
    self._cache: OrderedDict = OrderedDict()
    self._store = None
    self._lock = threading.Lock()
    self._pending = None
    self._latest = None
    self._event = threading.Event()
    threading.Thread(target=self._run, name='curve-map', daemon=True).start()

  def request(self, t, latitude, longitude, bearing, speed, prefer=None) -> None:
    with self._lock:
      self._pending = (t, latitude, longitude, bearing, speed, prefer)
    self._event.set()

  def latest(self):
    with self._lock:
      found, self._latest = self._latest, None
    return found

  def _tiles(self, latitude, longitude):
    from openpilot.starpilot.navigation.road_tiles import TileFormatError, decode_road_tile, tiles_in_radius
    if self._store is None:
      from openpilot.starpilot.navigation.offline_roads import TileStore
      self._store = TileStore()
    tiles = []
    for key in tiles_in_radius(latitude, longitude, TILE_RADIUS_M):
      tile = self._cache.get(key)
      if tile is None:
        raw = self._store.read(key)
        if raw is None:
          continue
        try:
          tile = decode_road_tile(key, raw)
        except TileFormatError:
          continue
        self._cache[key] = tile
        while len(self._cache) > TILE_CACHE:
          self._cache.popitem(last=False)
      else:
        self._cache.move_to_end(key)
      tiles.append(tile)
    return tiles

  def _run(self):
    while True:
      self._event.wait()
      self._event.clear()
      with self._lock:
        job, self._pending = self._pending, None
      if job is None:
        continue
      t, latitude, longitude, bearing, speed, prefer = job
      try:
        branches = map_curves.branch_profiles(self._tiles(latitude, longitude), latitude, longitude, bearing)
      except Exception:  # a bad tile must never take the daemon down
        branches = {}
      path = prefer if prefer in branches else 'straight'
      with self._lock:
        self._latest = (t, latitude, longitude, speed, branches.get(path), path, branches)


class CameraCheck:
  """Shadow-only camera checks at each map lookup: lane position, which branch the camera's path follows at a split,
  and whether the map's curve ahead matches the camera's. Learns the sign conventions it needs from driving."""

  def __init__(self):
    self.camera_sign = camera.SignLearner()   # model yaw-rate curvature vs the car's measured curvature
    self.measured_sign = camera.SignLearner()  # measured curvature vs the map's (left positive)
    self.last_on_link = False

  def observe(self, camera_now: float | None, measured_k: float, map_now: float | None) -> None:
    if camera_now is not None:
      self.camera_sign.observe(measured_k, camera_now)
    if map_now is not None:
      self.measured_sign.observe(map_now, measured_k)

  def camera_left(self, camera_k: float | None) -> float | None:
    a, b = self.camera_sign.factor, self.measured_sign.factor
    return None if camera_k is None or a is None or b is None else camera_k * a * b

  def check(self, t: float, latitude: float, longitude: float, speed: float, blinker: str | None,
            branches: dict, cam: tuple | None, lane: camera.LanePosition) -> dict | None:
    """A log line when there is something to check: a split ahead, a map/camera disagreement, or reaching a ramp."""
    straight = branches.get('straight')
    on_link = straight is not None and straight.road_class == LINK
    reached_ramp, self.last_on_link = on_link and not self.last_on_link, on_link
    cam_raw = camera.near_curvature(*cam) if cam is not None else None
    cam_k = self.camera_left(cam_raw)
    near = {name: camera.near_curvature(p.distances, p.curvature) for name, p in branches.items()}
    split = len(branches) > 1
    agrees = None if cam_k is None or near.get('straight') is None else camera.map_agrees(cam_k, near['straight'])
    if not (split or reached_ramp or agrees is False):
      return None
    says, errors = camera.which_branch(cam_k, near) if cam_k is not None else ('unknown', {})
    r = lambda x: None if x is None else round(float(x), 5)  # noqa: E731
    return {'t': round(t, 2), 'lat': round(latitude, 6), 'lon': round(longitude, 6), 'speed': round(speed, 2),
            'blinker': blinker, 'branches': sorted(branches), 'near_k': {k: r(v) for k, v in near.items()},
            'camera_k': r(cam_k), 'camera_raw_k': r(cam_raw), 'camera_says': says,
            'errors': {k: r(v) for k, v in errors.items()}, 'map_agrees': agrees, 'on_ramp': on_link,
            'reached_ramp': reached_ramp,
            'lane': {'rightmost': lane.rightmost, 'leftmost': lane.leftmost, 'right_edge_m': r(lane.right_edge_m),
                     'left_edge_m': r(lane.left_edge_m), 'outer_right_prob': r(lane.outer_right_prob),
                     'outer_left_prob': r(lane.outer_left_prob)},
            'signs_known': cam_k is not None}


def sample_from(sm) -> Sample:
  cs, cc, ctl, gps = sm['carState'], sm['carControl'], sm['controlsState'], sm['gpsLocationExternal']
  lcs = ctl.lateralControlState
  saturated = lcs.which() == 'torqueState' and bool(lcs.torqueState.saturated)
  return Sample(sm.logMonoTime['carState'] / 1e9, float(cs.vEgo), float(ctl.curvature), bool(cc.latActive),
                bool(cc.longActive), bool(cs.brakePressed), bool(cs.gasPressed), bool(cs.steeringPressed), saturated,
                bool(cs.cruiseState.enabled), float(gps.latitude), float(gps.longitude), float(gps.bearingDeg),
                float(cs.steeringTorque), float(cc.actuators.torque), *lead_of(sm, float(cs.vEgo)))


def lead_of(sm, v: float) -> tuple[float, float]:
  """(time gap to the car ahead in seconds, its speed); (inf, nan) without one."""
  if not sm.seen['radarState']:
    return math.inf, math.nan
  lead = sm['radarState'].leadOne
  if not lead.present or lead.dRel <= 0:  # this cereal's LeadData has `present` (older ones had `status`)
    return math.inf, math.nan
  return float(lead.dRel) / max(v, 1.0), float(lead.vLead)


def main() -> None:
  import openpilot.cereal.messaging as messaging
  from openpilot.common.swaglog import cloudlog

  os.nice(10)
  root = storage()
  if mode(root) == 'off':
    while True:
      time.sleep(3600)
  folder = root / 'curve_map'
  memory = CurveMemory.load(folder / 'curves.json')
  shadow = Shadow(memory)
  lookup = MapLookup()
  cameras = CameraCheck()
  sm = messaging.SubMaster(SERVICES, poll='modelV2')  # 20 Hz: plenty for curves, and no 100 Hz decoding
  last_lookup, last_save, moved = 0.0, time.monotonic(), LOOKUP_MOVE_M
  map_straight, map_traveled = None, 0.0

  def stop(*_):
    raise SystemExit(0)
  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  try:
    while True:
      sm.update()
      if not all(sm.seen[s] for s in ('carState', 'carControl', 'controlsState', 'gpsLocationExternal')):
        continue
      gps = sm['gpsLocationExternal']
      t = sm.logMonoTime['carState'] / 1e9
      gps_ok = (gps.hasFix and gps.horizontalAccuracy <= GPS_MAX_ACCURACY_M and
                t - sm.logMonoTime['gpsLocationExternal'] / 1e9 <= GPS_MAX_AGE_S)
      if not gps_ok:
        continue
      v = float(sm['carState'].vEgo)
      moved += v * 0.05
      if v > 7.0 and t - last_lookup >= LOOKUP_PERIOD_S and moved >= LOOKUP_MOVE_M:
        last_lookup, moved = t, 0.0
        cs = sm['carState']
        prefer = 'right' if cs.rightBlinker and not cs.leftBlinker else 'left' if cs.leftBlinker and not cs.rightBlinker else None
        lookup.request(t, float(gps.latitude), float(gps.longitude), float(gps.bearingDeg), v, prefer)
      found = lookup.latest()
      model = sm['modelV2'] if sm.seen['modelV2'] else None
      cam = camera.camera_curvature(model) if model is not None else None
      measured_k = float(sm['controlsState'].curvature)
      # every frame: the camera against the measured curvature, and the measured curvature against the map at the
      # car's position on the newest map profile (so the sign conventions are learned in seconds, not minutes)
      map_traveled += float(sm['carState'].vEgo) * 0.05
      map_now = (float(np.interp(map_traveled, map_straight.distances, map_straight.curvature))
                 if map_straight is not None and map_traveled <= float(map_straight.distances[-1]) else None)
      cameras.observe(camera.near_curvature(*cam, window=(0.0, 20.0)) if cam is not None else None, measured_k, map_now)
      if found is not None:
        if found[4] is not None:
          shadow.plan(found[0], found[1], found[2], found[4], found[3], found[5])
          map_straight, map_traveled = found[6].get('straight'), 0.0
        cs = sm['carState']
        blinker = 'right' if cs.rightBlinker and not cs.leftBlinker else 'left' if cs.leftBlinker and not cs.rightBlinker else None
        check = cameras.check(found[0], found[1], found[2], found[3], blinker, found[6], cam,
                              camera.lane_position(model) if model is not None else camera.LanePosition(None, None, None, None, False, False))
        if check is not None:
          clog = folder / 'camera' / f"{time.strftime('%Y-%m-%d')}.jsonl"
          clog.parent.mkdir(parents=True, exist_ok=True)
          with open(clog, 'a') as f:
            f.write(json.dumps(check, separators=(',', ':')) + '\n')
      experimental = bool(sm['selfdriveState'].experimentalMode) if sm.seen['selfdriveState'] else False
      line = shadow.add(sample_from(sm), experimental)
      if line is not None:
        log = folder / 'shadow' / f"{time.strftime('%Y-%m-%d')}.jsonl"
        log.parent.mkdir(parents=True, exist_ok=True)
        with open(log, 'a') as f:
          f.write(json.dumps(line, separators=(',', ':')) + '\n')
        cloudlog.event('starpilot_feature', feature='curve_map', verdict=line['curve']['verdict'],
                       map_seen=line['map_seen'], would_have_slowed=line.get('would_have_slowed'))
      if time.monotonic() - last_save >= SAVE_PERIOD_S:
        last_save = time.monotonic()
        memory.save()
  finally:
    memory.save()


if __name__ == '__main__':
  main()
