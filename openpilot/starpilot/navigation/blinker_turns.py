"""Blinker turns timed to the road: below the cutoff speed, a blinker waits for the actual turn instead of turning at once.

  map:    the stored road tiles show a road branching off to that side ahead -> go straight until it is close, then turn
  nudge:  no mapped road -> go straight until the driver pushes the wheel toward the blinker, then take the turn over
  turn:   once turning, the turn is held until the car has turned (heading) or the blinker goes off; it never gives up
          halfway. Above the cutoff (plus a margin) or with the blinker off it ends.

The map lookup never runs inside modeld: modeld writes a small request to shared memory and the sideroadd process does
the tile reads, decoding and road search, writing the result back (MapTurnFinder(remote=True)). Decisions here are pure
and use only the latest finished lookup, moved forward by the distance driven since. Settings live in
<starpilot storage>/blinker_turns.json {"mode": "shadow" | "map" | "speed"}:
  shadow  (default) the map/nudge logic decides and logs "would turn here", but the car keeps the old behavior
  map     the map/nudge timing steers
  speed   the old turn-at-once behavior only
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path

from openpilot.common.constants import CV

CUTOFF_MPS = 20 * CV.MPH_TO_MS
ABORT_MARGIN_MPS = 3.0  # a turn in progress ends if the car speeds up this far past the cutoff
TURN_DONE_DEG = 70.0
LOOKUP_PERIOD_S = 0.3
LOOKUP_MAX_AGE_S = 3.0
TILE_RADIUS_M = 320.0
TILE_CACHE = 24
NUDGE_NM = 0.0  # any driver torque toward the blinker while steeringPressed counts as the nudge
MODES = ('shadow', 'map', 'speed')
MAX_CONFIG_BYTES = 4096
REQUEST_PATH = Path('/dev/shm/starpilot_side_road_request.json')
RESULT_PATH = Path('/dev/shm/starpilot_side_road_result.json')
RESULT_READ_S = 0.2


def config_path() -> Path:
  from openpilot.starpilot.storage import starpilot_storage_root
  return starpilot_storage_root() / 'blinker_turns.json'


def mode(path: Path | None = None) -> str:
  try:
    raw = (config_path() if path is None else path).read_bytes()
    doc = json.loads(raw) if len(raw) <= MAX_CONFIG_BYTES else None
  except (OSError, ValueError):
    doc = None
  value = doc.get('mode') if isinstance(doc, dict) else None
  return value if value in MODES else 'shadow'


def trigger_distance(v_ego: float) -> float:
  """How far before the junction (road centreline) the turn starts: about a second of travel, at least 5 m."""
  return max(5.0, v_ego * 1.0 + 3.0)


@dataclass(frozen=True)
class Lookup:
  t: float  # monotonic seconds when the position was taken
  side: str
  road: object | None  # side_road.SideRoad or None (no road on that side ahead / unknown)
  map_known: bool  # tiles existed and the car's road was matched


def _write_json(path: Path, doc: dict) -> None:
  tmp = path.with_name(f'.{path.name}.{os.getpid()}')
  tmp.write_text(json.dumps(doc, separators=(',', ':')))
  os.replace(tmp, path)


def lookup_to_doc(lookup: Lookup) -> dict:
  road = lookup.road
  return {'t': lookup.t, 'side': lookup.side, 'map_known': lookup.map_known,
          'road': None if road is None else {'distance_m': road.distance_m, 'angle_deg': road.angle_deg,
                                             'road_class': road.road_class}}


def lookup_from_doc(doc) -> Lookup | None:
  from openpilot.starpilot.navigation.side_road import SideRoad
  try:
    road = doc['road']
    return Lookup(float(doc['t']), str(doc['side']),
                  None if road is None else SideRoad(float(road['distance_m']), float(road['angle_deg']), str(road['road_class'])),
                  bool(doc['map_known']))
  except (KeyError, TypeError, ValueError):
    return None


class MapTurnFinder:
  """Side-road lookups from the stored road tiles: on a worker thread, or (remote=True, as modeld uses it) in the
  sideroadd process through two small shared-memory files, so the search never competes with the driving model."""

  def __init__(self, store=None, clock=time.monotonic, remote: bool = False,
               request_path: Path = REQUEST_PATH, result_path: Path = RESULT_PATH):
    self.clock = clock
    self._store = store
    self._cache: OrderedDict = OrderedDict()
    self._lock = threading.Lock()
    self._pending = None
    self._latest: Lookup | None = None
    self._event = threading.Event()
    self._last_request = -math.inf
    self._thread = None
    self.remote = remote
    self.request_path, self.result_path = request_path, result_path
    self._last_read = -math.inf

  def request(self, latitude: float, longitude: float, bearing: float, side: str) -> None:
    now = self.clock()
    if now - self._last_request < LOOKUP_PERIOD_S:
      return
    self._last_request = now
    if self.remote:
      try:
        _write_json(self.request_path, {'t': now, 'latitude': latitude, 'longitude': longitude, 'bearing': bearing,
                                        'side': side})
      except OSError:
        pass
      return
    with self._lock:
      self._pending = (now, latitude, longitude, bearing, side)
    if self._thread is None:
      self._thread = threading.Thread(target=self._run, name='blinker-turn-map', daemon=True)
      self._thread.start()
    self._event.set()

  def latest(self, side: str) -> Lookup | None:
    if self.remote and self.clock() - self._last_read >= RESULT_READ_S:
      self._last_read = self.clock()
      try:
        found = lookup_from_doc(json.loads(self.result_path.read_text()))
      except (OSError, ValueError):
        found = None
      with self._lock:
        self._latest = found
    with self._lock:
      found = self._latest
    if found is None or found.side != side or self.clock() - found.t > LOOKUP_MAX_AGE_S:
      return None
    return found

  def lookup_now(self, t, latitude, longitude, bearing, side) -> Lookup:
    """One synchronous lookup (the worker's body; also used by tests and replays)."""
    from openpilot.starpilot.navigation import side_road
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
    road = side_road.find(tiles, latitude, longitude, bearing, side) if tiles else None
    known = bool(tiles) and side_road.matches_road(tiles, latitude, longitude, bearing)
    return Lookup(t, side, road, known)

  def _run(self) -> None:
    while True:
      self._event.wait()
      self._event.clear()
      with self._lock:
        job, self._pending = self._pending, None
      if job is None:
        continue
      try:
        result = self.lookup_now(*job)
      except Exception:  # a bad tile or store must never take the model loop down
        result = Lookup(job[0], job[4], None, False)
      with self._lock:
        self._latest = result


@dataclass
class Decision:
  turning: bool
  reason: str  # why it is (not) turning, for the decision log
  distance_m: float | None = None


class BlinkerTurn:
  """Pure state: waiting for the mapped road, waiting for a nudge, or turning until done."""

  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.side: str | None = None
    self.turning = False
    self.start_heading: float | None = None
    self.driven_m = 0.0  # since the lookup in use
    self.lookup_t: float | None = None
    self.last_t: float | None = None

  def update(self, t: float, side: str | None, v_ego: float, heading: float | None, lookup: Lookup | None,
             nudged: bool) -> Decision:
    dt = 0.0 if self.last_t is None else max(0.0, min(t - self.last_t, 0.5))
    self.last_t = t
    if side is None or side != self.side:
      self.reset()
      self.last_t = t
      self.side = side
      if side is None:
        return Decision(False, '')
    if self.turning:
      turned = (heading is not None and self.start_heading is not None and
                abs((heading - self.start_heading + 180.0) % 360.0 - 180.0) >= TURN_DONE_DEG)
      if turned:
        self.reset()
        self.last_t, self.side = t, side
        return Decision(False, 'turn done')
      if v_ego > CUTOFF_MPS + ABORT_MARGIN_MPS:
        self.turning = False
        return Decision(False, 'too fast for a turn')
      return Decision(True, 'turning')
    if v_ego >= CUTOFF_MPS:
      return Decision(False, 'above turn speed')
    if lookup is not None and lookup.road is not None:
      if lookup.t != self.lookup_t:
        self.lookup_t, self.driven_m = lookup.t, max(0.0, v_ego * (t - lookup.t))
      else:
        self.driven_m += max(v_ego, 0.0) * dt
      distance = lookup.road.distance_m - self.driven_m
      if distance <= trigger_distance(v_ego):
        return self._start(heading, 'map: at the road', distance)
      return Decision(False, 'map: waiting for the road', distance)
    if nudged:
      return self._start(heading, 'nudge', None)
    return Decision(False, 'map: no road there, waiting for a nudge' if lookup is not None and lookup.map_known
                    else 'no map here, waiting for a nudge')

  def _start(self, heading, reason, distance) -> Decision:
    self.turning, self.start_heading = True, heading
    return Decision(True, reason, distance)


def nudged_toward(side: str | None, steering_pressed: bool, steering_torque: float) -> bool:
  """The driver pushing the wheel toward the blinker (positive torque is to the left, as in the lane-change helper)."""
  if side is None or not steering_pressed:
    return False
  return steering_torque > NUDGE_NM if side == 'left' else steering_torque < -NUDGE_NM
