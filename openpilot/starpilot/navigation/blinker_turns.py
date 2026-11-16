"""Blinker turns timed to the road: below the cutoff speed, a blinker waits for the actual turn instead of turning at once.

  map:    the stored road tiles show a road branching off to that side ahead -> go straight until it is close, then turn
          (from a stop at the line, with no car ahead, within STOP_START_MAX_M of that road: once rolled STOP_ROLL_M)
  nudge:  no mapped road -> go straight until the driver pushes the wheel toward the blinker, then take the turn over
  turn:   once turning, the turn is held until the car has turned (heading) or the blinker goes off; it never gives up
          halfway. Above the cutoff (plus a margin) or with the blinker off it ends.
  one:    a blinker is for one turn (2026-11-16): after a turn is done, or the car has turned without the comma, it looks
          for no more mapped roads until the blinker goes off, and a mapped road further than SIGNAL_AHEAD_M from where
          the blinker came on (queue creep behind a car not counted) is not the one signalled for. Both still take a
          nudge. Unmapped driveways made it wait for, and could have turned at, the next mapped road on that side.

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


TRIGGER_S = 3.2   # seconds of travel before the side road's centreline...
TRIGGER_M = 7.0   # ...plus this, so the turn starts ~1 s before the steering passes 45 degrees


STOP_START_MAX_M = 35.0  # stopped this close to the mapped side road, with no car ahead: stopped at the line
STOP_ROLL_M = 2.0        # pulling away from that stop: the turn starts after this much rolling
STOPPED_MPS = 0.3
SIGNAL_AHEAD_M = 150.0  # a mapped road further than this from where the blinker came on was not signalled for


def trigger_distance(v_ego: float) -> float:
  """How far before the junction (the side road's centreline) the turn starts.

  Fitted on 17 map-timed blinker turns (2026-11-14): at the old 1 s + 3 m the turn began a median 1.7 s after the
  driver's steering had passed 45 degrees (the driver starts turning well before the centreline, and GPS lags);
  3.2 s + 7 m starts it a median 1.3 s before (10-90%: 1.7-0.4 s before). That fit is for rolling turns: pulling away
  from a stop at the line it gives ~17 m at 3 m/s, while the driver started steering 3-5 m after moving off, 20-26 m
  before the side road (two turns from stop signs, 2026-11-15), so those start from the stop instead (STOP_START_MAX_M)."""
  return max(5.0, v_ego * TRIGGER_S + TRIGGER_M)


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
    tiles = self._tiles(latitude, longitude)
    road = side_road.find(tiles, latitude, longitude, bearing, side) if tiles else None
    known = bool(tiles) and side_road.matches_road(tiles, latitude, longitude, bearing)
    return Lookup(t, side, road, known)

  def _tiles(self, latitude, longitude) -> list:
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
    self.odo_m = 0.0  # driven since the blinker came on
    self.stop_odo_m: float | None = None  # odo_m at the last stop with no car ahead (None: none, or queued behind a car)
    self.signal_m = 0.0  # driven since the blinker came on, not counting creep in a queue behind a car
    self.blink_heading: float | None = None  # heading when the blinker came on
    self.map_done: str | None = None  # why this blinker takes no more mapped roads (it waits for a nudge)

  def update(self, t: float, side: str | None, v_ego: float, heading: float | None, lookup: Lookup | None,
             nudged: bool, lead: bool = False) -> Decision:
    """lead: the planner is following a car ahead (a stop behind it is a queue, not the line)."""
    dt = 0.0 if self.last_t is None else max(0.0, min(t - self.last_t, 0.5))
    self.last_t = t
    if side is None or side != self.side:
      self.reset()
      self.last_t = t
      self.side = side
      if side is None:
        return Decision(False, '')
    self.odo_m += max(v_ego, 0.0) * dt
    if not lead:
      self.signal_m += max(v_ego, 0.0) * dt
    if self.blink_heading is None:
      self.blink_heading = heading
    if v_ego < STOPPED_MPS:
      self.stop_odo_m = None if lead else self.odo_m
    if self.turning:
      turned = (heading is not None and self.start_heading is not None and
                abs((heading - self.start_heading + 180.0) % 360.0 - 180.0) >= TURN_DONE_DEG)
      if turned:
        self.reset()
        self.last_t, self.side, self.map_done = t, side, 'turn done'
        return Decision(False, 'turn done')
      if v_ego > CUTOFF_MPS + ABORT_MARGIN_MPS:
        self.turning = False
        return Decision(False, 'too fast for a turn')
      return Decision(True, 'turning')
    if v_ego >= CUTOFF_MPS:
      return Decision(False, 'above turn speed')
    if (self.map_done is None and heading is not None and self.blink_heading is not None and
        abs((heading - self.blink_heading + 180.0) % 360.0 - 180.0) >= TURN_DONE_DEG):
      self.map_done = 'turned without the comma'
    if self.map_done is None and lookup is not None and lookup.road is not None:
      if lookup.t != self.lookup_t:
        self.lookup_t, self.driven_m = lookup.t, max(0.0, v_ego * (t - lookup.t))
      else:
        self.driven_m += max(v_ego, 0.0) * dt
      distance = lookup.road.distance_m - self.driven_m
      if self.signal_m + distance > SIGNAL_AHEAD_M:
        return self._nudge_or_wait(heading, nudged, 'map: road too far for this blinker, waiting for a nudge')
      if nudged and distance > trigger_distance(v_ego):
        return self._start(heading, 'nudge', None)  # the driver's turn comes first: a road the map does not have
      if distance <= trigger_distance(v_ego):
        return self._start(heading, 'map: at the road', distance)
      rolled = self.odo_m - self.stop_odo_m if self.stop_odo_m is not None else None
      if rolled is not None and not lead and rolled >= STOP_ROLL_M and distance + rolled <= STOP_START_MAX_M:
        return self._start(heading, 'map: pulling away from the stop', distance)
      return Decision(False, 'map: waiting for the road', distance)
    if self.map_done is not None:
      return self._nudge_or_wait(heading, nudged, f'{self.map_done}, waiting for the blinker to go off or a nudge')
    if nudged:
      return self._start(heading, 'nudge', None)
    if lookup is None:  # the first moments of a blinker, before sideroadd answers: not yet a missing map
      return Decision(False, 'map: looking up the road, waiting for a nudge')
    return Decision(False, 'map: no road there, waiting for a nudge' if lookup.map_known
                    else 'no map here, waiting for a nudge')

  def _nudge_or_wait(self, heading, nudged, reason) -> Decision:
    return self._start(heading, 'nudge', None) if nudged else Decision(False, reason)

  def _start(self, heading, reason, distance) -> Decision:
    self.turning, self.start_heading = True, heading
    return Decision(True, reason, distance)


def nudged_toward(side: str | None, steering_pressed: bool, steering_torque: float) -> bool:
  """The driver pushing the wheel toward the blinker (positive torque is to the left, as in the lane-change helper)."""
  if side is None or not steering_pressed:
    return False
  return steering_torque > NUDGE_NM if side == 'left' else steering_torque < -NUDGE_NM
