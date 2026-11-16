"""Side-road finder on synthetic road tiles: the car's road is matched, followed, and branches found per side."""
import math

import numpy as np
import pytest

from openpilot.starpilot.navigation import side_road
from openpilot.starpilot.navigation.road_tiles import (
  DATA_ZOOM, EXTENT, MAJOR, MINOR, MOTORWAY, STREET, RoadTile, TileKey, lat_lon, meters_per_tile, world_xy,
)

LAT, LON = 40.0, -75.0


def tile_with(lines_m: dict[int, list[list[tuple[float, float]]]]):
  """Build one RoadTile around (LAT, LON) from lines given in metres east/north of that point."""
  cx, cy = world_xy(LAT, LON, DATA_ZOOM)
  key = TileKey(DATA_ZOOM, int(cx), int(cy))
  mpt = meters_per_tile(LAT, DATA_ZOOM)
  out = {}
  for cls, lines in lines_m.items():
    pts, offsets = [], [0]
    for line in lines:
      for e, n in line:
        pts.append(((cx + e / mpt - key.x) * EXTENT, (cy - n / mpt - key.y) * EXTENT))
      offsets.append(len(pts))
    out[cls] = (np.array(pts, dtype=np.float32), np.array(offsets, dtype=np.int32))
  return RoadTile(key, out)


def car_at(east=0.0, north=0.0):
  cx, cy = world_xy(LAT, LON, DATA_ZOOM)
  mpt = meters_per_tile(LAT, DATA_ZOOM)
  return lat_lon(cx + east / mpt, cy - north / mpt, DATA_ZOOM)


# a north-south road, with a street leaving to the west (left when heading north) 80 m ahead
# and a driveway to the east (right) at 40 m
ROADS = {MAJOR: [[(0, -300), (0, 300)]], STREET: [[(0, 80), (-150, 80)]], MINOR: [[(0, 40), (35, 45)]]}


def test_finds_the_left_street_and_the_right_driveway():
  lat, lon = car_at()
  tiles = [tile_with(ROADS)]
  left = side_road.find(tiles, lat, lon, 0.0, 'left')
  right = side_road.find(tiles, lat, lon, 0.0, 'right')
  assert left is not None and left.distance_m == pytest.approx(80, abs=2) and left.angle_deg == pytest.approx(90, abs=5)
  assert left.road_class == 'street'
  assert right is not None and right.distance_m == pytest.approx(40, abs=2) and right.road_class == 'minor'


def test_heading_the_other_way_flips_the_sides_and_hides_roads_behind():
  lat, lon = car_at(north=120)  # past both branches, heading south
  tiles = [tile_with(ROADS)]
  right = side_road.find(tiles, lat, lon, 180.0, 'right')  # heading south, the westward street is on the right
  assert right is not None and right.distance_m == pytest.approx(40, abs=2)
  assert side_road.find(tiles, *car_at(north=120), 0.0, 'left') is None  # heading north again: it is behind


def test_a_crossing_street_counts_on_both_sides():
  tiles = [tile_with({MAJOR: [[(0, -300), (0, 300)]], STREET: [[(-150, 60), (150, 60)]]})]
  lat, lon = car_at()
  assert side_road.find(tiles, lat, lon, 0.0, 'left').distance_m == pytest.approx(60, abs=2)
  assert side_road.find(tiles, lat, lon, 0.0, 'right').distance_m == pytest.approx(60, abs=2)


def test_road_continues_through_split_lines():
  tiles = [tile_with({MAJOR: [[(0, -300), (0, 100)], [(0, 100), (0, 400)]], STREET: [[(0, 180), (-100, 180)]]})]
  found = side_road.find(tiles, *car_at(), 0.0, 'left')
  assert found is not None and found.distance_m == pytest.approx(180, abs=2)


def test_shallow_forks_far_branches_and_highways_are_not_turns():
  lat, lon = car_at()
  shallow = [tile_with({MAJOR: [[(0, -300), (0, 300)]], STREET: [[(0, 50), (-20, 150)]]})]  # ~11 degrees
  assert side_road.find(shallow, lat, lon, 0.0, 'left') is None
  far = [tile_with({MAJOR: [[(0, -300), (0, 400)]], STREET: [[(0, 300), (-100, 300)]]})]
  assert side_road.find(far, lat, lon, 0.0, 'left') is None
  highway = [tile_with({MOTORWAY: [[(0, -300), (0, 300)]], STREET: [[(0, 80), (-150, 80)]]})]
  assert side_road.find(highway, lat, lon, 0.0, 'left') is None  # on a highway the car's road is not matched


def test_unknown_without_a_matching_road():
  lat, lon = car_at(east=60)  # 60 m off any road
  assert side_road.find([tile_with(ROADS)], lat, lon, 0.0, 'left') is None
  assert side_road.find([tile_with(ROADS)], *car_at(), 90.0, 'left') is None  # heading across the road
  assert side_road.find([], *car_at(), 0.0, 'left') is None
  assert side_road.find([tile_with(ROADS)], *car_at(), math.nan, 'left') is None
