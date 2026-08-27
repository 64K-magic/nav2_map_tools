"""Local ENU map-meter conversion (Save/Load path used by Qt GUI)."""

from __future__ import annotations

import math
from typing import Tuple

from keepout_edit_api.models import Point2D

WGS84_A = 6378137.0


def latlon_to_map_meters(
    lat: float,
    lon: float,
    lat0: float,
    lon0: float,
    yaw_rad: float = 0.0,
) -> Point2D:
    """WGS84 → map meters: local ENU (x=east, y=north) then apply yaw."""
    lat0r = math.radians(lat0)
    east = math.radians(lon - lon0) * WGS84_A * math.cos(lat0r)
    north = math.radians(lat - lat0) * WGS84_A
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return Point2D(x=c * east + s * north, y=-s * east + c * north)


def map_meters_to_latlon(
    x: float,
    y: float,
    lat0: float,
    lon0: float,
    yaw_rad: float = 0.0,
) -> Tuple[float, float]:
    """Map meters → WGS84 lat/lon (inverse of latlon_to_map_meters)."""
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    east = c * x - s * y
    north = s * x + c * y
    lat0r = math.radians(lat0)
    lat = lat0 + math.degrees(north / WGS84_A)
    lon = lon0 + math.degrees(east / (WGS84_A * math.cos(lat0r)))
    return lat, lon


def yaw_deg_to_rad(yaw_deg: float) -> float:
    return math.radians(yaw_deg)
