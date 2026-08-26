"""Web Mercator / XYZ tile helpers (same formulas as map_coordinates_edit_gui)."""

from __future__ import annotations

import math
from typing import Tuple


def lon_to_tile_x(lon: float, z: int) -> float:
    return (lon + 180.0) / 360.0 * (1 << z)


def lat_to_tile_y(lat: float, z: int) -> float:
    latrad = math.radians(lat)
    return (1.0 - math.asinh(math.tan(latrad)) / math.pi) / 2.0 * (1 << z)


def tile_x_to_lon(x: float, z: int) -> float:
    return x / float(1 << z) * 360.0 - 180.0


def tile_y_to_lat(y: float, z: int) -> float:
    n = math.pi - (y / float(1 << z)) * 2.0 * math.pi
    return math.degrees(math.atan(math.sinh(n)))


def lon_to_tile_x_int(lon: float, z: int) -> int:
    return int(math.floor(lon_to_tile_x(lon, z)))


def lat_to_tile_y_int(lat: float, z: int) -> int:
    return int(math.floor(lat_to_tile_y(lat, z)))


def scene_to_latlon(
    scene_x: float,
    scene_y: float,
    min_tile_x: int,
    min_tile_y: int,
    zoom: int,
    tile_size: int = 256,
) -> Tuple[float, float]:
    """Scene pixel (mosaic top-left origin) → lat/lon."""
    tile_x = scene_x / tile_size + min_tile_x
    tile_y = scene_y / tile_size + min_tile_y
    lon = tile_x_to_lon(tile_x, zoom)
    lat = tile_y_to_lat(tile_y, zoom)
    return lat, lon


def latlon_to_scene(
    lat: float,
    lon: float,
    min_tile_x: int,
    min_tile_y: int,
    zoom: int,
    tile_size: int = 256,
) -> Tuple[float, float]:
    """Lat/lon → scene pixel."""
    tile_x = lon_to_tile_x(lon, zoom)
    tile_y = lat_to_tile_y(lat, zoom)
    return (tile_x - min_tile_x) * tile_size, (tile_y - min_tile_y) * tile_size


def build_tile_url(template: str, zoom: int, x: int, y: int) -> str:
    """Substitute {level}/{x}/{y} (and {z} alias)."""
    url = template.strip().strip('"')
    url = (
        url.replace('{level}', str(zoom))
        .replace('{z}', str(zoom))
        .replace('{x}', str(x))
        .replace('{y}', str(y))
    )
    return url
