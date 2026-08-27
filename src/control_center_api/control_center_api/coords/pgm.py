"""PGM / map_server frame conversions (matches map_coordinates_edit_gui)."""

from __future__ import annotations


def pixel_to_map(
    px: float,
    py: float,
    *,
    resolution: float,
    origin_x: float,
    origin_y: float,
    height_px: float,
) -> tuple[float, float]:
    """Image pixel (top-left origin) → map-frame meters."""
    x = origin_x + px * resolution
    y = origin_y + (height_px - py) * resolution
    return x, y


def map_to_pixel(
    x: float,
    y: float,
    *,
    resolution: float,
    origin_x: float,
    origin_y: float,
    height_px: float,
) -> tuple[float, float]:
    """Map-frame meters → image pixel (top-left origin)."""
    px = (x - origin_x) / resolution
    py = height_px - (y - origin_y) / resolution
    return px, py


def display_from_map(
    x: float,
    y: float,
    *,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> tuple[float, float]:
    """Map meters → Leaflet CRS.Simple display [lat, lng]."""
    lat = (y - origin_y) / resolution
    lng = (x - origin_x) / resolution
    return lat, lng


def display_to_map(
    lat: float,
    lng: float,
    *,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> tuple[float, float]:
    """Leaflet CRS.Simple display → map-frame meters."""
    x = origin_x + lng * resolution
    y = origin_y + lat * resolution
    return x, y
