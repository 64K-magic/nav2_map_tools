"""Coordinate conversion: local ENU (primary) + optional /fromLL /toLL."""

from __future__ import annotations

from typing import Optional, Sequence

from keepout_edit_api.coords.enu import latlon_to_map_meters, map_meters_to_latlon, yaw_deg_to_rad
from keepout_edit_api.models import Figure, Point2D
from keepout_edit_api.ros.bridge import RosBridge


class ConvertService:
    def __init__(self, ros: Optional[RosBridge] = None):
        self._ros = ros

    def ll_to_map_enu(
        self,
        lat: float,
        lon: float,
        origin_lat: float,
        origin_lon: float,
        yaw_deg: float = 0.0,
    ) -> Point2D:
        return latlon_to_map_meters(
            lat, lon, origin_lat, origin_lon, yaw_deg_to_rad(yaw_deg)
        )

    def map_to_ll_enu(
        self,
        x: float,
        y: float,
        origin_lat: float,
        origin_lon: float,
        yaw_deg: float = 0.0,
    ):
        return map_meters_to_latlon(
            x, y, origin_lat, origin_lon, yaw_deg_to_rad(yaw_deg)
        )

    def ll_to_map_ros(self, lat: float, lon: float, alt: float = 0.0) -> Point2D:
        if self._ros is None:
            raise RuntimeError('ROS bridge not available')
        x, y, _ = self._ros.from_ll(lat, lon, alt)
        return Point2D(x, y)

    def map_to_ll_ros(self, x: float, y: float, z: float = 0.0):
        if self._ros is None:
            raise RuntimeError('ROS bridge not available')
        return self._ros.to_ll(x, y, z)

    def convert_wgs84_figure_to_map(
        self,
        figure_type: str,
        figure_name: str,
        map_name: str,
        origin_lat: float,
        origin_lon: float,
        yaw_deg: float = 0.0,
        vertices_ll: Optional[Sequence[Sequence[float]]] = None,
        center_ll: Optional[Sequence[float]] = None,
        radius_m: Optional[float] = None,
        use_ros: bool = True,
    ) -> Figure:
        """Build a map-frame Figure from WGS84 (default: /fromLL)."""
        yaw = yaw_deg_to_rad(yaw_deg)

        def to_map(lat: float, lon: float) -> Point2D:
            if use_ros:
                return self.ll_to_map_ros(lat, lon)
            return latlon_to_map_meters(lat, lon, origin_lat, origin_lon, yaw)

        fig = Figure(
            figure_type=figure_type,
            figure_name=figure_name,
            map_name=map_name,
        )

        if figure_type == 'circle':
            if not center_ll or len(center_ll) < 2:
                raise ValueError('circle requires center_ll [lat, lon]')
            center = to_map(float(center_ll[0]), float(center_ll[1]))
            fig.center_x = center.x
            fig.center_y = center.y
            if radius_m is not None:
                fig.radius = float(radius_m)
            elif vertices_ll and len(vertices_ll) >= 1:
                rim = to_map(float(vertices_ll[0][0]), float(vertices_ll[0][1]))
                fig.radius = ((rim.x - center.x) ** 2 + (rim.y - center.y) ** 2) ** 0.5
            else:
                raise ValueError('circle requires radius_m or a rim lat/lon in vertices_ll')
            fig.validate()
            return fig

        if not vertices_ll:
            raise ValueError(f'{figure_type} requires vertices_ll')
        for pair in vertices_ll:
            fig.vertices.append(to_map(float(pair[0]), float(pair[1])))
        fig.validate()
        return fig

    def figure_to_wgs84(
        self,
        fig: Figure,
        origin_lat: float,
        origin_lon: float,
        yaw_deg: float = 0.0,
        use_ros: bool = True,
    ) -> dict:
        """Return figure with WGS84 coordinates for frontend overlay (default: /toLL)."""
        yaw = yaw_deg_to_rad(yaw_deg)

        def to_ll(x: float, y: float):
            if use_ros:
                lat, lon, _ = self.map_to_ll_ros(x, y)
                return lat, lon
            return map_meters_to_latlon(x, y, origin_lat, origin_lon, yaw)

        out = {
            'id': fig.id,
            'figure_type': fig.figure_type,
            'figure_name': fig.figure_name,
            'map_name': fig.map_name,
            'vertices_ll': [],
            'center_ll': None,
            'radius_m': None,
            'vertices_map': [[v.x, v.y] for v in fig.vertices],
            'center_map': None,
        }
        if fig.figure_type == 'circle':
            lat, lon = to_ll(fig.center_x, fig.center_y)
            out['center_ll'] = [lat, lon]
            out['center_map'] = [fig.center_x, fig.center_y]
            out['radius_m'] = fig.radius
        else:
            for v in fig.vertices:
                lat, lon = to_ll(v.x, v.y)
                out['vertices_ll'].append([lat, lon])
        return out
