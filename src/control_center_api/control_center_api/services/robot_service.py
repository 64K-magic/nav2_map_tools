"""Robot pose + footprint overlay for web map (GPS position, odom heading)."""

from __future__ import annotations

import ast
import json
import math
from typing import Any, Dict, List, Optional, Sequence

from control_center_api.coords.enu import yaw_deg_to_rad
from control_center_api.ros.bridge import RosBridge
from control_center_api.services.convert_service import ConvertService


def parse_footprint(raw: Any) -> List[List[float]]:
    if raw is None:
        return []
    if isinstance(raw, str):
        text = raw.strip()
        if not text:
            return []
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            parsed = ast.literal_eval(text)
        raw = parsed
    if not isinstance(raw, (list, tuple)):
        return []
    out: List[List[float]] = []
    for pt in raw:
        if isinstance(pt, (list, tuple)) and len(pt) >= 2:
            out.append([float(pt[0]), float(pt[1])])
    return out


def _rotate_footprint(
    footprint: Sequence[Sequence[float]], map_x: float, map_y: float, yaw_rad: float
) -> List[List[float]]:
    cos_y = math.cos(yaw_rad)
    sin_y = math.sin(yaw_rad)
    out: List[List[float]] = []
    for fx, fy in footprint:
        rx = fx * cos_y - fy * sin_y
        ry = fx * sin_y + fy * cos_y
        out.append([map_x + rx, map_y + ry])
    return out


class RobotService:
    def __init__(
        self,
        ros: Optional[RosBridge],
        convert: ConvertService,
        footprint: Optional[Sequence[Sequence[float]]] = None,
    ):
        self._ros = ros
        self._convert = convert
        self._footprint = list(footprint or [])

    @property
    def footprint(self) -> List[List[float]]:
        return self._footprint

    def get_pose(
        self,
        origin_lat: float,
        origin_lon: float,
        origin_yaw_deg: float = 0.0,
        use_ros: bool = True,
    ) -> Dict[str, Any]:
        ros = self._ros
        if ros is None:
            return {
                'available': False,
                'gps_ok': False,
                'odom_ok': False,
                'message': 'ROS bridge not available (start API with --ros)',
            }

        gps = ros.get_gps_state()
        odom = ros.get_odom_state()
        gps_ok = bool(
            gps
            and math.isfinite(gps.get('lat', float('nan')))
            and math.isfinite(gps.get('lon', float('nan')))
        )
        odom_ok = bool(odom and math.isfinite(odom.get('yaw_rad', float('nan'))))

        if not gps_ok:
            return {
                'available': True,
                'gps_ok': False,
                'odom_ok': odom_ok,
                'lat': gps.get('lat') if gps else None,
                'lon': gps.get('lon') if gps else None,
                'yaw_deg': math.degrees(odom['yaw_rad']) if odom_ok else None,
                'map_x': odom.get('x') if odom else None,
                'map_y': odom.get('y') if odom else None,
                'speed_mps': self._speed(odom),
                'stamp_sec': max(
                    gps.get('stamp', 0.0) if gps else 0.0,
                    odom.get('stamp', 0.0) if odom else 0.0,
                ),
                'footprint_map': [],
                'footprint_ll': [],
                'heading_ll': [],
                'message': 'Waiting for /gps/fix',
            }

        lat = float(gps['lat'])
        lon = float(gps['lon'])
        yaw_rad = float(odom['yaw_rad']) if odom_ok else yaw_deg_to_rad(origin_yaw_deg)

        try:
            if use_ros:
                center = self._convert.ll_to_map_ros(lat, lon)
                map_x, map_y = center.x, center.y
            else:
                center = self._convert.ll_to_map_enu(
                    lat, lon, origin_lat, origin_lon, origin_yaw_deg
                )
                map_x, map_y = center.x, center.y
        except RuntimeError as exc:
            return {
                'available': True,
                'gps_ok': True,
                'odom_ok': odom_ok,
                'lat': lat,
                'lon': lon,
                'yaw_deg': math.degrees(yaw_rad),
                'map_x': None,
                'map_y': None,
                'speed_mps': self._speed(odom),
                'stamp_sec': gps.get('stamp', 0.0),
                'footprint_map': [],
                'footprint_ll': [],
                'heading_ll': [],
                'message': str(exc),
            }

        footprint_map = (
            _rotate_footprint(self._footprint, map_x, map_y, yaw_rad)
            if self._footprint
            else []
        )
        footprint_ll = self._map_ring_to_ll(
            footprint_map, origin_lat, origin_lon, origin_yaw_deg, use_ros
        )

        front_map = [
            map_x + math.cos(yaw_rad) * 1.0,
            map_y + math.sin(yaw_rad) * 1.0,
        ]
        if use_ros:
            h_lat, h_lon, _ = self._convert.map_to_ll_ros(front_map[0], front_map[1])
        else:
            h_lat, h_lon = self._convert.map_to_ll_enu(
                front_map[0], front_map[1], origin_lat, origin_lon, origin_yaw_deg
            )
        heading_ll = [[lat, lon], [h_lat, h_lon]]

        return {
            'available': True,
            'gps_ok': True,
            'odom_ok': odom_ok,
            'lat': lat,
            'lon': lon,
            'yaw_deg': math.degrees(yaw_rad),
            'map_x': map_x,
            'map_y': map_y,
            'speed_mps': self._speed(odom),
            'stamp_sec': max(
                gps.get('stamp', 0.0),
                odom.get('stamp', 0.0) if odom else 0.0,
            ),
            'footprint_map': footprint_map,
            'footprint_ll': footprint_ll,
            'heading_ll': heading_ll,
            'message': '',
        }

    def _map_ring_to_ll(
        self,
        ring_map: Sequence[Sequence[float]],
        origin_lat: float,
        origin_lon: float,
        origin_yaw_deg: float,
        use_ros: bool,
    ) -> List[List[float]]:
        out: List[List[float]] = []
        for x, y in ring_map:
            if use_ros:
                lat, lon, _ = self._convert.map_to_ll_ros(x, y)
            else:
                lat, lon = self._convert.map_to_ll_enu(
                    x, y, origin_lat, origin_lon, origin_yaw_deg
                )
            out.append([lat, lon])
        return out

    @staticmethod
    def _speed(odom: Optional[dict]) -> Optional[float]:
        if not odom:
            return None
        vx = float(odom.get('vx', 0.0))
        vy = float(odom.get('vy', 0.0))
        return math.hypot(vx, vy)
