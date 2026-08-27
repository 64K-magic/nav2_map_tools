"""High-level keepout save / load (DB only; Nav2 reads sql_db_path itself)."""

from __future__ import annotations

from typing import List, Optional, Sequence

from control_center_api.db.keepout_db import KeepoutDatabase
from control_center_api.models import Figure
from control_center_api.ros.bridge import RosBridge
from control_center_api.services.convert_service import ConvertService


class KeepoutService:
    def __init__(
        self,
        db: KeepoutDatabase,
        convert: ConvertService,
        ros: Optional[RosBridge] = None,
    ):
        self.db = db
        self.convert = convert
        self.ros = ros

    def list_maps(self) -> List[str]:
        return self.db.list_map_names()

    def delete_map(self, map_name: str, notify_nav2: bool = False) -> int:
        n = self.db.delete_map(map_name)
        if notify_nav2 and self.ros is not None:
            try:
                self.notify_nav2(map_name)
            except RuntimeError:
                pass
        return n

    def load(self, map_name: str) -> List[Figure]:
        return self.db.load_figures(map_name)

    def load_as_wgs84(
        self,
        map_name: str,
        origin_lat: float,
        origin_lon: float,
        yaw_deg: float = 0.0,
        use_ros: bool = True,
    ) -> List[dict]:
        figs = self.db.load_figures(map_name)
        return [
            self.convert.figure_to_wgs84(f, origin_lat, origin_lon, yaw_deg, use_ros)
            for f in figs
        ]

    def replace_map_figures(self, map_name: str, figures: Sequence[Figure]) -> int:
        named: List[Figure] = []
        for i, f in enumerate(figures, start=1):
            if not f.figure_name:
                f.figure_name = f'fig_{i}'
            f.map_name = map_name
            named.append(f)
        self.db.replace_figures(map_name, named)
        return len(named)

    def save_from_wgs84(
        self,
        map_name: str,
        origin_lat: float,
        origin_lon: float,
        yaw_deg: float,
        figures_wgs84: Sequence[dict],
        use_ros: bool = True,
        notify_nav2: bool = False,
    ) -> int:
        """
        Accept frontend WGS84 figures, convert to map meters, replace DB rows.
        If notify_nav2 and ROS bridge is up, publish keepout_refresh map:<name>.
        """
        converted: List[Figure] = []
        for i, raw in enumerate(figures_wgs84, start=1):
            converted.append(
                self.convert.convert_wgs84_figure_to_map(
                    figure_type=raw['figure_type'],
                    figure_name=raw.get('figure_name') or f'fig_{i}',
                    map_name=map_name,
                    origin_lat=origin_lat,
                    origin_lon=origin_lon,
                    yaw_deg=yaw_deg,
                    vertices_ll=raw.get('vertices_ll'),
                    center_ll=raw.get('center_ll'),
                    radius_m=raw.get('radius_m'),
                    use_ros=use_ros,
                )
            )
        n = self.replace_map_figures(map_name, converted)
        if notify_nav2:
            self.notify_nav2(map_name)
        return n

    def notify_nav2(self, map_name: str) -> None:
        """Hot-reload filter_keepout for this map_name via keepout_refresh."""
        if self.ros is None:
            raise RuntimeError(
                'ROS bridge not available — start API with --ros while Nav2 is running'
            )
        self.ros.publish_keepout_map_switch(map_name)
