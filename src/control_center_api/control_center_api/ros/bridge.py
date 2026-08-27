"""ROS2 bridge: optional FromLL / ToLL / SetDatum / GPS / keepout_refresh."""

from __future__ import annotations

import math
import threading
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from robot_localization.srv import FromLL, SetDatum, ToLL
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

# filter_keepout is a costmap plugin → subscription is namespaced.
DEFAULT_KEEPOUT_REFRESH_TOPICS = (
    '/global_costmap/keepout_refresh',
)


class RosBridge:
    """Optional rclpy helpers for datum / fromLL / Nav2 keepout hot-reload."""

    def __init__(
        self,
        node: Node,
        from_ll_service: str = '/fromLL',
        to_ll_service: str = '/toLL',
        datum_service: str = '/datum',
        gps_topic: str = '/gps/fix',
        keepout_refresh_topic: str = '/global_costmap/keepout_refresh',
        keepout_refresh_topics: Optional[Sequence[str]] = None,
        clear_costmap_service: str = '/global_costmap/clear_entirely_global_costmap',
    ):
        self._node = node
        self._cb_group = ReentrantCallbackGroup()
        self._from_ll = node.create_client(
            FromLL, from_ll_service, callback_group=self._cb_group
        )
        self._to_ll = node.create_client(
            ToLL, to_ll_service, callback_group=self._cb_group
        )
        self._set_datum = node.create_client(
            SetDatum, datum_service, callback_group=self._cb_group
        )
        self._clear_costmap = node.create_client(
            ClearEntireCostmap, clear_costmap_service, callback_group=self._cb_group
        )

        topics: List[str] = []
        for t in DEFAULT_KEEPOUT_REFRESH_TOPICS:
            if t not in topics:
                topics.append(t)
        if keepout_refresh_topic and keepout_refresh_topic not in topics:
            # Normalize relative name used in yaml (keepout_refresh → namespaced)
            if keepout_refresh_topic.strip('/') == 'keepout_refresh':
                pass  # already covered by DEFAULT
            else:
                topics.append(keepout_refresh_topic)
        for t in keepout_refresh_topics or []:
            if t and t not in topics:
                topics.append(t)

        self._keepout_pubs = [node.create_publisher(String, t, 10) for t in topics]
        self._refresh_topics = topics
        node.get_logger().info(
            'keepout_refresh publishers: ' + ', '.join(topics)
        )

        self._last_gps: Optional[Tuple[float, float]] = None
        self._gps_lock = threading.Lock()
        self._gps_sub = node.create_subscription(NavSatFix, gps_topic, self._on_gps, 10)

    def _on_gps(self, msg: NavSatFix) -> None:
        with self._gps_lock:
            self._last_gps = (msg.latitude, msg.longitude)

    def get_last_gps(self) -> Optional[Tuple[float, float]]:
        with self._gps_lock:
            return self._last_gps

    def publish_keepout_map_switch(self, map_name: str, clear_costmap: bool = True) -> None:
        """Tell filter_keepout to reload keepouts for map_name, then clear costmap."""
        if not map_name:
            raise ValueError('map_name is required')
        msg = String()
        msg.data = f'map:{map_name}'
        for pub in self._keepout_pubs:
            pub.publish(msg)
        # Let refreshCallback load SQLite before clear_entirely re-inits the filter.
        time.sleep(0.35)
        if clear_costmap:
            self.clear_global_costmap()

    def publish_keepout_refresh(self, payload: str = 'refresh', clear_costmap: bool = True) -> None:
        msg = String()
        msg.data = payload
        for pub in self._keepout_pubs:
            pub.publish(msg)
        time.sleep(0.35)
        if clear_costmap:
            self.clear_global_costmap()

    def clear_global_costmap(self, timeout_sec: float = 8.0) -> bool:
        """Clear costmap so old keepout cells disappear in RViz after reload."""
        del timeout_sec  # fire-and-forget; waiting often races with costmap reset
        if not self._clear_costmap.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn(
                'clear_entirely_global_costmap not available — '
                'keepout geometry may have reloaded but RViz still shows stale cells'
            )
            return False
        req = ClearEntireCostmap.Request()
        self._clear_costmap.call_async(req)
        # Do not block on the future: clear_entirely re-inits the filter and can
        # stall the client future under sim_time / costmap lock. Request is enough.
        time.sleep(0.2)
        self._node.get_logger().info('Requested clear_entirely_global_costmap')
        return True

    def from_ll(
        self,
        lat: float,
        lon: float,
        alt: float = 0.0,
        timeout_sec: float = 5.0,
    ) -> Tuple[float, float, float]:
        if not self._from_ll.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(
                'fromLL service not available. Start navsat_transform_node and wait until ready.'
            )
        req = FromLL.Request()
        req.ll_point.latitude = lat
        req.ll_point.longitude = lon
        req.ll_point.altitude = alt
        future = self._from_ll.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            raise RuntimeError('fromLL call failed or timed out')
        p = future.result().map_point
        return p.x, p.y, p.z

    def to_ll(
        self,
        x: float,
        y: float,
        z: float = 0.0,
        timeout_sec: float = 5.0,
    ) -> Tuple[float, float, float]:
        if not self._to_ll.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('toLL service not available')
        req = ToLL.Request()
        req.map_point.x = x
        req.map_point.y = y
        req.map_point.z = z
        future = self._to_ll.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            raise RuntimeError('toLL call failed or timed out')
        ll = future.result().ll_point
        return ll.latitude, ll.longitude, ll.altitude

    def set_datum(
        self,
        lat: float,
        lon: float,
        yaw_rad: float = 0.0,
        timeout_sec: float = 5.0,
    ) -> None:
        if not self._set_datum.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('SetDatum service not available')
        req = SetDatum.Request()
        req.geo_pose.position.latitude = lat
        req.geo_pose.position.longitude = lon
        req.geo_pose.position.altitude = 0.0
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        req.geo_pose.orientation.x = 0.0
        req.geo_pose.orientation.y = 0.0
        req.geo_pose.orientation.z = sy
        req.geo_pose.orientation.w = cy
        future = self._set_datum.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if not future.done():
            raise RuntimeError('SetDatum call timed out')

    def navsat_ready(self, timeout_sec: float = 0.2) -> bool:
        return (
            self._from_ll.wait_for_service(timeout_sec=timeout_sec)
            and self._set_datum.wait_for_service(timeout_sec=timeout_sec)
        )
