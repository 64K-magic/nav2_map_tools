"""Entry point: FastAPI (uvicorn) keepout edit server. ROS is optional via --ros."""

from __future__ import annotations

import argparse
import logging
import sys
import threading
from pathlib import Path
from typing import Any, Dict, Optional

import uvicorn
import yaml

from control_center_api.api import create_app
from control_center_api.db import KeepoutDatabase
from control_center_api.services import ConvertService, KeepoutService, PgmService, RobotService, parse_footprint

logger = logging.getLogger('control_center_api')


def _load_yaml(path: Path) -> Dict[str, Any]:
    with path.open('r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def _try_ament_share(package: str) -> Optional[Path]:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package))
    except Exception:
        return None


def resolve_config(cli_path: Optional[str] = None) -> Dict[str, Any]:
    candidates = []
    if cli_path:
        candidates.append(Path(cli_path).expanduser())
    share = _try_ament_share('control_center_api')
    if share is not None:
        candidates.append(share / 'config' / 'default.yaml')
    # Package / source-tree config
    here = Path(__file__).resolve().parents[1]
    candidates.append(here / 'config' / 'default.yaml')
    for p in candidates:
        if p.is_file():
            return _load_yaml(p)
    return {}


def resolve_web_root() -> Optional[Path]:
    share = _try_ament_share('control_center_web')
    if share is not None:
        web = share / 'web'
        if web.is_dir():
            return web
    # Source-tree sibling: .../src/control_center_web/web
    sibling = Path(__file__).resolve().parents[2] / 'control_center_web' / 'web'
    if sibling.is_dir():
        return sibling
    return None


def _start_ros_bridge(ros_cfg: Dict[str, Any], ros_args: list):
    """Lazy-import rclpy only when --ros is requested."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    from control_center_api.ros import RosBridge

    rclpy.init(args=ros_args)
    node = Node('control_center_api')
    bridge = RosBridge(
        node,
        from_ll_service=ros_cfg.get('from_ll_service', '/fromLL'),
        to_ll_service=ros_cfg.get('to_ll_service', '/toLL'),
        datum_service=ros_cfg.get('datum_service', '/datum'),
        gps_topic=ros_cfg.get('gps_topic', '/gps/fix'),
        keepout_refresh_topic=ros_cfg.get(
            'keepout_refresh_topic', '/global_costmap/keepout_refresh'
        ),
        keepout_refresh_topics=ros_cfg.get('keepout_refresh_topics'),
        odom_topic=ros_cfg.get('odom_topic', 'odom'),
    )
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return rclpy, node, bridge, executor


def main(argv: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser(
        description='Control Center API (FastAPI). Default: pure Python, no ROS.'
    )
    parser.add_argument('--config', type=str, default=None, help='YAML config path')
    parser.add_argument('--host', type=str, default=None)
    parser.add_argument('--port', type=int, default=None)
    parser.add_argument('--db', type=str, default=None, help='SQLite keepout DB path')
    parser.add_argument(
        '--ros',
        action='store_true',
        help='Enable optional rclpy bridge (fromLL / SetDatum / GPS)',
    )
    # Backward compatible alias
    parser.add_argument(
        '--no-ros',
        action='store_true',
        help=argparse.SUPPRESS,
    )
    known, ros_args = parser.parse_known_args(args=argv)

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
    )

    cfg = resolve_config(known.config)
    http_cfg = cfg.get('http', {})
    host = known.host or http_cfg.get('host', '0.0.0.0')
    port = known.port or int(http_cfg.get('port', 8088))
    db_path = known.db or cfg.get('db_path', '~/gps_filter_ws/data/keepout.db')
    ros_cfg = cfg.get('ros', {})

    use_ros = bool(known.ros) and not known.no_ros

    db = KeepoutDatabase(db_path)
    db.ensure_schema()

    ros_bridge = None
    node = None
    executor = None
    rclpy_mod = None

    if use_ros:
        try:
            rclpy_mod, node, ros_bridge, executor = _start_ros_bridge(ros_cfg, ros_args)
            logger.info('ROS bridge enabled')
        except Exception as exc:
            logger.error('Failed to start ROS bridge: %s — continuing without ROS', exc)
            ros_bridge = None

    convert = ConvertService(ros=ros_bridge)
    keepout = KeepoutService(db=db, convert=convert, ros=ros_bridge)
    robot_cfg = cfg.get('robot', {})
    footprint = parse_footprint(robot_cfg.get('footprint'))
    robot = RobotService(ros=ros_bridge, convert=convert, footprint=footprint)
    pgm_cfg = cfg.get('pgm', {})
    pgm_dir = pgm_cfg.get('maps_dir')
    pgm = PgmService(pgm_dir) if pgm_dir else None
    if pgm is not None:
        logger.info('PGM maps dir: %s', pgm.maps_dir)
    web_root = resolve_web_root()
    if web_root is None:
        logger.warning('Frontend web root not found; API only (no / page)')
    else:
        logger.info('Serving frontend from %s', web_root)

    app = create_app(keepout, convert, robot_service=robot, pgm_service=pgm, config=cfg, web_root=web_root)
    logger.info('Control Center API http://%s:%s  DB=%s', host, port, db.db_path)

    try:
        uvicorn.run(app, host=host, port=port, log_level='info')
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy_mod is not None and rclpy_mod.ok():
            rclpy_mod.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
