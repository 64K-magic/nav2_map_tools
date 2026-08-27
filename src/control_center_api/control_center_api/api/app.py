"""FastAPI application — HTTP surface over keepout services."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from control_center_api.api.schemas import (
    ApiMessage,
    BatchLlToMapRequest,
    FigureMapModel,
    LlToMapRequest,
    LoadWgs84Request,
    MapToLlRequest,
    NotifyRequest,
    ReplaceMapFiguresRequest,
    SaveWgs84Request,
    SetDatumRequest,
)
from control_center_api.models import Figure, Point2D
from control_center_api.services import ConvertService, KeepoutService, RobotService


def _figure_to_map_model(f: Figure) -> FigureMapModel:
    return FigureMapModel(
        id=f.id,
        figure_type=f.figure_type,
        figure_name=f.figure_name,
        map_name=f.map_name,
        center_x=f.center_x,
        center_y=f.center_y,
        radius=f.radius,
        vertices=[[v.x, v.y] for v in f.vertices],
    )


def _map_model_to_figure(m: FigureMapModel, map_name: str) -> Figure:
    return Figure(
        id=m.id,
        figure_type=m.figure_type,
        figure_name=m.figure_name,
        map_name=map_name or m.map_name,
        center_x=m.center_x,
        center_y=m.center_y,
        radius=m.radius,
        vertices=[Point2D(v[0], v[1]) for v in m.vertices],
    )


def create_app(
    keepout_service: KeepoutService,
    convert_service: ConvertService,
    robot_service: Optional[RobotService] = None,
    config: Optional[Dict[str, Any]] = None,
    web_root: Optional[Path] = None,
) -> FastAPI:
    app = FastAPI(
        title='Control Center API',
        version='0.1.0',
        description='RTK tile-map keepout draw → map meters → SQLite for Nav2 filter_keepout',
    )
    app.add_middleware(
        CORSMiddleware,
        allow_origins=['*'],
        allow_credentials=True,
        allow_methods=['*'],
        allow_headers=['*'],
    )
    app.state.keepout = keepout_service
    app.state.convert = convert_service
    app.state.robot = robot_service
    app.state.config = config or {}

    @app.get('/api/health')
    def health():
        ros = keepout_service.ros
        gps = ros.get_last_gps() if ros else None
        odom = ros.get_odom_state() if ros else None
        return {
            'ok': True,
            'db_path': keepout_service.db.db_path,
            'navsat_ready': bool(ros and ros.navsat_ready()),
            'ros_enabled': ros is not None,
            'gps': gps,
            'odom_ok': bool(odom),
        }

    @app.get('/api/config')
    def get_config():
        cfg = app.state.config
        robot_cfg = cfg.get('robot', {})
        return {
            'default_origin': cfg.get('default_origin', {}),
            'tile': cfg.get('tile', {}),
            'db_path': keepout_service.db.db_path,
            'robot': {
                'footprint': robot_service.footprint if robot_service else [],
                'track_poll_hz': robot_cfg.get('track_poll_hz', 5),
            },
        }

    @app.get('/api/maps')
    def list_maps() -> List[str]:
        return keepout_service.list_maps()

    @app.delete('/api/maps/{map_name}', response_model=ApiMessage)
    def delete_map(map_name: str, notify_nav2: bool = False):
        try:
            n = keepout_service.delete_map(map_name, notify_nav2=notify_nav2)
            return ApiMessage(
                ok=True,
                message=f'Deleted {n} figures for map {map_name}',
                count=n,
            )
        except ValueError as e:
            raise HTTPException(400, str(e)) from e

    @app.get('/api/maps/{map_name}/keepouts', response_model=List[FigureMapModel])
    def load_keepouts(map_name: str):
        try:
            return [_figure_to_map_model(f) for f in keepout_service.load(map_name)]
        except ValueError as e:
            raise HTTPException(400, str(e)) from e

    @app.put('/api/maps/{map_name}/keepouts', response_model=ApiMessage)
    def replace_keepouts(map_name: str, body: ReplaceMapFiguresRequest):
        try:
            figs = [_map_model_to_figure(m, map_name) for m in body.figures]
            n = keepout_service.replace_map_figures(map_name, figs)
            return ApiMessage(ok=True, message=f'Replaced {n} figures for {map_name}', count=n)
        except ValueError as e:
            raise HTTPException(400, str(e)) from e

    @app.post('/api/maps/{map_name}/keepouts/wgs84', response_model=ApiMessage)
    def save_wgs84(map_name: str, body: SaveWgs84Request):
        try:
            n = keepout_service.save_from_wgs84(
                map_name=map_name,
                origin_lat=body.origin.lat,
                origin_lon=body.origin.lon,
                yaw_deg=body.origin.yaw_deg,
                figures_wgs84=[f.model_dump() for f in body.figures],
                use_ros=body.use_ros,
                notify_nav2=body.notify_nav2,
            )
            extra = ' + keepout_refresh' if body.notify_nav2 else ''
            return ApiMessage(
                ok=True,
                message=f'Saved {n} figures for {map_name}{extra}',
                count=n,
            )
        except (ValueError, RuntimeError) as e:
            raise HTTPException(400, str(e)) from e

    @app.post('/api/maps/{map_name}/keepouts/wgs84/load')
    def load_wgs84(map_name: str, body: LoadWgs84Request):
        try:
            return keepout_service.load_as_wgs84(
                map_name,
                body.origin.lat,
                body.origin.lon,
                body.origin.yaw_deg,
                body.use_ros,
            )
        except ValueError as e:
            raise HTTPException(400, str(e)) from e

    @app.post('/api/convert/ll-to-map')
    def ll_to_map(body: LlToMapRequest):
        try:
            if body.use_ros:
                p = convert_service.ll_to_map_ros(body.lat, body.lon)
            else:
                p = convert_service.ll_to_map_enu(
                    body.lat, body.lon, body.origin.lat, body.origin.lon, body.origin.yaw_deg
                )
            return {'x': p.x, 'y': p.y}
        except RuntimeError as e:
            raise HTTPException(503, str(e)) from e

    @app.post('/api/convert/map-to-ll')
    def map_to_ll(body: MapToLlRequest):
        try:
            if body.use_ros:
                lat, lon, alt = convert_service.map_to_ll_ros(body.x, body.y)
                return {'lat': lat, 'lon': lon, 'alt': alt}
            lat, lon = convert_service.map_to_ll_enu(
                body.x, body.y, body.origin.lat, body.origin.lon, body.origin.yaw_deg
            )
            return {'lat': lat, 'lon': lon, 'alt': 0.0}
        except RuntimeError as e:
            raise HTTPException(503, str(e)) from e

    @app.post('/api/convert/batch-ll-to-map')
    def batch_ll_to_map(body: BatchLlToMapRequest):
        out = []
        try:
            for pair in body.points:
                lat, lon = float(pair[0]), float(pair[1])
                if body.use_ros:
                    p = convert_service.ll_to_map_ros(lat, lon)
                else:
                    p = convert_service.ll_to_map_enu(
                        lat, lon, body.origin.lat, body.origin.lon, body.origin.yaw_deg
                    )
                out.append([p.x, p.y])
            return {'points': out}
        except RuntimeError as e:
            raise HTTPException(503, str(e)) from e

    @app.post('/api/nav2/notify', response_model=ApiMessage)
    def notify_nav2(body: NotifyRequest):
        try:
            keepout_service.notify_nav2(body.map_name)
            return ApiMessage(
                ok=True,
                message=f'Published keepout_refresh map:{body.map_name}',
            )
        except (ValueError, RuntimeError) as e:
            raise HTTPException(400, str(e)) from e

    @app.post('/api/ros/set-datum', response_model=ApiMessage)
    def set_datum(body: SetDatumRequest):
        ros = keepout_service.ros
        if ros is None:
            raise HTTPException(503, 'ROS bridge not available')
        try:
            import math

            ros.set_datum(body.lat, body.lon, math.radians(body.yaw_deg))
            return ApiMessage(ok=True, message='SetDatum OK')
        except RuntimeError as e:
            raise HTTPException(503, str(e)) from e

    @app.get('/api/gps')
    def gps():
        ros = keepout_service.ros
        if ros is None:
            return {'lat': None, 'lon': None}
        g = ros.get_last_gps()
        if not g:
            return {'lat': None, 'lon': None}
        return {'lat': g[0], 'lon': g[1]}

    @app.get('/api/robot/pose')
    def robot_pose(
        lat: float,
        lon: float,
        yaw_deg: float = 0.0,
        use_ros: bool = True,
    ):
        if robot_service is None:
            return {
                'available': False,
                'gps_ok': False,
                'odom_ok': False,
                'message': 'ROS bridge not available (start API with --ros)',
            }
        return robot_service.get_pose(lat, lon, yaw_deg, use_ros=use_ros)

    if web_root and web_root.is_dir():
        # colcon --symlink-install uses symlinks; Starlette blocks them unless enabled.
        static_kw = {'follow_symlink': True}
        assets = web_root / 'css'
        js = web_root / 'js'
        if assets.is_dir():
            app.mount('/css', StaticFiles(directory=str(assets), **static_kw), name='css')
        if js.is_dir():
            app.mount('/js', StaticFiles(directory=str(js), **static_kw), name='js')

        index = web_root / 'index.html'

        @app.get('/')
        def index_page():
            if index.is_file():
                return FileResponse(str(index))
            raise HTTPException(404, 'Frontend not installed (control_center_web)')

    return app
