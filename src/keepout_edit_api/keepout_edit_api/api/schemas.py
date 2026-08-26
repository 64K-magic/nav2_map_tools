"""Pydantic request/response schemas for the HTTP API."""

from __future__ import annotations

from typing import List, Optional

from pydantic import BaseModel, Field


class OriginModel(BaseModel):
    lat: float
    lon: float
    yaw_deg: float = 0.0


class FigureMapModel(BaseModel):
    """Figure already in map-frame meters."""

    figure_type: str
    figure_name: str = ''
    map_name: str = ''
    center_x: float = 0.0
    center_y: float = 0.0
    radius: float = 0.0
    vertices: List[List[float]] = Field(default_factory=list)
    id: Optional[int] = None


class FigureWgs84Model(BaseModel):
    """Figure in WGS84 for tile-map drawing frontends."""

    figure_type: str
    figure_name: str = ''
    vertices_ll: Optional[List[List[float]]] = None
    center_ll: Optional[List[float]] = None
    radius_m: Optional[float] = None


class ReplaceMapFiguresRequest(BaseModel):
    figures: List[FigureMapModel]


class SaveWgs84Request(BaseModel):
    origin: OriginModel
    figures: List[FigureWgs84Model]
    use_ros: bool = True  # default: /fromLL → map meters for Nav2
    notify_nav2: bool = True  # after save, publish keepout_refresh


class LoadWgs84Request(BaseModel):
    origin: OriginModel
    use_ros: bool = True


class LlToMapRequest(BaseModel):
    lat: float
    lon: float
    origin: OriginModel
    use_ros: bool = True


class MapToLlRequest(BaseModel):
    x: float
    y: float
    origin: OriginModel
    use_ros: bool = True


class BatchLlToMapRequest(BaseModel):
    points: List[List[float]]  # [[lat, lon], ...]
    origin: OriginModel
    use_ros: bool = True


class SetDatumRequest(BaseModel):
    lat: float
    lon: float
    yaw_deg: float = 0.0


class NotifyRequest(BaseModel):
    map_name: str


class ApiMessage(BaseModel):
    ok: bool = True
    message: str = ''
    count: Optional[int] = None
