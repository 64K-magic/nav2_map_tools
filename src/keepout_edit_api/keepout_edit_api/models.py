"""Domain models for keepout figures (map-frame meters)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional


FIGURE_TYPES = ('line', 'rectangle', 'polygon', 'circle')


@dataclass
class Point2D:
    x: float
    y: float

    def as_list(self) -> List[float]:
        return [self.x, self.y]


@dataclass
class Figure:
    """One keepout figure stored in SQLite (same schema as filter_keepout)."""

    figure_type: str
    figure_name: str = ''
    map_name: str = ''
    center_x: float = 0.0
    center_y: float = 0.0
    radius: float = 0.0
    vertices: List[Point2D] = field(default_factory=list)
    id: Optional[int] = None

    def validate(self) -> None:
        if self.figure_type not in FIGURE_TYPES:
            raise ValueError(
                f"figure_type must be one of {FIGURE_TYPES}, got {self.figure_type!r}"
            )
        if self.figure_type == 'circle':
            if self.radius < 1e-3:
                raise ValueError(f'{self.figure_name or "circle"}: radius too small')
            return
        n = len(self.vertices)
        if self.figure_type == 'line' and n < 2:
            raise ValueError(f'{self.figure_name or "line"}: need >= 2 vertices')
        if self.figure_type in ('polygon', 'rectangle') and n < 3:
            raise ValueError(f'{self.figure_name or self.figure_type}: need >= 3 vertices')
        if self.vertices:
            xs = [v.x for v in self.vertices]
            ys = [v.y for v in self.vertices]
            if (max(xs) - min(xs)) < 1e-3 and (max(ys) - min(ys)) < 1e-3:
                raise ValueError(
                    f'{self.figure_name or self.figure_type}: vertices collapsed (~0 span)'
                )
