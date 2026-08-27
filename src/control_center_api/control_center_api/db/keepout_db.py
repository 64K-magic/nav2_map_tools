"""SQLite persistence for keepout_figures (compatible with filter_keepout)."""

from __future__ import annotations

import json
import os
import sqlite3
from typing import List, Sequence

from control_center_api.models import Figure, Point2D

SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS keepout_figures (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  figure_type TEXT NOT NULL,
  figure_name TEXT,
  map_name TEXT NOT NULL DEFAULT '',
  center_x REAL,
  center_y REAL,
  radius REAL,
  vertices_json TEXT NOT NULL DEFAULT '[]',
  updated_at TEXT
);
CREATE INDEX IF NOT EXISTS idx_keepout_figures_map_name
  ON keepout_figures(map_name);
"""


def expand_user(path: str) -> str:
    if not path:
        return path
    return os.path.expanduser(path)


def vertices_to_json(pts: Sequence[Point2D]) -> str:
    return json.dumps([[p.x, p.y] for p in pts], separators=(',', ':'))


def parse_vertices_json(raw: str) -> List[Point2D]:
    if not raw:
        return []
    data = json.loads(raw)
    out: List[Point2D] = []
    for item in data:
        if len(item) < 2:
            continue
        out.append(Point2D(float(item[0]), float(item[1])))
    return out


class KeepoutDatabase:
    """CRUD scoped by map_name — same contract as C++ keepout_db."""

    def __init__(self, db_path: str):
        self.db_path = expand_user(db_path)

    def ensure_schema(self) -> None:
        parent = os.path.dirname(self.db_path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with sqlite3.connect(self.db_path) as conn:
            conn.executescript(SCHEMA_SQL)
            conn.commit()

    def replace_figures(self, map_name: str, figures: Sequence[Figure]) -> None:
        if not map_name:
            raise ValueError(
                'map_name is required (identify which navigation map these keepouts belong to)'
            )
        self.ensure_schema()
        with sqlite3.connect(self.db_path) as conn:
            conn.execute('BEGIN')
            try:
                conn.execute('DELETE FROM keepout_figures WHERE map_name = ?', (map_name,))
                for f in figures:
                    f.map_name = map_name
                    f.validate()
                    conn.execute(
                        'INSERT INTO keepout_figures '
                        '(figure_type, figure_name, map_name, center_x, center_y, radius, '
                        'vertices_json, updated_at) '
                        "VALUES (?,?,?,?,?,?,?,datetime('now'))",
                        (
                            f.figure_type,
                            f.figure_name,
                            map_name,
                            f.center_x,
                            f.center_y,
                            f.radius,
                            vertices_to_json(f.vertices),
                        ),
                    )
                conn.commit()
            except Exception:
                conn.rollback()
                raise

    def load_figures(self, map_name: str) -> List[Figure]:
        if not map_name:
            raise ValueError('map_name is required to load keepouts for the current navigation map')
        if not os.path.isfile(self.db_path):
            return []
        with sqlite3.connect(self.db_path) as conn:
            cur = conn.execute(
                'SELECT id, figure_type, figure_name, map_name, center_x, center_y, radius, '
                'vertices_json FROM keepout_figures WHERE map_name = ? ORDER BY id',
                (map_name,),
            )
            figures: List[Figure] = []
            for row in cur.fetchall():
                figures.append(
                    Figure(
                        id=row[0],
                        figure_type=row[1] or '',
                        figure_name=row[2] or '',
                        map_name=row[3] or '',
                        center_x=float(row[4] or 0.0),
                        center_y=float(row[5] or 0.0),
                        radius=float(row[6] or 0.0),
                        vertices=parse_vertices_json(row[7] or '[]'),
                    )
                )
            return figures

    def list_map_names(self) -> List[str]:
        if not os.path.isfile(self.db_path):
            return []
        with sqlite3.connect(self.db_path) as conn:
            cur = conn.execute(
                'SELECT DISTINCT map_name FROM keepout_figures '
                "WHERE map_name IS NOT NULL AND map_name != '' "
                'ORDER BY map_name'
            )
            return [r[0] for r in cur.fetchall() if r[0]]

    def delete_map(self, map_name: str) -> int:
        if not map_name:
            raise ValueError('map_name is required')
        self.ensure_schema()
        with sqlite3.connect(self.db_path) as conn:
            cur = conn.execute('DELETE FROM keepout_figures WHERE map_name = ?', (map_name,))
            conn.commit()
            return cur.rowcount
