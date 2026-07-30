-- Shared keepout geometry table (map-frame meters).
-- One row = one keepout figure (line / rectangle / polygon / circle).
-- Many rows share the same map_name for one navigation map.
-- At runtime, filter_keepout loads only rows matching the current map_name.

CREATE TABLE IF NOT EXISTS keepout_figures (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  figure_type TEXT NOT NULL,   -- 'line' | 'rectangle' | 'polygon' | 'circle'
  figure_name TEXT,
  map_name TEXT NOT NULL,      -- navigation map id (required)
  -- circle (meters in map frame)
  center_x REAL,
  center_y REAL,
  radius REAL,
  -- line / rectangle / polygon vertices as JSON: [[x,y],[x,y],...]
  -- rectangle: 4 corners (TL,TR,BR,BL)
  vertices_json TEXT NOT NULL DEFAULT '[]',
  updated_at TEXT
);

CREATE INDEX IF NOT EXISTS idx_keepout_figures_map_name
  ON keepout_figures(map_name);
