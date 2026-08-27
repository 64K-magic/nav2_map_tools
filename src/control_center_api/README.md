# control_center_api

Python FastAPI：瓦片图禁行区（线 / 矩形 / 多边形 / 圆）经坐标转换写入与 Nav2 `filter_keepout` 相同的 SQLite。

前端包：`control_center_web`（由本服务在 `/` 托管）。

- **默认转换**：`robot_localization` 的 `/fromLL`、`/toLL`（需 `--ros`）
- **备选**：本地 ENU（请求里 `use_ros: false`，Origin 须与导航 datum 一致）
- **热刷新**：保存后可选发布 `/global_costmap/keepout_refresh`，并请求清空 global costmap

## 分层

```
control_center_api/
├── models.py                 # Figure / Point2D
├── db/keepout_db.py          # SQLite CRUD（按 map_name）
├── coords/                   # ENU + Mercator
├── ros/bridge.py             # FromLL / ToLL / SetDatum / GPS / keepout_refresh
├── services/
│   ├── convert_service.py
│   └── keepout_service.py
├── api/
│   ├── schemas.py            # 请求/响应模型
│   └── app.py                # 路由
├── config/default.yaml
├── main.py
└── scripts/run_api.py
```

交互文档（Swagger）：服务启动后打开 `http://localhost:8088/docs`

---

## 启动

```bash
cd ~/gps_filter_ws/src/control_center_api
source /opt/ros/humble/setup.bash   # --ros 时需要

pip3 install fastapi uvicorn pydantic pyyaml   # 首次

# 推荐：带 ROS（默认 /fromLL，可热刷新 Nav2）
python3 scripts/run_api.py --ros \
  --host 0.0.0.0 --port 8088 \
  --db ~/gps_filter_ws/data/keepout.db

# 纯 HTTP（无 ROS；转换须 use_ros:false）
python3 scripts/run_api.py --port 8088
```

| 参数 | 说明 |
|------|------|
| `--ros` | 启用 rclpy：`/fromLL`、`/toLL`、`/datum`、GPS、`keepout_refresh` |
| `--host` / `--port` | HTTP 监听（默认见配置） |
| `--db` | SQLite 路径（默认 `~/gps_filter_ws/data/keepout.db`） |
| `--config` | YAML 配置路径 |

浏览器：`http://localhost:8088/`

---

## 与 Nav2

1. `db_path` / `--db` 与 `keepout_filter.sql_db_path` **同一文件**
2. 前端 / API 的 `map_name` 与 `keepout_filter.map_name` 一致（如 `map1`）
3. 热刷新话题：`/global_costmap/keepout_refresh`，payload：`map:<map_name>`

手动刷新示例：

```bash
ros2 topic pub --once /global_costmap/keepout_refresh std_msgs/msg/String "{data: 'map:map1'}"
```

---

## HTTP 接口一览

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/` | 托管前端页面 |
| GET | `/api/health` | 健康检查 / GPS / navsat |
| GET | `/api/config` | 默认原点、瓦片、DB 路径 |
| GET | `/api/maps` | 列出已有 `map_name` |
| DELETE | `/api/maps/{map_name}` | 删除该地图下全部禁行区 |
| GET | `/api/maps/{map_name}/keepouts` | 加载 **map 米制** 图形 |
| PUT | `/api/maps/{map_name}/keepouts` | 用 map 米制图形整图替换 |
| POST | `/api/maps/{map_name}/keepouts/wgs84` | WGS84 图形 → 转换 → 入库（可选热刷新） |
| POST | `/api/maps/{map_name}/keepouts/wgs84/load` | 库内 map 米制 → WGS84（前端回显） |
| POST | `/api/convert/ll-to-map` | 单点 lat/lon → map |
| POST | `/api/convert/map-to-ll` | 单点 map → lat/lon |
| POST | `/api/convert/batch-ll-to-map` | 批量 lat/lon → map |
| POST | `/api/nav2/notify` | 仅热刷新（重读 DB，不改库） |
| POST | `/api/ros/set-datum` | 调用 `/datum`（需 `--ros`） |
| GET | `/api/gps` | 最近 `/gps/fix` |

通用成功消息体：

```json
{ "ok": true, "message": "...", "count": 6 }
```

错误：`4xx/5xx`，`{"detail": "..."}`。

---

## 数据模型

### `figure_type`

`line` | `rectangle` | `polygon` | `circle`

### Map 米制图形（入库 / GET keepouts）

| 字段 | 类型 | 说明 |
|------|------|------|
| `figure_type` | string | 见上 |
| `figure_name` | string | 如 `fig_1` |
| `map_name` | string | 导航地图 ID |
| `vertices` | `[[x,y], ...]` | map 米；线≥2，矩/多≥3；圆可 `[]` |
| `center_x` / `center_y` / `radius` | number | **仅 circle** |
| `id` | int? | 库内主键（只读） |

矩形顶点约定：四角，顺序与编辑器一致（如 TL→TR→BR→BL）。

### WGS84 图形（前端保存/加载）

| 字段 | 类型 | 说明 |
|------|------|------|
| `figure_type` | string | |
| `figure_name` | string | 可选 |
| `vertices_ll` | `[[lat,lon], ...]` | 线/矩/多边形 |
| `center_ll` | `[lat, lon]` | 圆中心 |
| `radius_m` | number | 圆半径（米） |

### Origin

```json
{ "lat": 38.161479, "lon": -122.454630, "yaw_deg": 0.0 }
```

`use_ros: false` 时用 Origin 做本地 ENU；`true` 时走 `/fromLL`（Origin 仍建议与 datum 一致）。

---

## 接口详情

### `GET /api/health`

**响应示例**

```json
{
  "ok": true,
  "db_path": "/home/cdf/gps_filter_ws/data/keepout.db",
  "navsat_ready": true,
  "gps": [38.1615, -122.4546]
}
```

无 `--ros` 时 `navsat_ready` 为 false，`gps` 为 null。

---

### `GET /api/config`

**响应示例**

```json
{
  "default_origin": { "lat": 38.161479, "lon": -122.45463, "yaw_deg": 0.0 },
  "tile": {
    "default_center_lat": 38.161479,
    "default_center_lon": -122.45463,
    "default_zoom": 18,
    "default_tile_url": "https://server.arcgisonline.com/.../tile/{level}/{y}/{x}"
  },
  "db_path": "/home/cdf/gps_filter_ws/data/keepout.db"
}
```

---

### `GET /api/maps`

**响应**：`["map1", "warehouse_a"]`

---

### `DELETE /api/maps/{map_name}`

删除该 `map_name` 下全部行。

| Query | 默认 | 说明 |
|-------|------|------|
| `notify_nav2` | `false` | `true` 时额外发 keepout_refresh |

```bash
curl -X DELETE 'http://localhost:8088/api/maps/map1?notify_nav2=true'
```

---

### `GET /api/maps/{map_name}/keepouts`

返回该地图下 **map 米制** 图形列表（供其它工具直接用库坐标）。

```bash
curl 'http://localhost:8088/api/maps/map1/keepouts'
```

---

### `PUT /api/maps/{map_name}/keepouts`

用已是 map 米制的图形 **整图替换**（先删该 map 下旧行再插入）。

**请求**

```json
{
  "figures": [
    {
      "figure_type": "polygon",
      "figure_name": "fig_1",
      "vertices": [[1.0, 2.0], [3.0, 2.0], [3.0, 4.0]]
    },
    {
      "figure_type": "circle",
      "figure_name": "fig_2",
      "center_x": 0.0,
      "center_y": 0.0,
      "radius": 2.5,
      "vertices": []
    }
  ]
}
```

---

### `POST /api/maps/{map_name}/keepouts/wgs84`

前端主保存接口：WGS84 → map 米制 → SQLite；可选热刷新。

**请求**

```json
{
  "origin": { "lat": 38.161479, "lon": -122.454630, "yaw_deg": 0.0 },
  "use_ros": true,
  "notify_nav2": true,
  "figures": [
    {
      "figure_type": "line",
      "figure_name": "fig_1",
      "vertices_ll": [[38.1615, -122.4546], [38.1616, -122.4545]]
    },
    {
      "figure_type": "rectangle",
      "vertices_ll": [
        [38.1617, -122.4548],
        [38.1617, -122.4544],
        [38.1614, -122.4544],
        [38.1614, -122.4548]
      ]
    },
    {
      "figure_type": "circle",
      "center_ll": [38.1615, -122.4546],
      "radius_m": 5.0
    }
  ]
}
```

| 字段 | 默认 | 说明 |
|------|------|------|
| `use_ros` | `true` | `true`→`/fromLL`；`false`→本地 ENU |
| `notify_nav2` | `true` | 保存后发 `map:<name>` 到 keepout_refresh |

```bash
curl -X POST 'http://localhost:8088/api/maps/map1/keepouts/wgs84' \
  -H 'Content-Type: application/json' \
  -d @save_body.json
```

---

### `POST /api/maps/{map_name}/keepouts/wgs84/load`

读库并转回 WGS84，供瓦片图叠加。

**请求**

```json
{
  "origin": { "lat": 38.161479, "lon": -122.454630, "yaw_deg": 0.0 },
  "use_ros": true
}
```

**响应元素示例**

```json
[
  {
    "id": 1,
    "figure_type": "polygon",
    "figure_name": "fig_1",
    "map_name": "map1",
    "vertices_ll": [[38.16, -122.45], [38.17, -122.45], [38.17, -122.44]],
    "vertices_map": [[1.2, 3.4], [5.6, 3.4], [5.6, 7.8]],
    "center_ll": null,
    "center_map": null,
    "radius_m": null
  }
]
```

---

### `POST /api/convert/ll-to-map`

**请求**

```json
{
  "lat": 38.1615,
  "lon": -122.4546,
  "origin": { "lat": 38.161479, "lon": -122.454630, "yaw_deg": 0.0 },
  "use_ros": true
}
```

**响应**：`{ "x": 1.23, "y": -4.56 }`

---

### `POST /api/convert/map-to-ll`

**请求**

```json
{
  "x": 1.23,
  "y": -4.56,
  "origin": { "lat": 38.161479, "lon": -122.454630, "yaw_deg": 0.0 },
  "use_ros": true
}
```

**响应**：`{ "lat": 38.16, "lon": -122.45, "alt": 0.0 }`

---

### `POST /api/convert/batch-ll-to-map`

**请求**

```json
{
  "points": [[38.1615, -122.4546], [38.1616, -122.4545]],
  "origin": { "lat": 38.161479, "lon": -122.454630, "yaw_deg": 0.0 },
  "use_ros": true
}
```

**响应**：`{ "points": [[x1, y1], [x2, y2]] }`

---

### `POST /api/nav2/notify`

仅通知 Nav2 重读指定地图（**不改数据库**）。未保存的网页修改不会出现在 costmap。

```json
{ "map_name": "map1" }
```

需 `--ros`。发布到 `/global_costmap/keepout_refresh`。

---

### `POST /api/ros/set-datum`

```json
{ "lat": 38.161479, "lon": -122.454630, "yaw_deg": 0.0 }
```

调用 `robot_localization` `/datum`。需 `--ros` 且服务可用。

---

### `GET /api/gps`

```json
{ "lat": 38.1615, "lon": -122.4546 }
```

无 GPS 时 lat/lon 为 `null`。

---

## SQLite 表（与 filter_keepout 一致）

```sql
CREATE TABLE keepout_figures (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  figure_type TEXT NOT NULL,   -- line | rectangle | polygon | circle
  figure_name TEXT,
  map_name TEXT NOT NULL,
  center_x REAL, center_y REAL, radius REAL,
  vertices_json TEXT NOT NULL DEFAULT '[]',  -- [[x,y],...]
  updated_at TEXT
);
```

---

## 配置 `config/default.yaml`

| 键 | 说明 |
|----|------|
| `db_path` | SQLite 路径 |
| `http.host` / `http.port` | 监听地址 |
| `default_origin` | 前端默认 Origin |
| `tile.*` | 默认中心、缩放、瓦片 URL（`{level}/{x}/{y}`） |
| `ros.from_ll_service` 等 | ROS 服务/话题名 |
| `ros.keepout_refresh_topic` | 默认 `/global_costmap/keepout_refresh` |
