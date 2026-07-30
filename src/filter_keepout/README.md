# filter_keepout

基于 **ROS2 Humble** 的 Nav2 Costmap 禁行区过滤器：从 SQLite 按 **地图名** 加载线 / 多边形 / 圆（map 米制），在 global/local costmap 中标记 `LETHAL_OBSTACLE`。

## 多地图模型

| 概念 | 说明 |
| :--- | :--- |
| 一行记录 | 一个禁行图形（line / rectangle / polygon / circle） |
| `map_name` | 导航地图 ID（必填）；同一地图可有多条图形 |
| 运行时 | 只加载 **当前** `map_name` 的图形 |

切换地图时发布：

```bash
# 切换到 warehouse_b 的禁行区并重新读库
ros2 topic pub --once /keepout_refresh std_msgs/msg/String "{data: 'map:warehouse_b'}"

# 或仅重新加载当前 map_name
ros2 topic pub --once /keepout_refresh std_msgs/msg/String "{data: refresh}"
```

也支持 `map_name=warehouse_b` 或直接 `warehouse_b`。

## 目录结构

```
filter_keepout/
├── CMakeLists.txt
├── package.xml
├── keepout_filter.xml
├── sql/schema.sql
├── config/costmap_config.yaml
├── include/filter_keepout/keepout_filter.hpp
└── src/keepout_filter.cpp
```

## 参数（示例）

见 [`config/costmap_config.yaml`](config/costmap_config.yaml)：

| 参数 | 说明 |
| :--- | :--- |
| `sql_db_path` | SQLite 路径 |
| `map_name` | 启动时加载的导航地图 ID（必填才读库） |
| `refresh_topic` | 默认 `keepout_refresh` |
| `fill_polygons` / `line_thickness` | 栅格化选项 |

## 编译

```bash
cd ~/gps_filter_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select filter_keepout
source install/setup.bash
```

## 与 GUI 的关系

[`map_coordinates_edit_gui`](../map_coordinates_edit_gui) 按 MapName 保存/加载；保存只替换该地图下的行。GUI 的 **Notify Nav2** 会发 `map:<MapName>` 到 `keepout_refresh`。
