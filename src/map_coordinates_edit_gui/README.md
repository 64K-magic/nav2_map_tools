# map_coordinates_edit_gui

基于 ROS2 Humble + Qt5：在在线瓦片地图上绘制禁行区（线 / 多边形 / 圆），经 `robot_localization` 的 `/fromLL` 转为 map 米制，写入 SQLite，供 `filter_keepout` 在 Nav2 中加载。

## 依赖

- ROS2 Humble：`rclcpp`、`geometry_msgs`、`sensor_msgs`、`geographic_msgs`、`robot_localization`
- Qt5 Widgets / Network
- SQLite3
- 运行保存时需 `navsat_transform_node` 已就绪（`/fromLL`、`/toLL`、`/datum`）

## 目录结构

```
map_coordinates_edit_gui/
├── CMakeLists.txt
├── config/default_config.yaml
├── include/map_coordinates_edit_gui/
│   ├── qnode.hpp
│   └── keepout_db.hpp
├── src/
│   ├── main.cpp
│   ├── qnode.cpp
│   └── keepout_db.cpp
└── build/   # standalone 可执行文件
```

## 数据流

1. 瓦片场景像素 → Web Mercator 经纬度  
2. `/fromLL` → map 米制  
3. 写入 `keepout_figures`（与 `filter_keepout/sql/schema.sql` 一致）  
4. Nav2 `filter_keepout` 读库并栅格化为致命障碍  

## 编译与启动（Standalone）

```bash
source /opt/ros/humble/setup.bash
cd ~/gps_filter_ws/src/map_coordinates_edit_gui
mkdir -p build && cd build
cmake .. -DBUILD_STANDALONE=ON
make -j$(nproc)
./map_coordinates_edit_gui_node
```

或 colcon：

```bash
cd ~/gps_filter_ws
colcon build --packages-select map_coordinates_edit_gui --cmake-args -DBUILD_STANDALONE=OFF
source install/setup.bash
ros2 run map_coordinates_edit_gui map_coordinates_edit_gui_node
```

## 界面布局

- **左侧抽屉**：一级菜单（地图 / 禁行绘制 / 坐标与数据 / 发布），二级为对应控件；可「收起」腾出地图空间
- **主区域**：在线瓦片 / PGM 地图全幅显示；顶部快捷工具条（选择 / 线 / 矩 / 圆 / 多）
- **禁行工具**：点选工具即进入模式，无需再点「开始绘制」；「选择」用于编辑已有图形

## 使用步骤

1. 填写 **Origin Lat/Lon/Yaw**（与机器人 datum 一致），**Apply Datum**  
2. 填写 **MapName**（当前导航地图 ID，如 `warehouse_a`）— 必填  
3. 绘制禁行区；Polygon/Line 用右键或 Enter 结束  
4. **Save to DB (this MapName)**：只替换该地图下的图形行（其它地图不动）  
5. **Load from DB (this MapName)**：只加载该地图的图形  
6. 换图导航时：改 MapName 后点 **Notify Nav2**，或  
   `ros2 topic pub --once /keepout_refresh std_msgs/msg/String "{data: 'map:仓库B'}"`  

> 一行记录 = 一个图形；多张导航地图用不同 `map_name` 隔离。

刷新导航侧禁行层：

```bash
ros2 topic pub --once /keepout_refresh std_msgs/msg/String "{data: refresh}"
```

## 界面字段

| 字段 | 说明 |
| :--- | :--- |
| Origin Lat/Lon/Yaw | 地图原点；Apply Datum → `/datum` |
| DB | SQLite 路径 |
| MapName | **必填**。导航地图 ID；Save/Load/Notify 均按此过滤 |

## 话题

| 方向 | 话题 | 类型 |
| :--- | :--- | :--- |
| 订阅 | `/gps/fix` | `sensor_msgs/NavSatFix` |
| 发布 | `prohibition_areas` | `geometry_msgs/PoseArray`（Publish 时为 map 米制） |

## 参数配置

`config/default_config.yaml`：默认中心经纬度、缩放、天地图瓦片 URL。  
可执行文件从 `../config/default_config.yaml`（相对 `build/`）加载。

## 注意

- 保存前必须 `/fromLL` 可用；否则会提示启动 `navsat_transform`  
- GUI 原点须与导航 `navsat_transform` datum **一致**，否则禁行区在 costmap 上会错位  
- 默认并发瓦片请求已带浏览器 Header；缓存目录：`~/gps_filter_ws/src/tiledata`
