# nav2_map_tools

ROS 2 Humble 工作空间：面向 **Nav2** 的禁行区（keepout）编辑、SQLite 存储、costmap 过滤，以及瓦片 / PGM 地图上的路径绘制与 PGM 擦除工具。

GitHub 仓库名为 **[nav2_map_tools](https://github.com/64K-magic/nav2_map_tools)**。

## 功能概览

| 能力 | 说明 |
| :--- | :--- |
| 禁行区 GUI | 在线瓦片或本地 PGM 上绘制线 / 矩形 / 多边形 / 圆，按 `map_name` 写入 SQLite |
| Nav2 过滤 | `filter_keepout` 插件按地图名加载图形，在 costmap 上标为致命障碍（可自膨胀） |
| 路径绘制 | 贝塞尔控制点路径；瓦片用 Origin ENU，PGM 用 YAML `resolution`/`origin` |
| PGM 编辑 | 画笔擦除噪声、保存 P5；可选叠加 `amcl_pose` / `scan` / 障碍点云 |
| GPS 演示 | `nav2_gps_waypoint_follower_demo`：Gazebo + dual EKF + 无静态地图导航 |

## 仓库结构

```
nav2_map_tools/          # 或本地 gps_filter_ws
├── README.md
├── data/                # 运行时数据（如 keepout.db，默认 gitignore）
└── src/
    ├── filter_keepout/              # Nav2 CostmapFilter 插件
    ├── map_coordinates_edit_gui/    # 主 GUI（禁行区 + 路径 + PGM 辅助）
    ├── map_editor/                  # 早期独立 PGM 编辑器（功能已并入 GUI）
    ├── navigation2_tutorials/       # 含 GPS waypoint 演示等
    └── tiledata/                    # 瓦片缓存（可选）
```

核心包说明见各子目录 README：

- [`src/filter_keepout/README.md`](src/filter_keepout/README.md)
- [`src/map_coordinates_edit_gui/README.md`](src/map_coordinates_edit_gui/README.md)
- [`src/navigation2_tutorials/nav2_gps_waypoint_follower_demo/README.md`](src/navigation2_tutorials/nav2_gps_waypoint_follower_demo/README.md)

## 数据流

```
┌────────────────────────────┐
│  map_coordinates_edit_gui  │
│  瓦片 / PGM 上画禁行区     │
└─────────────┬──────────────┘
              │ map 米制 + map_name
              ▼
┌────────────────────────────┐
│  SQLite keepout_figures    │
│  (默认 ~/…/data/keepout.db)│
└─────────────┬──────────────┘
              │ filter_keepout 读库
              ▼
┌────────────────────────────┐
│  Nav2 global costmap       │
│  LETHAL + 可选膨胀         │
└────────────────────────────┘
```

切换导航地图时，GUI **通知 Nav2** 或手动发布：

```bash
ros2 topic pub --once /keepout_refresh std_msgs/msg/String "{data: 'map:map1'}"
ros2 topic pub --once /keepout_refresh std_msgs/msg/String "{data: refresh}"
```

## 坐标系约定

| 底图 | 场景 → map 米制 |
| :--- | :--- |
| **在线瓦片** | 像素 → Web Mercator 经纬度 → 相对界面 **Origin Lat/Lon/Yaw** 的本地 ENU（须与机器人 `navsat_transform` datum 一致） |
| **PGM** | 像素 → 同名 YAML 的 `resolution` / `origin`（ROS map_server 惯例，Y 轴翻转） |

路径文件格式：`x y yaw`（米、弧度）多行，末行 `EOP`。

## 依赖

- ROS 2 **Humble**
- Qt5（Widgets / Network）
- SQLite3
- `robot_localization`、`tf2`、`nav2_costmap_2d` 等（见各包 `package.xml`）
- GPS 演示另需：`gazebo`、相关 Nav2 / mapviz 包（见 demo README）

系统包示例（Ubuntu）：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  qtbase5-dev libsqlite3-dev \
  build-essential cmake
```

## 编译

```bash
cd ~/gps_filter_ws   # 或 clone 后的 nav2_map_tools 目录
source /opt/ros/humble/setup.bash

colcon build --packages-select filter_keepout map_coordinates_edit_gui \
  --cmake-args -DBUILD_STANDALONE=OFF

# GPS 演示（可选）
colcon build --packages-select nav2_gps_waypoint_follower_demo

source install/setup.bash
```

## 快速开始

### 1. 编辑禁行区 / 路径

```bash
source ~/gps_filter_ws/install/setup.bash
ros2 run map_coordinates_edit_gui map_coordinates_edit_gui_node
```

或 standalone：

```bash
cd src/map_coordinates_edit_gui/build
./map_coordinates_edit_gui_node
```

建议流程：

1. 加载 **在线瓦片** 或 **本地 PGM**（需同名 `.yaml`）
2. 瓦片模式填写 **Origin Lat/Lon/Yaw**（与机器人 datum 一致）
3. 设置 **MapName**，绘制禁行区 → **保存到数据库**
4. **⑤ 路径绘制**：贝塞尔路径 → 路径保存 / 打开
5. 需要时 **通知 Nav2 切换地图**

### 2. Nav2 + keepout（GPS 无图演示）

```bash
# 终端 1：仿真世界
ros2 launch nav2_gps_waypoint_follower_demo gazebo_gps_world.launch.py use_sim_time:=True

# 终端 2：Nav2（参数中已挂 filter_keepout，见 nav2_no_map_params.yaml）
ros2 launch nav2_gps_waypoint_follower_demo gps_waypoint_follower.launch.py \
  use_rviz:=True use_sim_time:=True
```

确保 costmap 中 `keepout_filter` 的 `sql_db_path`、`map_name` 与 GUI 保存一致。示例配置：

- `src/filter_keepout/config/costmap_config.yaml`
- `src/navigation2_tutorials/nav2_gps_waypoint_follower_demo/config/nav2_no_map_params.yaml`

## SQLite 表（摘要）

表 `keepout_figures`：一行一个图形；`map_name` 区分导航地图。完整 schema 见 [`src/filter_keepout/sql/schema.sql`](src/filter_keepout/sql/schema.sql)。

## 许可证与维护

- 维护者：见各包 `package.xml`
- 仓库：https://github.com/64K-magic/nav2_map_tools

更多细节请阅读子包 README；GUI / 插件行为以源码与当前参数文件为准。
