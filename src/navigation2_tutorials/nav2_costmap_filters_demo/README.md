# nav2_costmap_filters_demo（Humble / TurtleBot3）

官方 Costmap Filter 示例，已补齐 **TB3 world 对齐的 keepout 掩膜**，可直接联调 Nav2。

> 这是 **PGM 掩膜版** `nav2_costmap_2d::KeepoutFilter`。  
> 你自己的 SQLite 禁行区请用工作区里的 `filter_keepout`（见该包 README）。

## 编译

```bash
cd ~/gps_filter_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nav2_costmap_filters_demo --symlink-install
source install/setup.bash
```

## 一键打印启动命令

```bash
source ~/gps_filter_ws/install/setup.bash
ros2 run --prefix '' bash $(ros2 pkg prefix nav2_costmap_filters_demo)/share/nav2_costmap_filters_demo/run_tb3_keepout_demo.sh
# 或直接：
bash $(ros2 pkg prefix nav2_costmap_filters_demo)/share/nav2_costmap_filters_demo/run_tb3_keepout_demo.sh
```

## 三个终端启动

每个终端先：

```bash
source /opt/ros/humble/setup.bash
source ~/gps_filter_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
# 必须包含系统模型目录，否则 gzserver 会卡住去下 ground_plane/sun，导致 /spawn_entity 超时
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
export GAZEBO_MODEL_DATABASE_URI=   # 禁止联网拉模型
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib
```

若之前启动失败，先清残留：

```bash
pkill -9 gzserver gzclient 2>/dev/null
```

```bash
# 终端 1 — Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 终端 2 — Nav2（已启用 KeepoutFilter）
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=True \
  map:=$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/maps/turtlebot3_world.yaml \
  params_file:=$(ros2 pkg prefix nav2_costmap_filters_demo)/share/nav2_costmap_filters_demo/params/nav2_tb3_keepout_params.yaml

# 终端 3 — 发布 keepout 掩膜 + filter info
ros2 launch nav2_costmap_filters_demo costmap_filter_info.launch.py \
  params_file:=$(ros2 pkg prefix nav2_costmap_filters_demo)/share/nav2_costmap_filters_demo/params/nav2_tb3_keepout_params.yaml \
  mask:=$(ros2 pkg prefix nav2_costmap_filters_demo)/share/nav2_costmap_filters_demo/maps/tb3_keepout_mask.yaml \
  use_sim_time:=True \
  use_composition:=False
```

可选 RViz：

```bash
ros2 launch nav2_bringup rviz_launch.py
```

在 RViz 里 **2D Pose Estimate** 初始化，再 **Nav2 Goal**；路径应绕开地图中心附近的禁行方块。

### Gazebo spawn 失败排查

若日志出现 `Service /spawn_entity unavailable`：

1. `pkill -9 gzserver gzclient` 清掉旧进程  
2. 确认已 export 上面的 `GAZEBO_MODEL_PATH`（含 `/usr/share/gazebo-11/models`）  
3. 再启动；可用 `ros2 service list | grep spawn` 确认 `/spawn_entity` 已出现  

## 文件说明

| 文件 | 作用 |
| :--- | :--- |
| `maps/tb3_keepout_mask.*` | 与 `turtlebot3_world` 同尺寸/原点的禁行掩膜 |
| `params/nav2_tb3_keepout_params.yaml` | 系统 `nav2_params` + KeepoutFilter |
| `launch/costmap_filter_info.launch.py` | mask server + info server |
| `run_tb3_keepout_demo.sh` | 打印上述命令 |
