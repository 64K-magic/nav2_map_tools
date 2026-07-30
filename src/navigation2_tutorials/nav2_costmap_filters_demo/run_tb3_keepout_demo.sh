#!/usr/bin/env bash
# TurtleBot3 + Nav2 + Keepout Filter demo (Humble)
set -eo pipefail
source /opt/ros/humble/setup.bash
# workspace overlay if built
WS="${HOME}/gps_filter_ws"
if [[ -f "${WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WS}/install/setup.bash"
fi

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models${GAZEBO_MODEL_PATH:+:}${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11${GAZEBO_RESOURCE_PATH:+:}${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib${GAZEBO_PLUGIN_PATH:+:}${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=

DEMO_SHARE="$(ros2 pkg prefix nav2_costmap_filters_demo)/share/nav2_costmap_filters_demo"
PARAMS="${DEMO_SHARE}/params/nav2_tb3_keepout_params.yaml"
MASK="${DEMO_SHARE}/maps/tb3_keepout_mask.yaml"
MAP="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/maps/turtlebot3_world.yaml"

echo "=== Keepout demo ==="
echo "  MODEL=$TURTLEBOT3_MODEL"
echo "  PARAMS=$PARAMS"
echo "  MASK=$MASK"
echo "  MAP=$MAP"
echo ""
echo "Open 3 terminals and run:"
echo ""
echo "# 1) Gazebo world"
echo "export TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo ""
echo "# 2) Nav2 bringup (with KeepoutFilter plugin)"
echo "source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash"
echo "export TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo "ros2 launch nav2_bringup bringup_launch.py \\"
echo "  use_sim_time:=True \\"
echo "  map:=${MAP} \\"
echo "  params_file:=${PARAMS}"
echo ""
echo "# 3) Keepout mask + filter info servers"
echo "source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash"
echo "ros2 launch nav2_costmap_filters_demo costmap_filter_info.launch.py \\"
echo "  params_file:=${PARAMS} \\"
echo "  mask:=${MASK} \\"
echo "  use_sim_time:=True \\"
echo "  use_composition:=False"
echo ""
echo "# Optional RViz"
echo "ros2 launch nav2_bringup rviz_launch.py"
