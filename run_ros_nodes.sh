#!/bin/bash
set -e

WS_DIR="/home/altair/catchrobo"
ROS_SETUP="source /opt/ros/humble/setup.bash; source $WS_DIR/install/setup.bash"

# build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# gamepad_node を新しいタブで実行
wezterm cli spawn --cwd "$WS_DIR" -- \
  bash -lc "$ROS_SETUP; ros2 run catchrobo_pkg gamepad_node; exec bash"

wezterm cli spawn --cwd "$WS_DIR" -- \
  bash -lc "$ROS_SETUP; ros2 run catchrobo_pkg custom_controller_node; exec bash"

# serial_can_gui_node を新しいタブで実行
wezterm cli spawn --cwd "$WS_DIR" -- \
  bash -lc "$ROS_SETUP; ros2 run catchrobo_pkg serial_can_gui_node; exec bash"

# camera_node を新しいタブで実行
#wezterm cli spawn --cwd "$WS_DIR" -- \
#   bash -lc "$ROS_SETUP; ros2 run catchrobo_pkg camera_node; exec bash"
