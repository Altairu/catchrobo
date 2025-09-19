#!/bin/bash
set -e

WS_DIR="/home/altair/catchrobo"
ROS_SETUP="source /opt/ros/humble/setup.bash; source $WS_DIR/install/setup.bash"

# build
source /opt/ros/humble/setup.bash
#colcon build --symlink-install
source install/setup.bash

# ...existing code...

# gamepad_node を新しいタブで実行
gnome-terminal -- bash -c "$ROS_SETUP; ros2 run catchrobo_pkg gamepad_node; exec bash"

# custom_controller_node を新しいタブで実行
gnome-terminal -- bash -c "$ROS_SETUP; ros2 run catchrobo_pkg custom_controller_node; exec bash"