#!/bin/bash

# ROS 2環境のセットアップ
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash

# 各ノードを新しいターミナルウィンドウで実行
gnome-terminal --tab --title="can_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run catchrobo_pkg can_node; exec bash"
gnome-terminal --tab --title="pid_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run catchrobo_pkg pid_node; exec bash"
gnome-terminal --tab --title="planning_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run catchrobo_pkg planning_node; exec bash"
gnome-terminal --tab --title="web_socket_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run catchrobo_pkg web_socket_node; exec bash"
gnome-terminal --tab --title="dualshock3_node" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run catchrobo_pkg dualshock3_node; exec bash"
