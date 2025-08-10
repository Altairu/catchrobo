#!/bin/bash

# ROS 2環境のセットアップ
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash