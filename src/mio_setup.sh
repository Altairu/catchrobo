#!/bin/bash

echo "=== CatchRobo: ビルド開始 ==="
colcon build --symlink-install               # ワークスペースをビルド
echo "=== CatchRobo: ビルド完了 ==="

echo "=== source /opt/ros/humble/setup.bash   ==="
echo "source install/setup.bash ==="