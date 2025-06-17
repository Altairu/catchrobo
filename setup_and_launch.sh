#!/bin/bash

# -*- coding: utf-8 -*-
# キャチロボ用ROS2パッケージのアンダーレイ設定、オーバーレイ設定、ビルド、起動を一括で行うスクリプト

# アンダーレイ設定
source /opt/ros/humble/setup.bash

# オーバーレイ設定
source ./install/setup.bash

# ビルド
colcon build --symlink-install

# ビルド結果の確認
echo "ビルドが完了しました。エラーがないか確認してください。"

# launchファイルを正しい場所にコピー
cp ./src/catchrobo_pkg/launch/catchrobo_launch.xml ./install/catchrobo_pkg/share/catchrobo_pkg/

# launchファイルでパッケージを起動
ros2 launch catchrobo_pkg catchrobo_launch.xml