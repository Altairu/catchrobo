#!/bin/bash

# ROS 2環境のセットアップ
source /opt/ros/humble/setup.bash

# トピック一覧を取得
echo "Fetching all topics..."
ros2 topic list > topics.txt

# 各トピックの詳細を表示
echo "Checking topic details..."
while read -r topic; do
    echo "----------------------------------------"
    echo "Topic: $topic"
    ros2 topic info "$topic"
    ros2 topic type "$topic"
done < topics.txt

# トピック一覧ファイルを削除
rm topics.txt
