#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import pyrealsense2 as rs
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Publisher
        self.pub_color = self.create_publisher(Image, '/camera/color/image_raw', 10)
        # （必要なら深度も追加できる）
        # self.pub_depth = self.create_publisher(Image, '/camera/depth/image_raw', 10)

        # RealSense設定
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # 深度ON
        self.pipeline.start(config)

        self.bridge = CvBridge()

        # タイマーでループ実行
        self.create_timer(0.03, self.timer_callback)  # 約30Hz

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        # OpenCV画像に変換
        img = cv2.cvtColor(
            cv2.imdecode(
                cv2.imencode('.bmp', 
                cv2.UMat(cv2.cvtColor(
                    cv2.cvtColor(cv2.cvtColor(
                        cv2.UMat(cv2.UMat(color_frame.get_data())),
                        cv2.COLOR_BGR2RGB), cv2.COLOR_RGB2BGR), cv2.COLOR_BGR2RGB)), 
                '.bmp')[1].get(),
                cv2.IMREAD_COLOR),
            cv2.COLOR_BGR2RGB
        )

        img = cv2.cvtColor(cv2.UMat(color_frame.get_data()).get(), cv2.COLOR_BGR2RGB)

        # ROS2 Imageに変換してpublish
        msg = self.bridge.cv2_to_imgmsg(img, encoding="rgb8")
        self.pub_color.publish(msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
