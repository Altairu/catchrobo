#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import pyrealsense2 as rs
import cv2
from cv_bridge import CvBridge
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.bridge = CvBridge()

        self.rs_pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.rs_pipeline.start(config)

        self.timer = self.create_timer(1/30, self.publish_frame)

    def publish_frame(self):
        frames = self.rs_pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return
        # Convert RealSense color frame to numpy array (BGR format per config)
        img = np.asanyarray(color_frame.get_data())
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.rs_pipeline.stop()
        except Exception:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
