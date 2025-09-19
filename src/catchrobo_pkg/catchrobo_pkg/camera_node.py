#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        # Publisher
        self.pub = self.create_publisher(Image, "camera/color/image_raw", 10)
        self.bridge = CvBridge()

        # RealSense パイプライン設定
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        try:
            self.pipeline.start(config)
            self.get_logger().info("RealSense pipeline started")
        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense pipeline: {e}")
            raise

        # 30Hz = 約33msごと
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                self.get_logger().warn("No color frame received")
                return

            # NumPy配列に変換
            color_image = np.asanyarray(color_frame.get_data())

            # OpenCV 表示（枠なしフルスクリーン）
            cv2.namedWindow("Color", cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty("Color", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.imshow("Color", color_image)
            cv2.waitKey(1)

            # ROS2 Image メッセージに変換して publish
            msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Camera callback error: {e}")


    def destroy_node(self):
        # ノード終了時に pipeline を stop
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
