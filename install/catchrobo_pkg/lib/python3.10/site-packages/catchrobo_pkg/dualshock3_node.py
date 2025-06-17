# -*- coding: utf-8 -*-
"""
DualShock3コントローラを使用して動作番号を操作するノード。
"""

import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import pygame


class DualShock3Node(Node):
    def __init__(self):
        """
        初期化処理。
        """
        super().__init__('dualshock3_node')
        self.publisher_ = self.create_publisher(Int32, 'action_number', 10)
        os.environ["SDL_JOYSTICK_DEVICE"] = "/dev/input/js0"
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info(f"Joystick initialized: {self.joystick.get_name()}")
        else:
            self.get_logger().error("No joystick found")

        self.action_number = 0  # 初期値を設定
        self.max_action_number = 100  # 動作番号の上限
        self.min_action_number = 0  # 動作番号の下限
        self.button_states = [False, False]  # 丸ボタンと四角ボタンの状態を保持
        self.timer = self.create_timer(0.01, self.timer_callback)  # 10msに一回

    def timer_callback(self):
        """
        タイマーコールバックでボタンの状態を監視します。
        """
        try:
            pygame.event.pump()

            # 丸ボタン
            if self.joystick.get_button(1):
                if not self.button_states[0]:
                    self.button_states[0] = True
                    self.get_logger().info("丸ボタンが押されました")
            else:
                if self.button_states[0]:
                    self.action_number = min(self.action_number + 1, self.max_action_number)  # 上限を超えないように制御
                    self.button_states[0] = False
                    self.get_logger().info("丸ボタンが離されました")

            # 四角ボタン
            if self.joystick.get_button(3):
                if not self.button_states[1]:
                    self.button_states[1] = True
                    self.get_logger().info("四角ボタンが押されました")
            else:
                if self.button_states[1]:
                    self.action_number = max(self.action_number - 1, self.min_action_number)  # 下限を超えないように制御
                    self.button_states[1] = False
                    self.get_logger().info("四角ボタンが離されました")

            # 動作番号をパブリッシュ
            msg = Int32()
            msg.data = self.action_number
            self.publisher_.publish(msg)

            self.get_logger().info(f"Action Number: {self.action_number}")
        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {e}")


def main(args=None):
    """
    ノードのエントリポイント。
    """
    rclpy.init(args=args)
    node = DualShock3Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
