# -*- coding: utf-8 -*-
"""ロボットの自動制御を行うノード。
動作番号と特別動作番号に基づきロボットを制御します。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32
from .can_node import CANNode


class PlanningNode(Node):
    """ロボットの動作計画を実行するノード."""

    def __init__(self) -> None:
        super().__init__('planning_node')
        # 動作番号と特別動作番号の初期化
        self.action_number = 0
        self.special_action_number = 0
        self.can_node = CANNode()

        # PID制御用の目標値を送信するパブリッシャ
        self.target_publisher = self.create_publisher(
            Int32MultiArray,
            'target_values',
            10
        )

        # 動作番号を購読するサブスクライバ
        self.action_subscriber = self.create_subscription(
            Int32,
            'action_number',
            self.action_callback,
            10
        )

        # 特別動作番号を購読するサブスクライバ
        self.special_action_subscriber = self.create_subscription(
            Int32,
            'special_action_number',
            self.special_action_callback,
            10
        )

        # タイマーの実行速度を高速化
        self.timer = self.create_timer(0.01, self.timer_callback)  # 10ms間隔

    def action_callback(self, msg):
        """
        動作番号を更新するコールバック。
        """
        self.action_number = msg.data
        self.get_logger().info(f"Received action_number: {self.action_number}")

    def special_action_callback(self, msg):
        """
        特別動作番号を更新するコールバック。
        """
        self.special_action_number = msg.data
        self.get_logger().info(f"Received special_action_number: {self.special_action_number}")

    def timer_callback(self):
        """
        動作番号と特別動作番号に基づきロボットを制御します。
        """
        target_values = Int32MultiArray()

        if self.action_number == 0:
            self.get_logger().info("停止")
            target_values.data = [0, 0, 0, 0, 0]  # ロボマス
            self.can_node.send_data(0x100, [0, 0, 0, 0, 0, 0])  # モーター
            self.can_node.send_data(0x300, [0, 0, 0, 0, 0, 0])  # サーボ1～6
            self.can_node.send_data(0x301, [0, 0, 0, 0, 0, 0])  # サーボ7～12
        elif self.action_number == 1:
            self.get_logger().info("初期状態: CAN通信開始")
            target_values.data = [0, 0, 0, 0, 0]  # ロボマス
            self.can_node.send_data(0x100, [0, 0, 0, 0, 0, 0])  # モーター
            self.can_node.send_data(0x300, [0, 0, 0, 0, 0, 0])  # サーボ1～6
            self.can_node.send_data(0x301, [0, 0, 0, 0, 0, 0])  # サーボ7～12
        elif self.action_number == 2:
            self.get_logger().info("ベルト動作開始: ロボマス4 目標[100RPM]")
            target_values.data = [0, 0, 0, 100, 0]  # ロボマス
            self.can_node.send_data(0x100, [0, 0, 0, 0, 0, 0])  # モーター
            self.can_node.send_data(0x300, [0, 0, 0, 0, 0, 0])  # サーボ1～6
            self.can_node.send_data(0x301, [0, 0, 0, 0, 0, 0])  # サーボ7～12
        elif self.action_number == 3:
            self.get_logger().info("ベルト停止: ロボマス4 目標値0")
            target_values.data = [0, 0, 0, 0, 0]  # ロボマス
            self.can_node.send_data(0x100, [0, 0, 0, 0, 0, 0])  # モーター
            self.can_node.send_data(0x300, [0, 0, 0, 0, 0, 0])  # サーボ1～6
            self.can_node.send_data(0x301, [0, 0, 0, 0, 0, 0])  # サーボ7～12
        else:
            self.get_logger().info("未定義の動作番号")
            target_values.data = [0, 0, 0, 0, 0]  # ロボマス
            self.can_node.send_data(0x100, [0, 0, 0, 0, 0, 0])  # モーター
            self.can_node.send_data(0x300, [0, 0, 0, 0, 0, 0])  # サーボ1～6
            self.can_node.send_data(0x301, [0, 0, 0, 0, 0, 0])  # サーボ7～12

        # 目標値をPIDノードに送信
        self.target_publisher.publish(target_values)

    def execute_special_action(self):
        """
        特別動作番号に基づきロボットを制御します。
        """
        if self.special_action_number == 1:
            self.get_logger().info("妨害機構展開: サーボ1 [10度]")
            self.can_node.send_data(0x300, [10, 0, 0, 0, 0, 0])
        elif self.special_action_number == 2:
            self.get_logger().info("妨害機構収納: サーボ1 [0度]")
            self.can_node.send_data(0x300, [0, 0, 0, 0, 0, 0])
        elif self.special_action_number == 3:
            self.get_logger().info("緊急非常停止（初期化）")
            self.can_node.send_data(0x200, [0, 0, 0, 0, 0, 0])
            self.can_node.send_data(0x100, [0, 0, 0, 0, 0, 0])
            self.can_node.send_data(0x300, [0, 0, 0, 0, 0, 0])
            self.can_node.send_data(0x301, [0, 0, 0, 0, 0, 0])
        elif self.special_action_number == 4:
            self.get_logger().info("緊急非常停止（CAN通信の停止）")
            self.can_node.send_data(0x200, [0, 0, 0, 0, 0, 0])
        else:
            self.get_logger().info("未定義の特別動作番号")
            self.can_node.send_data(0x200, [0, 0, 0, 0, 0, 0])


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
