# -*- coding: utf-8 -*-
"""ロボットの自動制御を行うノード。
動作番号と特別動作番号に基づきロボットを制御します。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray   # ← Int32MultiArray を追加
from .can_node import CANNode


class PlanningNode(Node):
    """ロボットの動作計画を実行するノード."""

    def __init__(self) -> None:
        super().__init__('planning_node')
        # 動作番号と特別動作番号の初期化
        self.action_number = 0
        self.special_action_number = 0
        self.can_node = CANNode()

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

        # リミットスイッチ状態の初期化
        self.limit_switch_states = [0, 0, 0, 0]
        # リミットスイッチを購読
        self.limit_switch_subscriber = self.create_subscription(
            Int32MultiArray,
            'limit_switch_states',
            self.limit_switch_callback,
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

    def limit_switch_callback(self, msg):
        """
        リミットスイッチの状態を更新するコールバック。
        """
        self.limit_switch_states = list(msg.data)
        self.get_logger().info(f"Received limit switches: {self.limit_switch_states}")

    def timer_callback(self):
        """
        動作番号と特別動作番号に基づきロボマス，モーター，サーボの目標値を CAN 送信します。
        """

        if self.action_number == 0:
            self.get_logger().info("停止")
            # 各モーター目標値（1–3,5:mm / 4: RPM）
            rmt = [0, 0, 0, 0, 0]         # ロボマス
            mt  = [10.0, 10.0,  0.0]      # 一般モーター1,2は10mm, 3は0
            smt = [0] * 12                # サーボ全12台を0度
        elif self.action_number == 1:
            self.get_logger().info("初期状態: CAN通信開始")
            rmt = [0, 0, 0, 0, 0]
            mt  = [10, 10, 0]
            smt = [0] * 12
        elif self.action_number == 2:
            self.get_logger().info("ベルト動作開始: ロボマス4 目標[100RPM]")
            rmt = [0, 0, 0, 100, 0]
            mt  = [0, 0, 0]
            smt = [0] * 12
        elif self.action_number == 3:
            self.get_logger().info("ベルト停止: ロボマス4 目標値0")
            rmt = [0, 0, 0, 0, 0]
            mt  = [0, 0, 0]
            smt = [0] * 12
        else:
            self.get_logger().info("未定義の動作番号")
            rmt = [0, 0, 0, 0, 0]
            mt  = [0, 0, 0]
            smt = [0] * 12

        # ロボマス1–4 を ID=0x200，ロボマス5 を ID=0x1FF で送信
        self.can_node.send_data(
            0x200,
            [ rmt[0]>>8, rmt[0]&0xFF,
              rmt[1]>>8, rmt[1]&0xFF,
              rmt[2]>>8, rmt[2]&0xFF,
              rmt[3]>>8, rmt[3]&0xFF ]
        )
        self.can_node.send_data(
            0x1FF,
            [ rmt[4]>>8, rmt[4]&0xFF, 0,0,0,0,0,0 ]
        )

        # 一般モーター1–3 の目標値 (ID=0x300)
        self.can_node.send_data(
            0x300,
            [ int(mt[0])>>8, int(mt[0])&0xFF,
              int(mt[1])>>8, int(mt[1])&0xFF,
              int(mt[2])>>8, int(mt[2])&0xFF,
              0, 0 ]
        )

        # サーボ1–6 の目標値 (ID=0x400)
        self.can_node.send_data(
            0x400,
            smt[0:6] + [0, 0]
        )

        # サーボ7–12 の目標値 (ID=0x401)
        self.can_node.send_data(
            0x401,
            smt[6:12] + [0, 0]
        )

    def execute_special_action(self):
        """
        特別動作番号に基づきロボマス，モーター，サーボの目標値を CAN 送信します。
        """
        # 初期値設定
        rmt = [0] * 5   # ロボマス目標
        mt  = [0] * 3   # 一般モーター目標
        smt = [0] * 12  # サーボ目標

        if self.special_action_number == 1:
            self.get_logger().info("妨害機構展開: サーボ1 [10度]")
            smt[0] = 10
        elif self.special_action_number == 2:
            self.get_logger().info("妨害機構収納: サーボ1 [0度]")
            smt[0] = 0
        elif self.special_action_number == 3:
            self.get_logger().info("緊急非常停止（初期化）")
            # 全機器目標はゼロのまま
        elif self.special_action_number == 4:
            self.get_logger().info("緊急非常停止（CAN通信の停止）")
            # ロボマスのみゼロ停止
        else:
            self.get_logger().info("未定義の特別動作番号")

        # send ロボマス
        self.can_node.send_data(
            0x200,
            [rmt[0]>>8, rmt[0]&0xFF,
             rmt[1]>>8, rmt[1]&0xFF,
             rmt[2]>>8, rmt[2]&0xFF,
             rmt[3]>>8, rmt[3]&0xFF]
        )
        self.can_node.send_data(
            0x1FF,
            [rmt[4]>>8, rmt[4]&0xFF, 0,0,0,0,0,0]
        )

        # send 一般モーター1–3
        self.can_node.send_data(
            0x300,
            [mt[0]>>8, mt[0]&0xFF,
             mt[1]>>8, mt[1]&0xFF,
             mt[2]>>8, mt[2]&0xFF,
             0, 0]
        )

        # send サーボ1–6 / 7–12
        self.can_node.send_data(0x400, smt[0:6]   + [0, 0])
        self.can_node.send_data(0x401, smt[6:12]  + [0, 0])


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
