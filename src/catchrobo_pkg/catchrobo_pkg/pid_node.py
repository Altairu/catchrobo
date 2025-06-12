# -*- coding: utf-8 -*-
"""
PID制御を行うノード。
ロボマスモーターの制御に使用されます。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from .can_node import CANNode


class PIDNode(Node):
    def __init__(self):
        """
        PID制御を行うノード。
        """
        super().__init__('pid_node')

        # 各ロボマスモーターのPIDコントローラを初期化
        self.controllers = {
            1: {'kp': 1.0, 'dead_zone': 10, 'target': 0},
            2: {'kp': 1.2, 'dead_zone': 15, 'target': 0},
            3: {'kp': 1.5, 'dead_zone': 20, 'target': 0},
            4: {'kp': 0.8, 'dead_zone': 5, 'target': 0},
            5: {'kp': 1.1, 'dead_zone': 12, 'target': 0},
        }

        self.can_node = CANNode()

        # サブスクライバで目標値を受け取る
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'target_values',
            self.target_callback,
            10
        )

        # タイマーの実行速度を高速化
        self.timer = self.create_timer(0.01, self.timer_callback)  # 10ms間隔

    def target_callback(self, msg):
        """
        目標値を受け取るコールバック関数。
        """
        for i, target in enumerate(msg.data, start=1):
            if i in self.controllers:
                self.controllers[i]['target'] = target

    def timer_callback(self):
        """
        PID制御を実行し、CAN通信で制御出力を送信します。
        """
        outputs = []
        for motor_id, params in self.controllers.items():
            # 現在の値を仮に0とする（実際にはセンサーデータを取得）
            current_value = 0

            # 誤差計算
            error = params['target'] - current_value

            # 不感帯処理
            if abs(error) < params['dead_zone']:
                output = 0
            else:
                # P制御
                output = int(params['kp'] * error)
                output = max(min(output, 16384), -16384)  # 飽和制限

            outputs.append(output)

        # CAN通信で制御出力を送信
        self.can_node.send_data(0x200, outputs + [0] * (6 - len(outputs)))


def main(args=None):
    """
    ノードのエントリポイント。
    """
    rclpy.init(args=args)
    node = PIDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
