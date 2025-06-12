# -*- coding: utf-8 -*-
"""
PID制御を行うノード。
ロボマスモーターの制御に使用されます。
"""

import rclpy
from rclpy.node import Node


class PIDController:
    def __init__(self, kp, dead_zone):
        """
        PID制御用のコントローラクラス。

        :param kp: 比例ゲイン
        :param dead_zone: 不感帯
        """
        self.kp = kp
        self.dead_zone = dead_zone
        self.target = 0
        self.origin = None

    def set_target(self, target):
        """
        目標値を設定します。

        :param target: 目標値
        """
        self.target = target

    def control(self, current_value):
        """
        PID制御を実行します。

        :param current_value: 現在の値
        :return: 制御出力
        """
        if self.origin is None:
            self.origin = current_value

        # 誤差計算
        error = self.target - (current_value - self.origin)

        # 不感帯処理
        if abs(error) < self.dead_zone:
            return 0

        # P制御
        output = int(self.kp * error)
        return max(min(output, 16384), -16384)  # 飽和制限


class PIDNode(Node):
    def __init__(self):
        """
        PID制御を行うノード。
        """
        super().__init__('pid_node')

        # 各ロボマスモーターのPIDコントローラを初期化
        self.controllers = {
            1: PIDController(kp=1.0, dead_zone=10),
            2: PIDController(kp=1.2, dead_zone=15),
            3: PIDController(kp=1.5, dead_zone=20),
            4: PIDController(kp=0.8, dead_zone=5),
            5: PIDController(kp=1.1, dead_zone=12),
        }

        # タイマーの実行速度を高速化
        self.timer = self.create_timer(0.01, self.timer_callback)  # 10ms間隔

    def timer_callback(self):
        """
        タイマーコールバックで制御を実行します。
        """
        for motor_id, controller in self.controllers.items():
            # 現在の値を仮に0とする（実際にはセンサーデータを取得）
            current_value = 0

            # 制御出力を計算
            output = controller.control(current_value)

            # 制御出力をログに出力（実際にはモーターに送信）
            self.get_logger().info(f"Motor {motor_id}: Control Output = {output}")


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
