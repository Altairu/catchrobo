# -*- coding: utf-8 -*-
"""
PID制御を行うノード。
ロボマスモーターの制御に使用されます。
"""

import rclpy
from rclpy.node import Node


class PIDNode(Node):
    """ロボマスのPID制御を行うノード."""

    def __init__(self, kp, dead_zone) -> None:
        """
        初期化処理。

        :param kp: 比例ゲイン
        :param dead_zone: 不感帯
        """
        super().__init__('pid_node')
        self.kp = kp
        self.dead_zone = dead_zone
        self.target_mm = 0
        self.angle_tracker = None
        self.motor_origin = None

    def set_target(self, target_mm):
        """
        目標値を設定します。

        :param target_mm: 目標値（mm）
        """
        self.target_mm = target_mm

    def control(self, current_angle):
        """
        PID制御を実行します。

        :param current_angle: 現在の角度
        :return: トルク値
        """
        if self.motor_origin is None:
            self.motor_origin = current_angle

        # 目標角度（mm → angle）
        target_angle = int((self.target_mm / 1.0) * 8192)  # 仮の変換値

        # 誤差計算
        error = target_angle - (current_angle - self.motor_origin)

        # 不感帯処理
        if abs(error) < self.dead_zone:
            return 0

        # P制御
        torque = int(self.kp * error / 8192)
        return max(min(torque, 16384), -16384)  # 飽和制限


def main() -> None:
    """ノードエントリポイント."""
    rclpy.init()
    node = PIDNode(kp=1.0, dead_zone=10)
    node.set_target(100)
    torque = node.control(50)
    print(f"Calculated torque: {torque}")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
