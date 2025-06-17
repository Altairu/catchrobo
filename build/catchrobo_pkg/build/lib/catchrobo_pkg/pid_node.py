# -*- coding: utf-8 -*-
"""
PID制御を行うノード。
ロボマスモーターの制御に使用されます。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from .can_node import CANNode
import array


class PIDNode(Node):
    def __init__(self):
        """
        PID制御を行うノード。
        """
        super().__init__('pid_node')

        # 各ロボマスモーターのPIDコントローラを初期化
        self.controllers = {
            1: {'kp': 20.0, 'dead_zone': 100, 'target': 0},
            2: {'kp': 20.0, 'dead_zone': 100, 'target': 0},
            3: {'kp': 20.0, 'dead_zone': 100, 'target': 0},
            4: {'kp': 10.0, 'dead_zone': 0, 'target': 0},
            5: {'kp': 20.0, 'dead_zone': 100, 'target': 0},
        }

        # 制御量を送信するためのパブリッシャ
        self.control_publisher = self.create_publisher(
            Int32MultiArray,
            'control_values',
            10
        )

        # サブスクライバで目標値を受け取る
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'target_values',
            self.target_callback,
            10
        )

        # 各モーターの現在値を受信するためのサブスクライバ
        self.motor_subscriptions = {
            1: self.create_subscription(Int32MultiArray, 'motor_1_mm', self.motor_callback(1), 10),
            2: self.create_subscription(Int32MultiArray, 'motor_2_mm', self.motor_callback(2), 10),
            3: self.create_subscription(Int32MultiArray, 'motor_3_mm', self.motor_callback(3), 10),
            4: self.create_subscription(Int32MultiArray, 'motor_4_rpm', self.motor_callback(4), 10),
            5: self.create_subscription(Int32MultiArray, 'motor_5_mm', self.motor_callback(5), 10),
        }

        # タイマーの実行速度を高速化
        self.timer = self.create_timer(0.01, self.timer_callback)  # 10ms間隔

        # PID現在値の初期化
        self.current_values = [0] * len(self.controllers)  # モーター数に応じて初期化

    def target_callback(self, msg):
        """
        目標値を受け取るコールバック関数。
        """
        if not isinstance(msg.data, list) and not isinstance(msg.data, array.array):
            self.get_logger().error(f"目標値データ形式が不正です: {type(msg.data)}")
            return

        if len(msg.data) != len(self.controllers):
            self.get_logger().error(f"目標値のデータ数が不正です: {len(msg.data)}")
            return

        for i, target in enumerate(msg.data, start=1):
            if i in self.controllers:
                self.controllers[i]['target'] = target

    def motor_callback(self, motor_id):
        """
        各モーターの現在値を受信するコールバック関数。
        """
        def callback(msg):
            if not isinstance(msg.data, list) and not isinstance(msg.data, array.array):
                self.get_logger().error(f"モーター{motor_id}の現在値データ形式が不正です: {type(msg.data)}")
                return

            if len(msg.data) != 1:
                self.get_logger().error(f"モーター{motor_id}の現在値データ数が不正です: {len(msg.data)}")
                return

            try:
                self.current_values[motor_id - 1] = int(msg.data[0])
            except ValueError as e:
                self.get_logger().error(f"モーター{motor_id}の現在値変換エラー: {e}")

        return callback

    def rpm_callback(self, msg):
        """
        RPM値を受信するコールバック関数。
        """
        self.get_logger().info(f"受信したRPM値: {msg.data}")

    def timer_callback(self):
        """
        PID制御を実行し、制御量をトピックで送信します。
        """
        outputs = []
        debug_info = []  # デバッグ情報を格納するリスト

        for motor_id, params in self.controllers.items():
            current_value = self.current_values[motor_id - 1]  # 現在値を取得
            target_value = params['target']  # 目標値を取得

            # 誤差計算
            error = target_value - current_value

            # ロボマス4の特別処理: 目標値が0の場合は確実に停止
            if motor_id == 4 and target_value == 0:
                output = 0
            elif abs(error) < params['dead_zone']:
                # 不感帯処理
                output = 0
            else:
                # P制御
                output = int(params['kp'] * error)
                output = max(min(output, 3000), -3000)  # 飽和制限を+-3000に変更

            outputs.append(output)
            debug_info.append(f"モーター{motor_id}: 現在値={current_value}, 目標値={target_value}, 制御量={output}")

        # 制御量をトピックで送信
        control_msg = Int32MultiArray()
        control_msg.data = outputs
        self.control_publisher.publish(control_msg)

        # デバッグ情報をログに記録
        self.get_logger().info(f"PID制御結果: {debug_info}")


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
