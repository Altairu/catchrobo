# -*- coding: utf-8 -*-
"""CAN通信を行うノード。
ロボマスモーターやマイコンと通信し、データの送受信を管理します。
"""

import struct
from std_msgs.msg import Int32MultiArray
import rclpy
from rclpy.node import Node


class RobomasterAngleTracker:
    def __init__(self, mm_per_rotation):
        self.last_raw = None
        self.total_angle = 0  # 累積角度（0基準）
        self.mm_per_rotation = mm_per_rotation

    def update(self, raw_angle):
        """
        モーターから受信した生角度（0〜8191）を更新。
        :param raw_angle: モーターから受信した生角度
        :return: 累積角度（mm単位）
        """
        if self.last_raw is None:
            self.last_raw = raw_angle
            return self.total_angle * self.mm_per_rotation / 8192

        delta = (raw_angle - self.last_raw + 8192) % 8192
        if delta > 4096:
            delta -= 8192  # 巻き戻し対応

        self.total_angle += delta
        self.last_raw = raw_angle
        return self.total_angle * self.mm_per_rotation / 8192

    def get_mm(self):
        """
        累積距離を取得（mm単位）。
        :return: 累積距離
        """
        return self.total_angle * self.mm_per_rotation / 8192


class CANNode(Node):
    def __init__(self):
        super().__init__('can_node')
        # 初期化処理
        self.data = {}
        self.trackers = {
            1: RobomasterAngleTracker(2.617993),
            2: RobomasterAngleTracker(1.240102),
            3: RobomasterAngleTracker(3.141592),
            4: None,  # RPMのためトラッカー不要
            5: RobomasterAngleTracker(2.530727),
        }

        # PID現在値を送信するためのパブリッシャ
        self.pid_current_publisher = self.create_publisher(
            Int32MultiArray,
            'pid_current_values',
            10
        )

        # リミットスイッチの状態を送信するためのパブリッシャ
        self.limit_switch_publisher = self.create_publisher(
            Int32MultiArray,
            'limit_switch_states',
            10
        )

    def send_data(self, id, data):
        """
        CAN通信でデータを送信します。

        :param id: CAN ID
        :param data: 送信するデータ（リスト形式）
        """
        # 実際のCAN通信送信処理をここに実装
        print(f"[CAN送信] ID: {id}, データ: {data}")

    def receive_data(self, id):
        """
        CAN通信でデータを受信します。

        :param id: CAN ID
        :return: 受信したデータ
        """
        # 実際のCAN通信受信処理をここに実装
        return self.data.get(id, None)

    def process_debug_info(self, id, raw_data):
        """
        デバッグ情報を処理し、PID現在値をトピックで送信します。
        """
        angle_raw, rpm, current, temperature = struct.unpack('>HHHH', raw_data[:8])
        if id in self.trackers and self.trackers[id] is not None:
            distance_mm = self.trackers[id].update(angle_raw)
            self.get_logger().info(f"[デバッグ情報] ID: {id}, 距離: {distance_mm:.2f} mm, RPM: {rpm}, 電流: {current}, 温度: {temperature}")
        else:
            self.get_logger().info(f"[デバッグ情報] ID: {id}, 角度: {angle_raw}, RPM: {rpm}, 電流: {current}, 温度: {temperature}")

        # PID現在値をトピックで送信
        pid_current_values = Int32MultiArray()
        pid_current_values.data = [
            self.trackers[1].get_mm() if self.trackers[1] else 0,
            self.trackers[2].get_mm() if self.trackers[2] else 0,
            self.trackers[3].get_mm() if self.trackers[3] else 0,
            rpm,  # ロボマス4はRPM
            self.trackers[5].get_mm() if self.trackers[5] else 0
        ]
        self.pid_current_publisher.publish(pid_current_values)

    def process_limit_switch_data(self, id, raw_data):
        """
        リミットスイッチのデータを処理し、トピックで送信します。
        """
        limit_switch_states = list(raw_data[:4])  # リミットスイッチの状態を抽出
        self.get_logger().info(f"[リミットスイッチ] ID: {id}, 状態: {limit_switch_states}")

        # トピックで送信
        limit_switch_msg = Int32MultiArray()
        limit_switch_msg.data = limit_switch_states
        self.limit_switch_publisher.publish(limit_switch_msg)

    def send_servo_motor_data(self, servo_data, motor_data):
        """
        サーボとモーターのデータを送信します。
        """
        self.send_data(0x300, servo_data[:6])  # サーボ1～6
        self.send_data(0x301, servo_data[6:12])  # サーボ7～12
        self.send_data(0x100, motor_data)  # モーター


def main():
    """エントリポイント."""
    tracker = RobomasterAngleTracker()
    print(tracker.get_angle())


# 使用例
if __name__ == "__main__":
    # 実際の仕様に基づきCANNodeを初期化し、データ送信と受信を行う
    can_node = CANNode()

    # 例: ロボマスモーターの制御データを送信
    can_node.send_data(0x200, [0, 0, 0, 0, 0, 0])  # 初期化時のデータ

    # 例: デバッグ情報を受信して処理
    raw_data = can_node.receive_data(0x201)  # ロボマス1のデバッグ情報
    if raw_data:
        can_node.process_debug_info(0x201, raw_data)

    # 他のIDに対する送受信も同様に実装可能
