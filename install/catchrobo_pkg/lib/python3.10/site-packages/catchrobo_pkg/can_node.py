# -*- coding: utf-8 -*-
"""CAN通信を行うノード。
ロボマスモーターやマイコンと通信し、データの送受信を管理します。
"""

import can
import struct
from std_msgs.msg import Int32MultiArray
import rclpy
from rclpy.node import Node


class CANNode(Node):
    def __init__(self):
        super().__init__('can_node')
        # 初期化処理
        self.data = {}
        self.trackers = {
            1: self.create_angle_tracker(2.617993),
            2: self.create_angle_tracker(1.240102),
            3: self.create_angle_tracker(3.141592),
            4: None,  # RPMのためトラッカー不要
            5: self.create_angle_tracker(2.530727),
        }
        self.rpm_values = {4: 0}  # ロボマス4のRPM値を初期化

        # CANインターフェースの初期化
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')

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

        # RPM値を送信するためのパブリッシャ
        self.rpm_publisher = self.create_publisher(
            Int32MultiArray,
            'can_rpm_values',  # トピック名を明確化
            10
        )

        # 各モーターの現在値を送信するためのパブリッシャ
        self.motor_publishers = {
            1: self.create_publisher(Int32MultiArray, 'motor_1_mm', 10),
            2: self.create_publisher(Int32MultiArray, 'motor_2_mm', 10),
            3: self.create_publisher(Int32MultiArray, 'motor_3_mm', 10),
            4: self.create_publisher(Int32MultiArray, 'motor_4_rpm', 10),
            5: self.create_publisher(Int32MultiArray, 'motor_5_mm', 10),
        }

        # → PIDノード廃止につきサブスクライバ削除

    def create_angle_tracker(self, mm_per_rotation):
        """
        角度トラッカーを作成します。

        :param mm_per_rotation: 1回転あたりの移動距離（ミリメートル）
        :return: 角度トラッカー
        """
        return {
            "mm_per_rotation": mm_per_rotation,
            "last_raw": None,
            "total_angle": 0
        }

    def update_angle_tracker(self, tracker, raw_angle):
        """
        角度トラッカーを更新します。

        :param tracker: 角度トラッカー
        :param raw_angle: モーターから受信した生角度
        :return: 累積距離（ミリメートル）
        """
        if tracker["last_raw"] is None:
            tracker["last_raw"] = raw_angle
            return 0

        delta = (raw_angle - tracker["last_raw"] + 8192) % 8192
        if delta > 4096:
            delta -= 8192

        tracker["total_angle"] += delta
        tracker["last_raw"] = raw_angle
        return tracker["total_angle"] * tracker["mm_per_rotation"] / 8192

    def receive_data(self):
        """
        CAN通信でデータを受信します。

        :return: 受信したデータ
        """
        try:
            while True:  # 休み時間なしで受信を試みる
                msg = self.bus.recv(timeout=0)  # 即時受信
                if msg:
                    self.get_logger().info(f"[CAN受信] ID: {msg.arbitration_id}, データ: {list(msg.data)}")
                    return msg.arbitration_id, list(msg.data)
                else:
                    self.get_logger().warning("[CAN受信] データなし")
                    return None, None
        except can.CanError as e:
            self.get_logger().error(f"CAN受信エラー: {e}")
            return None, None

    def timer_callback(self):
        """
        送信・受信処理を定期的に実行します。
        """
        # PIDノード廃止：ここでは主に受信データ（リミットスイッチなど）の処理のみを行う
        id, data = self.receive_data()
        if id == 0x305:
            self.process_limit_switch_data(id, data)

    def send_data(self, id, data):
        """
        CAN通信でデータを送信します。

        :param id: CAN ID
        :param data: 送信するデータ（リスト形式）
        """
        try:
            # データを検証して範囲外の値を防ぐ
            validated_data = [min(max(0, byte), 255) for byte in data]
            if len(validated_data) != 8:
                validated_data = validated_data[:8] + [0] * (8 - len(validated_data))  # データ長を8に統一
            msg = can.Message(arbitration_id=id, data=validated_data, is_extended_id=False)
            self.bus.send(msg)
            self.get_logger().info(f"[CAN送信] ID: {id}, データ: {validated_data}")
        except can.CanError as e:
            self.get_logger().error(f"CAN送信エラー: {e}")
        except ValueError as e:
            self.get_logger().error(f"データ検証エラー: {e}, データ: {data}")

    def process_debug_info(self, id, raw_data):
        """
        デバッグ情報を処理し、各モーターの現在値をトピックで送信します。
        """
        motor_id = id - 0x200  # モーターIDは0x200からのオフセットで計算
        if motor_id < 1 or motor_id > 5:
            self.get_logger().warning(f"[デバッグ情報] 無効なID: {id}")
            return

        angle_raw, rpm, current, temperature = struct.unpack('>HHHH', bytes(raw_data[:8]))
        if motor_id in self.trackers and self.trackers[motor_id] is not None:
            distance_mm = self.update_angle_tracker(self.trackers[motor_id], angle_raw)
            self.get_logger().info(f"[デバッグ情報] モーター{motor_id}, 距離: {distance_mm:.2f} mm, RPM: {rpm}, 電流: {current}, 温度: {temperature}")

            # モーターの現在値をトピックで送信
            motor_msg = Int32MultiArray()
            motor_msg.data = [int(distance_mm) if motor_id != 4 else int(rpm)]  # モーター4はRPM、それ以外は距離
            self.motor_publishers[motor_id].publish(motor_msg)
            self.get_logger().info(f"[モーター{motor_id}現在値送信] {motor_msg.data}")

        # PID現在値をトピックで送信
        pid_current_values = Int32MultiArray()
        pid_current_values.data = [
            int(self.trackers[1]["total_angle"] * self.trackers[1]["mm_per_rotation"] / 8192) if self.trackers[1] else 0,
            int(self.trackers[2]["total_angle"] * self.trackers[2]["mm_per_rotation"] / 8192) if self.trackers[2] else 0,
            int(self.trackers[3]["total_angle"] * self.trackers[3]["mm_per_rotation"] / 8192) if self.trackers[3] else 0,
            int(rpm),  # ロボマス4のRPM値を直接使用
            int(self.trackers[5]["total_angle"] * self.trackers[5]["mm_per_rotation"] / 8192) if self.trackers[5] else 0
        ]
        self.pid_current_publisher.publish(pid_current_values)
        self.get_logger().info(f"[PID現在値送信] {pid_current_values.data}")

        # RPM値をトピックで送信
        rpm_values_msg = Int32MultiArray()
        rpm_values_msg.data = [rpm]  # 現在のRPM値を送信
        self.rpm_publisher.publish(rpm_values_msg)
        self.get_logger().info(f"[RPM値送信] {rpm_values_msg.data}")

    def process_limit_switch_data(self, id, raw_data):
        """
        リミットスイッチのデータを処理し、トピックで送信します。
        """
        limit_switch_states = [int(state) for state in raw_data[:4]]  # リミットスイッチの状態を整数に変換
        self.get_logger().info(f"[リミットスイッチ] ID: {id}, 状態: {limit_switch_states}")

        # トピックで送信
        limit_switch_msg = Int32MultiArray()
        limit_switch_msg.data = limit_switch_states
        self.limit_switch_publisher.publish(limit_switch_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CANNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("CANNodeを終了します。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
