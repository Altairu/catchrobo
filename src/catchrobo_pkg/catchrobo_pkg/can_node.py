# -*- coding: utf-8 -*-
"""CAN通信を行うノード。
ロボマスモーターやマイコンと通信し、データの送受信を管理します。
"""

import struct

from .angle_tracker import RobomasterAngleTracker


class CANNode:
    def __init__(self):
        # 初期化処理
        self.data = {}

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
        デバッグ情報を処理します。

        :param id: CAN ID
        :param raw_data: 受信した生データ
        """
        # デバッグ情報の処理
        angle, rpm, current, temperature = struct.unpack('>HHHH', raw_data[:8])
        print(f"[デバッグ情報] ID: {id}, 角度: {angle}, RPM: {rpm}, 電流: {current}, 温度: {temperature}")


def main():
    """エントリポイント."""
    tracker = RobomasterAngleTracker()
    print(tracker.get_angle())


# 使用例
if __name__ == "__main__":
    can_node = CANNode()
    can_node.send_data(0x200, [0x01, 0x02, 0x03])
    received = can_node.receive_data(0x200)
    print(f"Received: {received}")
    main()
