# -*- coding: utf-8 -*-
"""CAN通信を行うノード。
ロボマスモーターやマイコンと通信し、データの送受信を管理します。
"""

from .angle_tracker import RobomasterAngleTracker


class CANNode:
    def __init__(self):
        # 初期化処理
        self.data = {}

    def send_data(self, id, data):
        """
        CAN通信でデータを送信します。

        :param id: CAN ID
        :param data: 送信するデータ
        """
        # 送信処理（仮実装）
        print(f"Sending data to ID {id}: {data}")

    def receive_data(self, id):
        """
        CAN通信でデータを受信します。

        :param id: CAN ID
        :return: 受信したデータ
        """
        # 受信処理（仮実装）
        return self.data.get(id, None)

    def process_debug_info(self, id, raw_data):
        """
        デバッグ情報を処理します。

        :param id: CAN ID
        :param raw_data: 受信した生データ
        """
        # デバッグ情報の処理（仮実装）
        print(f"Processing debug info for ID {id}: {raw_data}")


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
