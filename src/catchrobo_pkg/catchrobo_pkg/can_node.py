"""CANノードの実装."""

import struct

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class CANNode(Node):
    """CAN通信を行うノード."""

    def __init__(self) -> None:
        super().__init__('can_node')
        try:
            # SocketCAN デバイスに接続
            self.bus = can.Bus(interface='socketcan', channel='can0')
        except OSError as exc:
            self.get_logger().error(
                f"SocketCANデバイスcan0にアクセスできません: {exc}")
            rclpy.shutdown()
            return

        # パブリッシャ、サブスクライバを設定
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 'robot_position', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'send_can_message',
            self.send_can_message_callback,
            10,
        )
        self.timer = self.create_timer(0.001, self.timer_callback)
        self.action_number = 0

    def send_can_message_callback(self, msg: Float32MultiArray) -> None:
        """送信メッセージを受け取りCANバスへ送信する."""
        self.get_logger().info(
            f"send_can_message_callback called with data: {msg.data}")
        if len(msg.data) != 4:
            return

        vx = float(msg.data[0])  # Vx
        vy = float(msg.data[1])  # Vy
        omega = float(msg.data[2])  # ω
        action_number = int(msg.data[3])  # 指示番号

        # action_number を 0-10 に制限
        action_number = max(0, min(10, action_number))

        # 0x160 メッセージを送信
        data_160 = struct.pack('>hhh', int(vx), int(vy), int(omega))
        can_msg_160 = can.Message(
            arbitration_id=0x160,
            data=data_160,
            is_extended_id=False,
        )
        try:
            self.bus.send(can_msg_160)
            self.get_logger().info(
                f"送信[0x160: {data_160.hex()}] 送信[{vx}, {vy}, {omega}]")
        except can.CanError:
            self.get_logger().error('Failed to send CAN message 0x160')

        # 0x161 メッセージを送信
        data_161 = struct.pack('>B', action_number)
        can_msg_161 = can.Message(
            arbitration_id=0x161,
            data=data_161,
            is_extended_id=False,
        )
        try:
            self.bus.send(can_msg_161)
            self.get_logger().info(
                "送信[0x161: %s] 送信[Action Number: %d]" % (
                    data_161.hex(),
                    action_number,
                ),
            )
        except can.CanError:
            self.get_logger().error('Failed to send CAN message 0x161')

    def timer_callback(self) -> None:
        """CANバスからの受信処理を行う."""
        msg = self.bus.recv(timeout=0.001)
        if msg is None:
            return

        if msg.arbitration_id in (0x150, 0x360):
            try:
                x = struct.unpack('>h', msg.data[0:2])[0]
                y = struct.unpack('>h', msg.data[2:4])[0]
                theta = struct.unpack('>h', msg.data[4:6])[0]
                command_msg = Float32MultiArray()
                command_msg.data = [float(x), float(y), float(theta),
                                    float(self.action_number)]
                self.publisher_.publish(command_msg)
            except struct.error as exc:
                self.get_logger().error(f"Unpacking error: {exc}")
        elif msg.arbitration_id == 0x151:
            try:
                self.action_number = struct.unpack('>B', msg.data)[0]
            except struct.error as exc:
                self.get_logger().error(f"Unpacking error: {exc}")


def main(args=None) -> None:
    """ノードのエントリポイント."""
    rclpy.init(args=args)
    node = CANNode()
    if node.bus:
        rclpy.spin(node)
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
