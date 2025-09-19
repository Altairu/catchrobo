#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int16MultiArray
import serial
import can
import time

HEADER = bytes([0xA5, 0xA5])

class SerialCanNode(Node):
    def __init__(self):
        super().__init__("serial_can_node")

        # --- Serial設定 ---
        try:
            self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=0.2)
            self.get_logger().info("Serial open OK")
        except Exception as e:
            self.get_logger().error(f"Serial open failed: {e}")
            self.ser = None

        # --- CAN設定 ---
        try:
            self.bus = can.interface.Bus(channel="can0", bustype="socketcan")
            self.get_logger().info("CAN open OK")
        except Exception as e:
            self.get_logger().error(f"CAN open failed: {e}")
            self.bus = None

        # --- Subscriber登録 ---
        self.create_subscription(Int32MultiArray, "cmd_spec", self.on_cmd_spec, 10)
        self.create_subscription(Int16MultiArray, "motor_currents", self.on_motor_currents, 10)

    def on_cmd_spec(self, msg):
        if not self.ser: return
        try:
            cmd, spec = msg.data
            pkt = HEADER + bytes([cmd & 0xFF, spec & 0xFF])
            self.ser.write(pkt)
            self.ser.flush()
        except Exception as e:
            self.get_logger().error(f"Serial send error: {e}")

    def on_motor_currents(self, msg):
        if not self.bus: return
        try:
            currents = msg.data
            data200 = []
            for i in range(4):
                v = int(currents[i]) if i < len(currents) else 0
                data200 += [(v>>8)&0xFF, v&0xFF]
            msg200 = can.Message(arbitration_id=0x200, data=data200, is_extended_id=False)
            self.bus.send(msg200)

            v = int(currents[4]) if len(currents) >= 5 else 0
            data1ff = [(v>>8)&0xFF, v&0xFF] + [0]*6
            msg1ff = can.Message(arbitration_id=0x1FF, data=data1ff, is_extended_id=False)
            self.bus.send(msg1ff)
        except Exception as e:
            self.get_logger().error(f"CAN send error: {e}")

def main():
    rclpy.init()
    node = SerialCanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
