#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial, threading
from rclpy.logging import LoggingSeverity

class CustomControllerNode(Node):
    def __init__(self):
        super().__init__('custom_controller_node')
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        self.pub_cmdspec2 = self.create_publisher(Int32MultiArray, 'cmd_spec2', 10)

        self.cmd2_val = 0
        self.spec2_val = 0
        self.prev_b_vals = [0] * 8

        port = "/dev/ttyUSB0"
        baud = 115200
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Serial open: {port}")
        except Exception as e:
            self.get_logger().error(f"Serial open failed: {e}")
            self.ser = None
            return

        self.stop = threading.Event()
        threading.Thread(target=self.read_loop, daemon=True).start()

        self.create_timer(0.02, self.publish_cmdspec2)

    def read_loop(self):
        while not self.stop.is_set() and self.ser:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                self.get_logger().debug(f"Raw serial line: {line}")
                if not line.startswith("B:"): continue

                parts = line.split("T:")
                self.get_logger().debug(f"Split parts: {parts}")
                b_vals = [int(x) for x in parts[0].replace("B:", "").strip(",").split(",") if x]
                self.get_logger().debug(f"Parsed B values: {b_vals}")
                t_vals = [int(x) for x in parts[1].strip(",").split(",") if x]
                self.get_logger().debug(f"Parsed T values: {t_vals}")

                if len(b_vals) >= 4:
                    if b_vals[0] == 1 and self.prev_b_vals[0] == 0:
                        self.cmd2_val = max(0, self.cmd2_val - 1)
                    if b_vals[2] == 1 and self.prev_b_vals[2] == 0:
                        self.cmd2_val = min(100, self.cmd2_val + 1)
                    if b_vals[3] == 1 and self.prev_b_vals[3] == 0:
                        self.spec2_val = max(0, self.spec2_val - 1)
                    if b_vals[1] == 1 and self.prev_b_vals[1] == 0:
                        self.spec2_val = min(100, self.spec2_val + 1)

                if any(t_vals):
                    self.spec2_val = 0

                self.prev_b_vals = b_vals

            except Exception as e:
                self.get_logger().warn(f"Read error: {e}")

    def publish_cmdspec2(self):
        msg = Int32MultiArray()
        msg.data = [self.cmd2_val, self.spec2_val]
        self.pub_cmdspec2.publish(msg)
        self.get_logger().debug(f"Published cmd_spec2: {msg.data}")

    def destroy_node(self):
        self.stop.set()
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CustomControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
