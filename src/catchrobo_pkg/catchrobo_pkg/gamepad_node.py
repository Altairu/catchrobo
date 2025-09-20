#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int32MultiArray
import evdev


class GamepadNode(Node):
    def __init__(self):
        super().__init__('gamepad_node')

        self.pub_motor = self.create_publisher(Int16MultiArray, 'motor_cmd', 10)
        self.pub_cmdspec1 = self.create_publisher(Int32MultiArray, 'cmd_spec1', 10)

        device_path = "/dev/input/event16"
        try:
            self.dev = evdev.InputDevice(device_path)
            self.dev.grab()
            self.get_logger().info(f"Gamepad open: {self.dev.name}")
        except Exception as e:
            self.get_logger().error(f"Gamepad open failed: {e}")
            self.dev = None
            return

        self.axes = {0: 0, 1: 0, 2: 0, 5: 0}
        self.buttons = {304: 0, 305: 0, 310: 0, 311: 0, 312: 0, 313: 0}
        self.last_motor4 = 0

        self.cmd_val = 0
        self.spec_val = 0

        self.create_timer(0.02, self.read_gamepad)

    def _normalize_axis(self, val):
        norm = (val - 128) / 127.0
        return 0 if abs(norm) < 0.05 else norm

    def read_gamepad(self):
        if self.dev is None:
            return
        try:
            for event in self.dev.read():
                if event.type == evdev.ecodes.EV_ABS and event.code in self.axes:
                    self.axes[event.code] = event.value
                elif event.type == evdev.ecodes.EV_KEY and event.code in self.buttons:
                    self.buttons[event.code] = event.value
                    if event.value == 0:
                        self._on_release(event.code)
        except BlockingIOError:
            pass

        m1 = int(-self._normalize_axis(self.axes[1]) * 1000)
        m2 = int(self._normalize_axis(self.axes[0]) * 1000)
        m3 = int(-self._normalize_axis(self.axes[5]) * 1000)
        m5 = int(self._normalize_axis(self.axes[2]) * 1000)

        if self.buttons[305]:
            self.last_motor4 = 5000
        elif self.buttons[304]:
            self.last_motor4 = 0
        m4 = self.last_motor4

        msg_motor = Int16MultiArray()
        msg_motor.data = [m1, m2, m3, m4, m5]
        self.pub_motor.publish(msg_motor)
        self.get_logger().debug(f"Published motor_cmd: {msg_motor.data}")

        msg_cmdspec1 = Int32MultiArray()
        msg_cmdspec1.data = [self.cmd_val, self.spec_val]
        self.pub_cmdspec1.publish(msg_cmdspec1)
        self.get_logger().debug(f"Published cmd_spec1: {msg_cmdspec1.data}")

    def _on_release(self, code):
        if code == 313:  # RT
            self.cmd_val = min(100, self.cmd_val + 1)
        elif code == 312:  # LT
            self.cmd_val = max(0, self.cmd_val - 1)
        elif code == 311:  # R
            self.spec_val = min(100, self.spec_val + 1)
        elif code == 310:  # L
            self.spec_val = max(0, self.spec_val - 1)


def main(args=None):
    rclpy.init(args=args)
    node = GamepadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
