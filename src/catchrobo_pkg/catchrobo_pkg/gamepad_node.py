#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int32MultiArray
import evdev   # ゲームパッド入力用


class GamepadNode(Node):
    def __init__(self):
        super().__init__('gamepad_node')

        # Publisher
        self.pub_motor = self.create_publisher(Int16MultiArray, 'motor_cmd', 10)
        self.pub_cmdspec = self.create_publisher(Int32MultiArray, 'cmd_spec', 10)

        # ゲームパッドデバイス（必要に応じて変更）
        device_path = "/dev/input/event16"
        try:
            self.dev = evdev.InputDevice(device_path)
            self.dev.grab()  # 他のアプリに入力を奪われないようにする
            self.get_logger().info(f"Gamepad open: {self.dev.name}")
        except Exception as e:
            self.get_logger().error(f"Gamepad open failed: {e}")
            self.dev = None
            return

        # 値保持
        self.axes = {0: 0, 1: 0, 2: 0, 5: 0}
        self.buttons = {304: 0, 305: 0, 307: 0, 308: 0, 310: 0, 311: 0, 312: 0, 313: 0}
        self.last_motor4 = 0

        # イベントチェック用タイマー
        self.create_timer(0.02, self.read_gamepad)  # 50Hz

        # 指示番号保持
        self.cmd_val = 0
        self.spec_val = 0

        # デバッグ出力をONにするフラグ
        self.debug_mode = True

    def _normalize_axis(self, val):
        norm = (val - 128) / 127.0
        if abs(norm) < 0.05:
            norm = 0
        return norm

    def read_gamepad(self):
        if self.dev is None:
            return

        # イベント読み取り
        try:
            for event in self.dev.read():
                if event.type == evdev.ecodes.EV_ABS and event.code in self.axes:
                    self.axes[event.code] = event.value
                elif event.type == evdev.ecodes.EV_KEY and event.code in self.buttons:
                    self.buttons[event.code] = event.value
                    if event.value == 0:  # ボタン離した瞬間
                        self._on_release(event.code)
        except BlockingIOError:
            pass  # イベントがない場合

        # 軸からモータ値に変換
        m1 = int(-self._normalize_axis(self.axes[1]) * 1000)  # 左Y
        m2 = int(self._normalize_axis(self.axes[0]) * 1000)   # 左X
        m3 = int(-self._normalize_axis(self.axes[5]) * 1000)  # 右Y
        m5 = int(self._normalize_axis(self.axes[2]) * 1000)   # 右X

        if self.buttons[305]:       # B
            self.last_motor4 = 1000
        elif self.buttons[304]:     # A
            self.last_motor4 = 0
        m4 = self.last_motor4

        # motor_cmd publish
        msg_motor = Int16MultiArray()
        msg_motor.data = [m1, m2, m3, m4, m5]
        self.pub_motor.publish(msg_motor)

        # cmd_spec publish
        msg_cmdspec = Int32MultiArray()
        msg_cmdspec.data = [self.cmd_val, self.spec_val]
        self.pub_cmdspec.publish(msg_cmdspec)

        # ===== デバッグ出力 =====
        if self.debug_mode:
            self.get_logger().info(
                f"motor_cmd={msg_motor.data} | cmd={self.cmd_val}, spec={self.spec_val}"
            )

    def _on_release(self, code):
        if code == 313:  # RT
            self.cmd_val = min(100, self.cmd_val + 1)
        elif code == 312:  # LT
            self.cmd_val = max(0, self.cmd_val - 1)
        elif code == 311:  # R
            self.spec_val = min(3, self.spec_val + 1)
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
