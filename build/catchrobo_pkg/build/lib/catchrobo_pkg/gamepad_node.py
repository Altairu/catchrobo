#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import evdev
import fcntl, os

class GamepadNode(Node):
    def __init__(self):
        super().__init__("gamepad_node")
        self.pub = self.create_publisher(Joy, "joy", 10)
        self.device_path = "/dev/input/event16"  # 環境に合わせる
        try:
            self.dev = evdev.InputDevice(self.device_path)
            self.get_logger().info(f"Gamepad open: {self.dev}")

            # ノンブロッキングに設定
            fd = self.dev.fd
            fl = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

        except Exception as e:
            self.get_logger().error(f"Gamepad open error: {e}")
            self.dev = None
            return

        self.create_timer(0.01, self.publish_joy)

        # 確認済みマッピング
        self.axes_map = {0: 0, 1: 0, 2: 0, 5: 0}
        self.buttons_map = {
            304: 0,  # B
            305: 0,  # A
            307: 0,  # Y
            308: 0,  # X
            310: 0,  # LB
            311: 0,  # RB
            312: 0,  # LT
            313: 0   # RT
        }

        # デバッグフラグ
        self.debug = True

    def publish_joy(self):
        if not self.dev:
            return

        # イベントを全部読む
        while True:
            try:
                event = self.dev.read_one()
                if event is None:
                    break
                if event.type == evdev.ecodes.EV_ABS and event.code in self.axes_map:
                    self.axes_map[event.code] = event.value
                elif event.type == evdev.ecodes.EV_KEY and event.code in self.buttons_map:
                    self.buttons_map[event.code] = event.value
            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().warn(f"evdev read error: {e}")
                break

        # Joy メッセージ作成
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [float(v) for v in self.axes_map.values()]
        msg.buttons = [int(v) for v in self.buttons_map.values()]
        self.pub.publish(msg)

        # デバッグ出力
        if self.debug:
            self.get_logger().info(
                f"axes={msg.axes}, buttons={msg.buttons}"
            )

def main():
    rclpy.init()
    node = GamepadNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
