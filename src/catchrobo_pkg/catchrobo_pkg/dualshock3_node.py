"""DualShock3コントローラ用ノード."""

import os

import rclpy
from rclpy.node import Node
import pygame


class DualShock3Node(Node):
    """DualShock3から入力を取得するノード."""

    def __init__(self) -> None:
        super().__init__('dualshock3_node')
        os.environ['SDL_JOYSTICK_DEVICE'] = '/dev/input/js0'
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        else:
            self.joystick = None
            self.get_logger().error('ジョイスティックが見つかりません')
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self) -> None:
        """定期処理で軸の値を表示する."""
        if self.joystick:
            pygame.event.pump()
            axis0 = self.joystick.get_axis(0)
            self.get_logger().info(f'axis0: {axis0:.2f}')


def main() -> None:
    """ノードエントリポイント."""
    rclpy.init()
    node = DualShock3Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
