"""PID制御ノード."""

import rclpy
from rclpy.node import Node


class PIDNode(Node):
    """ロボマスのPID制御を行うノード."""

    def __init__(self) -> None:
        super().__init__('pid_node')
        # ここでPID制御用のパラメータを初期化する


def main() -> None:
    """ノードエントリポイント."""
    rclpy.init()
    node = PIDNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
