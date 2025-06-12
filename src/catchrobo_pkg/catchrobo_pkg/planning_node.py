"""自動制御のためのプランニングノード."""

import rclpy
from rclpy.node import Node


class PlanningNode(Node):
    """ロボットの動作計画を実行するノード."""

    def __init__(self) -> None:
        super().__init__('planning_node')
        # ここにプランニングの初期設定を書く


def main() -> None:
    """ノードエントリポイント."""
    rclpy.init()
    node = PlanningNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
