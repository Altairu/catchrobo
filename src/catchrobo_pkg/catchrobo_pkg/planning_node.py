# -*- coding: utf-8 -*-
"""ロボットの自動制御を行うノード。
動作番号と特別動作番号に基づきロボットを制御します。
"""

import rclpy
from rclpy.node import Node


class PlanningNode(Node):
    """ロボットの動作計画を実行するノード."""

    def __init__(self) -> None:
        super().__init__('planning_node')
        # 動作番号と特別動作番号の初期化
        self.action_number = 0
        self.special_action_number = 0

    def execute_action(self):
        """
        動作番号に基づきロボットを制御します。
        """
        if self.action_number == 0:
            print("停止")
        elif self.action_number == 1:
            print("初期状態: CAN通信開始")
        elif self.action_number == 2:
            print("ベルト動作開始: ロボマス4 目標[100RPM]")
        elif self.action_number == 3:
            print("ベルト停止: ロボマス4 目標値0")
        else:
            print("未定義の動作番号")

    def execute_special_action(self):
        """
        特別動作番号に基づきロボットを制御します。
        """
        if self.special_action_number == 0:
            print("通常状態")
        elif self.special_action_number == 1:
            print("妨害機構展開")
        elif self.special_action_number == 2:
            print("妨害機構収納")
        elif self.special_action_number == 3:
            print("緊急非常停止（初期化）")
        elif self.special_action_number == 4:
            print("緊急非常停止（CAN通信の停止）")
        else:
            print("未定義の特別動作番号")


def main() -> None:
    """ノードエントリポイント."""
    rclpy.init()
    node = PlanningNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# 使用例
if __name__ == "__main__":
    planning_node = PlanningNode()
    planning_node.action_number = 2
    planning_node.execute_action()
    planning_node.special_action_number = 1
    planning_node.execute_special_action()
