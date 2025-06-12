"""WebSocket通信を行うノード."""

import asyncio

import rclpy
from rclpy.node import Node

try:
    import websockets
except ImportError:  # websocketsがない環境向け
    websockets = None


class WebSocketNode(Node):
    """スマートフォンと通信するためのノード."""

    def __init__(self) -> None:
        super().__init__('web_socket_node')
        self.server = None

    async def handler(self, websocket, path) -> None:
        """メッセージ受信時の処理."""
        async for message in websocket:
            self.get_logger().info(f"受信: {message}")
            await websocket.send(message)

    async def start(self) -> None:
        """WebSocketサーバーを起動する."""
        if websockets is None:
            self.get_logger().error('websockets モジュールがありません')
            return
        self.server = await websockets.serve(self.handler, '0.0.0.0', 8080)
        await self.server.wait_closed()


def main() -> None:
    """ノードエントリポイント."""
    rclpy.init()
    node = WebSocketNode()
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(node.start())
    finally:
        loop.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
