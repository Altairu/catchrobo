# -*- coding: utf-8 -*-
"""
WebSocket通信を行うノード。
スマートフォンとPC間で通信を行い、特別動作番号を操作します。
"""

import asyncio
import websockets


class WebSocketNode:
    def __init__(self, host="localhost", port=8765):
        """
        初期化処理。

        :param host: ホスト名
        :param port: ポート番号
        """
        self.host = host
        self.port = port
        self.special_action_number = 0

    async def handler(self, websocket, path):
        """
        WebSocketのハンドラ。

        :param websocket: WebSocket接続
        :param path: 接続パス
        """
        async for message in websocket:
            print(f"受信: {message}")
            self.special_action_number = int(message)
            await websocket.send(f"特別動作番号を {self.special_action_number} に設定しました。")

    def start_server(self):
        """
        WebSocketサーバを開始します。
        """
        print(f"WebSocketサーバを {self.host}:{self.port} で開始します")
        start_server = websockets.serve(self.handler, self.host, self.port)
        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()


# 使用例
if __name__ == "__main__":
    node = WebSocketNode(host="0.0.0.0", port=8080)
    node.start_server()
