# -*- coding: utf-8 -*-
"""
WebSocket通信を行うノード。
スマートフォンとPC間で通信を行い、特別動作番号を操作します。
"""

import asyncio
import websockets
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from fastapi import FastAPI, WebSocket as FastAPIWebSocket, Request
from fastapi.responses import HTMLResponse
import uvicorn
import os


# IPアドレスとポート設定
IP_ADDRESS = '192.168.85.216'
PORT = 8080

# UIファイル（`UI.txt`）のパス
UI_PATH = '/home/altair/catchrobo/src/catchrobo_pkg/catchrobo_pkg/UI.txt'


# FastAPIのインスタンスを作成
app = FastAPI()

@app.middleware("http")
async def handle_invalid_requests(request: Request, call_next):
    try:
        response = await call_next(request)
        return response
    except Exception as e:
        print(f"WARNING: Invalid HTTP request received. Error: {e}")
        return Response(content="Invalid request", status_code=400)

# UIの読み込み
if not os.path.exists(UI_PATH):
    raise FileNotFoundError(f'File not found: {UI_PATH}')
with open(UI_PATH, 'r') as f:
    html = f.read()


class WebSocketNode(Node):
    def __init__(self):
        super().__init__('web_socket_node')
        self.special_action_number = 0  # 特別動作番号
        self.pub = self.create_publisher(Int32, 'special_action_number', 10)

        @app.get("/")
        async def get():
            return HTMLResponse(html)

        @app.websocket('/ws')
        async def websocket_endpoint(websocket: FastAPIWebSocket):
            await websocket.accept()
            try:
                while True:
                    receive_data = await websocket.receive_text()
                    print(f"Received data: {receive_data}")  # デバッグ用ログ

                    # 特別動作番号を更新
                    if receive_data.isdigit():
                        self.special_action_number = int(receive_data)
                        print(f"Special Action Number: {self.special_action_number}")  # デバッグ用ログ

                        # 特別動作番号をパブリッシュ
                        msg = Int32()
                        msg.data = self.special_action_number
                        self.pub.publish(msg)

                    await asyncio.sleep(0.01)  # 10msの間隔を追加
            except Exception as e:
                print(f'WebSocket error: {str(e)}')


def run_ros2():
    rclpy.init()
    node = WebSocketNode()
    rclpy.spin(node)
    rclpy.shutdown()

def run_fastapi():
    uvicorn.run(
        app, 
        host=IP_ADDRESS, 
        port=PORT
    )

def main():
    ros2_thread = threading.Thread(target=run_ros2)
    ros2_thread.start()

    fastapi_thread = threading.Thread(target=run_fastapi)
    fastapi_thread.start()

    ros2_thread.join()
    fastapi_thread.join()

if __name__ == '__main__':
    main()
