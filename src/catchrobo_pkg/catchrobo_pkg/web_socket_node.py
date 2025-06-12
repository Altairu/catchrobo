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
from std_msgs.msg import String, Float32MultiArray
from fastapi import FastAPI, WebSocket as FastAPIWebSocket
from fastapi.responses import HTMLResponse
import uvicorn
import os


# IPアドレスとポート設定
IP_ADDRESS = '192.168.98.216'
PORT = 8010

# UIファイル（`UI.txt`）のパス
UI_PATH = '/home/altair/catchrobo/src/catchrobo_pkg/catchrobo_pkg/UI.txt'

# FastAPIのインスタンスを作成
app = FastAPI()

# UIの読み込み
if not os.path.exists(UI_PATH):
    raise FileNotFoundError(f'File not found: {UI_PATH}')
with open(UI_PATH, 'r') as f:
    html = f.read()


class WebSocketNode(Node):
    def __init__(self):
        super().__init__('web_socket_node')
        self.send_data = ''
        self.pub = self.create_publisher(String, 'web_socket_pub', 10)
        self.sub = self.create_subscription(Float32MultiArray, 'estimated_position', self.callback, 10)

        @app.get("/")
        async def get():
            return HTMLResponse(html)

        @app.websocket('/ws')
        async def websocket_endpoint(websocket: FastAPIWebSocket):
            await websocket.accept()
            try:
                while True:
                    receive_data = await websocket.receive_text()
                    msg = String()
                    msg.data = receive_data
                    self.pub.publish(msg)

                    string_send_data = ",".join(map(str, self.send_data))
                    await websocket.send_text(string_send_data)
            except Exception as e:
                print(f'WebSocket error: {str(e)}')

    def callback(self, sub_msg):
        self.send_data = sub_msg.data


def run_ros2():
    rclpy.init()
    node = WebSocketNode()
    rclpy.spin(node)
    rclpy.shutdown()


def run_fastapi():
    uvicorn.run(app, host=IP_ADDRESS, port=PORT)


def main():
    ros2_thread = threading.Thread(target=run_ros2)
    ros2_thread.start()

    fastapi_thread = threading.Thread(target=run_fastapi)
    fastapi_thread.start()

    ros2_thread.join()
    fastapi_thread.join()


if __name__ == '__main__':
    main()
