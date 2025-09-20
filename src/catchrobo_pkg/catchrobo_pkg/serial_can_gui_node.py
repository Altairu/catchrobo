#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int32MultiArray, String
import serial, threading, time, subprocess, can
from serial.tools import list_ports
import tkinter as tk
from tkinter import ttk, messagebox

HEADER = bytes([0xA5, 0xA5])


# ===================== ROS2 Node =====================
class SerialCanGuiNode(Node):
    def __init__(self, app):
        super().__init__('serial_can_gui_node')
        self.app = app

        # Subscriber
        self.sub_motor = self.create_subscription(Int16MultiArray, 'motor_cmd', self.on_motor_cmd, 10)
        self.sub_cmdspec1 = self.create_subscription(Int32MultiArray, 'cmd_spec1', self.on_cmdspec1, 10)
        self.sub_cmdspec2 = self.create_subscription(Int32MultiArray, 'cmd_spec2', self.on_cmdspec2, 10)

        # Publisher
        self.pub_echo = self.create_publisher(String, 'echo', 10)

        # Serial / CAN
        self.ser = None
        self.bus = None

        # 値保持
        self.cmd1 = self.spec1 = 0
        self.cmd2 = self.spec2 = 0
        self.motor_currents = [0, 0, 0, 0, 0]

    # ===== Serial処理 =====
    def connect_serial(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
            self.get_logger().info(f"Serial connected: {port}")
            threading.Thread(target=self._rx_loop, daemon=True).start()
            return True
        except Exception as e:
            messagebox.showerror("シリアル接続エラー", str(e))
            return False

    def close_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.get_logger().info("Serial closed")

    def _rx_loop(self):
        stage, buf = 0, bytearray()
        while self.ser and self.ser.is_open:
            try:
                b = self.ser.read(1)
                if not b: continue
                x = b[0]
                if stage == 0 and x == 0xA5:
                    stage = 1
                elif stage == 1 and x == 0xA5:
                    stage = 2
                elif stage == 1:
                    stage = 0
                elif stage == 2:
                    buf = bytearray([x]); stage = 3
                elif stage == 3:
                    buf.append(x)
                    if len(buf) >= 4:
                        cmd, spec, cmd2, spec2 = buf[:4]
                        txt = f"Echo: cmd={cmd}, spec={spec}, cmd2={cmd2}, spec2={spec2}"
                        self.pub_echo.publish(String(data=txt))
                        self.app.update_echo(txt)
                        stage = 0
            except Exception:
                time.sleep(0.05)

    def send_serial(self):
        if not (self.ser and self.ser.is_open): return
        pkt = HEADER + bytes([
            self.cmd1 & 0xFF,
            self.spec1 & 0xFF,
            self.cmd2 & 0xFF,
            self.spec2 & 0xFF
        ])
        try:
            self.ser.write(pkt); self.ser.flush()
        except Exception as e:
            self.get_logger().warn(f"Serial write failed: {e}")

    # ===== CAN処理 =====
    def setup_can(self, port, canif):
        cmds = [
            ["sudo", "modprobe", "slcan"],
            ["sudo", "modprobe", "can"],
            ["sudo", "modprobe", "can_raw"],
            ["sudo", "slcand", "-o", "-c", "-s8", port, canif],
            ["sudo", "ip", "link", "set", canif, "up"]
        ]
        try:
            for cmd in cmds:
                subprocess.run(cmd, check=True)
            self.bus = can.interface.Bus(channel=canif, bustype="socketcan")
            self.app.update_can_status(f"{canif} 有効 ({port})")
            return True
        except Exception as e:
            messagebox.showerror("CAN初期化エラー", str(e))
            self.bus = None
            return False

    def send_can(self, currents):
        if self.bus is None: return
        try:
            data200 = []
            for i in range(4):
                v = int(currents[i])
                data200 += [(v>>8)&0xFF, v&0xFF]
            msg200 = can.Message(arbitration_id=0x200, data=data200, is_extended_id=False)
            self.bus.send(msg200)

            v = int(currents[4])
            data1ff = [(v>>8)&0xFF, v&0xFF] + [0]*6
            msg1ff = can.Message(arbitration_id=0x1FF, data=data1ff, is_extended_id=False)
            self.bus.send(msg1ff)
        except Exception as e:
            self.get_logger().warn(f"CAN send failed: {e}")

    # ===== Subscriberコールバック =====
    def on_motor_cmd(self, msg):
        if len(msg.data) >= 5:
            self.motor_currents = msg.data[:5]
            self.send_can(self.motor_currents)
            self.app.update_motor_labels(self.motor_currents)

    def on_cmdspec1(self, msg):
        if len(msg.data) >= 2:
            self.cmd1, self.spec1 = msg.data[:2]
            self.send_serial()
            self.app.update_cmdspec(self.cmd1, self.spec1, self.cmd2, self.spec2)

    def on_cmdspec2(self, msg):
        if len(msg.data) >= 2:
            self.cmd2, self.spec2 = msg.data[:2]
            self.send_serial()
            self.app.update_cmdspec(self.cmd1, self.spec1, self.cmd2, self.spec2)


# ===================== GUI =====================
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Serial+CAN GUI")
        self.geometry("700x560")
        self.option_add('*Font', ('Segoe UI', 11))

        # 状態変数
        self.port_var = tk.StringVar()
        self.baud_var = tk.IntVar(value=115200)
        self.status_var = tk.StringVar(value="未接続")

        self.can_port_var = tk.StringVar()
        self.can_if_var = tk.StringVar(value="can0")
        self.can_status_var = tk.StringVar(value="未設定")

        self.node = SerialCanGuiNode(self)

        self._build_ui()
        self._refresh_ports()

        # ROS2スレッド起動
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    # ===== UI構築 =====
    def _build_ui(self):
        # --- シリアル設定 ---
        frm_top = ttk.LabelFrame(self, text="シリアル設定", padding=10)
        frm_top.pack(fill="x", padx=10, pady=5)

        ttk.Label(frm_top, text="ポート:").grid(row=0, column=0, sticky="e")
        self.cmb_port = ttk.Combobox(frm_top, textvariable=self.port_var, state="readonly")
        self.cmb_port.grid(row=0, column=1, sticky="ew")
        ttk.Button(frm_top, text="更新", command=self._refresh_ports).grid(row=0, column=2)
        self.btn_connect = ttk.Button(frm_top, text="接続", command=self.on_connect)
        self.btn_connect.grid(row=0, column=3, padx=5)
        ttk.Label(frm_top, text="ボーレート:").grid(row=1, column=0, sticky="e")
        ttk.Entry(frm_top, textvariable=self.baud_var, width=10).grid(row=1, column=1, sticky="w")
        ttk.Label(frm_top, textvariable=self.status_var).grid(row=1, column=2, columnspan=2, sticky="w")

        # --- CAN設定 ---
        frm_can = ttk.LabelFrame(self, text="CAN設定", padding=10)
        frm_can.pack(fill="x", padx=10, pady=5)

        ttk.Label(frm_can, text="USBtoCANポート:").grid(row=0, column=0, sticky="e")
        self.cmb_can_port = ttk.Combobox(frm_can, textvariable=self.can_port_var, state="readonly")
        self.cmb_can_port.grid(row=0, column=1, sticky="ew")
        ttk.Button(frm_can, text="更新", command=self._refresh_ports).grid(row=0, column=2)
        ttk.Label(frm_can, text="IF名:").grid(row=1, column=0, sticky="e")
        ttk.Entry(frm_can, textvariable=self.can_if_var, width=10).grid(row=1, column=1, sticky="w")
        ttk.Button(frm_can, text="CAN有効化", command=self.on_can_enable).grid(row=2, column=0, columnspan=2)
        ttk.Label(frm_can, textvariable=self.can_status_var).grid(row=2, column=2, sticky="w")

        # --- cmd/spec 表示 ---
        frm_cmd = ttk.LabelFrame(self, text="cmd/spec 値", padding=10)
        frm_cmd.pack(fill="x", padx=10, pady=5)
        self.lbl_cmd1 = ttk.Label(frm_cmd, text="cmd1=0"); self.lbl_cmd1.pack(anchor="w")
        self.lbl_spec1 = ttk.Label(frm_cmd, text="spec1=0"); self.lbl_spec1.pack(anchor="w")
        self.lbl_cmd2 = ttk.Label(frm_cmd, text="cmd2=0"); self.lbl_cmd2.pack(anchor="w")
        self.lbl_spec2 = ttk.Label(frm_cmd, text="spec2=0"); self.lbl_spec2.pack(anchor="w")
        self.lbl_echo = ttk.Label(frm_cmd, text="---", foreground="gray"); self.lbl_echo.pack(anchor="w")

        # --- 送信電圧表示 ---
        frm_v = ttk.LabelFrame(self, text="RoboMasterへ送信している電圧", padding=10)
        frm_v.pack(fill="x", padx=10, pady=5)
        # 5ch分のラベルを用意
        self.volt_labels = []
        headers = ["CH1", "CH2", "CH3", "CH4", "CH5"]
        for i, h in enumerate(headers):
            ttk.Label(frm_v, text=f"{h}:").grid(row=i, column=0, sticky="e", padx=4, pady=2)
            lbl = ttk.Label(frm_v, text="0")
            lbl.grid(row=i, column=1, sticky="w")
            self.volt_labels.append(lbl)

    # ===== UIイベント =====
    def _refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.cmb_port["values"] = ports
        self.cmb_can_port["values"] = ports
        if ports:
            if not self.port_var.get():
                self.port_var.set(ports[0])
            if not self.can_port_var.get():
                self.can_port_var.set(ports[0])

    def on_connect(self):
        if not self.node.ser:
            port = self.port_var.get()
            baud = self.baud_var.get()
            if self.node.connect_serial(port, baud):
                self.status_var.set(f"接続中: {port} @ {baud}")
                self.btn_connect.config(text="切断")
        else:
            self.node.close_serial()
            self.status_var.set("未接続")
            self.btn_connect.config(text="接続")

    def on_can_enable(self):
        port = self.can_port_var.get()
        canif = self.can_if_var.get()
        if not port or not canif:
            messagebox.showwarning("未選択", "USBtoCANポートとIF名を指定してください")
            return
        self.node.setup_can(port, canif)

    # ===== 値更新 =====
    def update_motor_labels(self, currents):
        # currentsに受け取った5ch分の電圧(または指令値)を表示
        for i in range(min(5, len(currents))):
            self.volt_labels[i].config(text=str(int(currents[i])))
    def update_cmdspec(self, cmd1, spec1, cmd2, spec2):
        self.lbl_cmd1.config(text=f"cmd1={cmd1}")
        self.lbl_spec1.config(text=f"spec1={spec1}")
        self.lbl_cmd2.config(text=f"cmd2={cmd2}")
        self.lbl_spec2.config(text=f"spec2={spec2}")
    def update_echo(self, txt): self.lbl_echo.config(text=txt)
    def update_can_status(self, txt): self.can_status_var.set(txt)


# ===================== main =====================
def main():
    rclpy.init()
    App().mainloop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
