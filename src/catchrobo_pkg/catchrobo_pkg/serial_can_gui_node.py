#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int32MultiArray, String

import threading, time, subprocess
import serial
from serial.tools import list_ports
import can
import tkinter as tk
from tkinter import ttk, messagebox


HEADER = bytes([0xA5, 0xA5])


class SerialCanGuiNode(Node):
    def __init__(self, app):
        super().__init__('serial_can_gui_node')
        self.app = app

        # ---- Subscriber ----
        self.sub_motor = self.create_subscription(Int16MultiArray, 'motor_cmd', self.on_motor_cmd, 10)
        self.sub_cmdspec = self.create_subscription(Int32MultiArray, 'cmd_spec', self.on_cmd_spec, 10)
        self.pub_echo = self.create_publisher(String, 'echo', 10)

        # ---- 接続状態 ----
        self.ser = None
        self.bus = None

        # 状態保持
        self.cmd_val = 0
        self.spec_val = 0
        self.motor_currents = [0,0,0,0,0]

    # ================== Serial ==================
    def connect_serial(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
            self.get_logger().info(f"Serial connected: {port}")
            threading.Thread(target=self._rx_loop, daemon=True).start()
            return True
        except Exception as e:
            messagebox.showerror("シリアル接続エラー", str(e))
            return False

    def _rx_loop(self):
        stage, buf = 0, bytearray()
        while self.ser and self.ser.is_open:
            try:
                b = self.ser.read(1)
                if not b: continue
                x = b[0]
                if stage == 0 and x == 0xA5: stage = 1
                elif stage == 1 and x == 0xA5: stage = 2
                elif stage == 1: stage = 0
                elif stage == 2: buf = bytearray([x]); stage = 3
                elif stage == 3:
                    buf.append(x)
                    cmd, spec = buf[0], buf[1]
                    txt = f"Echo: cmd={cmd}, spec={spec}"
                    self.pub_echo.publish(String(data=txt))
                    self.app.update_echo(txt)
                    stage = 0
            except Exception:
                time.sleep(0.05)

    def send_serial(self, cmd, spec):
        if not (self.ser and self.ser.is_open): return
        pkt = HEADER + bytes([cmd & 0xFF, spec & 0xFF])
        try:
            self.ser.write(pkt); self.ser.flush()
        except Exception as e:
            self.get_logger().warn(f"Serial write failed: {e}")

    # ================== CAN ==================
    def setup_can(self, port, canif):
        cmds = [
            ["sudo","modprobe","slcan"],
            ["sudo","modprobe","can"],
            ["sudo","modprobe","can_raw"],
            ["sudo","slcand","-o","-c","-s8", port, canif],
            ["sudo","ip","link","set",canif,"up"]
        ]
        try:
            for cmd in cmds: subprocess.run(cmd, check=True)
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
            # Motor1-4
            data200 = []
            for i in range(4):
                v = int(currents[i])
                data200 += [(v>>8)&0xFF, v&0xFF]
            msg200 = can.Message(arbitration_id=0x200, data=data200, is_extended_id=False)
            self.bus.send(msg200)

            # Motor5
            v = int(currents[4])
            data1ff = [(v>>8)&0xFF, v&0xFF] + [0]*6
            msg1ff = can.Message(arbitration_id=0x1FF, data=data1ff, is_extended_id=False)
            self.bus.send(msg1ff)
        except Exception as e:
            self.get_logger().warn(f"CAN send failed: {e}")

    # ================== Callback ==================
    def on_motor_cmd(self, msg):
        if len(msg.data) >= 5:
            self.motor_currents = msg.data[:5]
            self.send_can(self.motor_currents)
            self.app.update_motor_labels(self.motor_currents)

    def on_cmd_spec(self, msg):
        if len(msg.data) >= 2:
            self.cmd_val, self.spec_val = msg.data[:2]
            self.send_serial(self.cmd_val, self.spec_val)
            self.app.update_cmdspec(self.cmd_val, self.spec_val)


# ===================== GUI =====================
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Serial+CAN GUI")
        self.geometry("600x400")
        self.option_add('*Font', ('Segoe UI', 11))

        # 変数
        self.port_var = tk.StringVar()
        self.baud_var = tk.IntVar(value=115200)
        self.can_port_var = tk.StringVar()
        self.can_if_var = tk.StringVar(value="can0")
        self.status_var = tk.StringVar(value="未接続")
        self.can_status_var = tk.StringVar(value="未設定")

        # ノード生成
        self.node = SerialCanGuiNode(self)

        # UI構築
        self._build_ui()
        self._refresh_ports()

        # ROSスレッド
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def update_can_status(self, txt: str):
        self.can_status_var.set(txt)


    def _build_ui(self):
        frm = ttk.Frame(self, padding=10); frm.pack(fill="both", expand=True)

        # Serial
        ttk.Label(frm,text="シリアル:").grid(row=0,column=0,sticky="e")
        self.cmb_serial = ttk.Combobox(frm,textvariable=self.port_var,state="readonly")
        self.cmb_serial.grid(row=0,column=1)
        ttk.Button(frm,text="更新",command=self._refresh_ports).grid(row=0,column=2)
        ttk.Button(frm,text="接続",command=self.on_connect_serial).grid(row=0,column=3)

        # CAN
        ttk.Label(frm,text="USBtoCAN:").grid(row=1,column=0,sticky="e")
        self.cmb_can = ttk.Combobox(frm,textvariable=self.can_port_var,state="readonly")
        self.cmb_can.grid(row=1,column=1)
        ttk.Button(frm,text="更新",command=self._refresh_ports).grid(row=1,column=2)
        ttk.Entry(frm,textvariable=self.can_if_var,width=8).grid(row=1,column=3)
        ttk.Button(frm,text="CAN有効化",command=self.on_setup_can).grid(row=1,column=4)

        ttk.Label(frm,textvariable=self.can_status_var).grid(row=2,column=1,columnspan=3,sticky="w")

        # CMD/SPEC
        ttk.Label(frm,text="cmd:").grid(row=3,column=0,sticky="e")
        self.lbl_cmd = ttk.Label(frm,text="0"); self.lbl_cmd.grid(row=3,column=1,sticky="w")
        ttk.Label(frm,text="spec:").grid(row=4,column=0,sticky="e")
        self.lbl_spec = ttk.Label(frm,text="0"); self.lbl_spec.grid(row=4,column=1,sticky="w")

        # Motors
        self.motor_labels = []
        for i in range(5):
            ttk.Label(frm,text=f"Motor{i+1}:").grid(row=5+i,column=0,sticky="e")
            lbl = ttk.Label(frm,text="0"); lbl.grid(row=5+i,column=1,sticky="w")
            self.motor_labels.append(lbl)

        # Echo
        ttk.Label(frm,text="Echo:").grid(row=10,column=0,sticky="e")
        self.lbl_echo = ttk.Label(frm,text="---",foreground="gray"); self.lbl_echo.grid(row=10,column=1,sticky="w")

    # ====== Event Handlers ======
    def _refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.cmb_serial["values"] = ports
        self.cmb_can["values"] = ports
        if ports and not self.port_var.get(): self.port_var.set(ports[0])
        if ports and not self.can_port_var.get(): self.can_port_var.set(ports[0])

    def on_connect_serial(self):
        port = self.port_var.get().strip()
        baud = int(self.baud_var.get())
        if self.node.connect_serial(port, baud):
            self.status_var.set(f"接続: {port}@{baud}")

    def on_setup_can(self):
        port = self.can_port_var.get().strip()
        canif = self.can_if_var.get().strip()
        if self.node.setup_can(port, canif):
            self.can_status_var.set(f"{canif} 有効 ({port})")

    # ====== UI Update ======
    def update_echo(self, txt): self.lbl_echo.config(text=txt)
    def update_motor_labels(self, currents):
        for i,v in enumerate(currents): self.motor_labels[i].config(text=str(v))
    def update_cmdspec(self, cmd, spec):
        self.lbl_cmd.config(text=str(cmd))
        self.lbl_spec.config(text=str(spec))


def main():
    rclpy.init()
    App().mainloop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
