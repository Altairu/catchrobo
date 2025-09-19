#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int16MultiArray
import tkinter as tk
from tkinter import ttk

class DebugGuiNode(Node):
    def __init__(self):
        super().__init__("debug_gui_node")

        self.pub_cmd = self.create_publisher(Int32MultiArray, "cmd_spec", 10)
        self.pub_motor = self.create_publisher(Int16MultiArray, "motor_currents", 10)

        self.root = tk.Tk()
        self.root.title("Debug GUI")

        self.cmd = tk.IntVar(value=0)
        self.spec = tk.IntVar(value=0)

        frm = ttk.Frame(self.root)
        frm.pack(padx=10, pady=10)

        ttk.Label(frm, text="Cmd").grid(row=0, column=0)
        tk.Entry(frm, textvariable=self.cmd).grid(row=0, column=1)
        ttk.Label(frm, text="Spec").grid(row=1, column=0)
        tk.Entry(frm, textvariable=self.spec).grid(row=1, column=1)
        ttk.Button(frm, text="Send Cmd/Spec", command=self.send_cmd_spec).grid(row=2, column=0, columnspan=2)

        self.motor_vars = [tk.IntVar(value=0) for _ in range(5)]
        for i,var in enumerate(self.motor_vars):
            ttk.Label(frm, text=f"Motor{i+1}").grid(row=3+i, column=0)
            tk.Entry(frm, textvariable=var).grid(row=3+i, column=1)
        ttk.Button(frm, text="Send Motors", command=self.send_motors).grid(row=8, column=0, columnspan=2)

        self.root.after(100, self.spin_once)

    def send_cmd_spec(self):
        msg = Int32MultiArray()
        msg.data = [self.cmd.get(), self.spec.get()]
        self.pub_cmd.publish(msg)

    def send_motors(self):
        msg = Int16MultiArray()
        msg.data = [v.get() for v in self.motor_vars]
        self.pub_motor.publish(msg)

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(10, self.spin_once)

def main():
    rclpy.init()
    node = DebugGuiNode()
    node.root.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
