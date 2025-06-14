#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import font
from PIL import Image, ImageTk
import os
import datetime

class Dashboard(Node):
    def __init__(self):
        super().__init__('dashboard')
        self.get_logger().info("**Dashboard Node Started**")

        # Tkinter root
        self.root = tk.Tk()
        self.root.attributes("-fullscreen", True)
        self.root.overrideredirect(True)  

        self.window_width = self.root.winfo_screenwidth()
        self.window_height = self.root.winfo_screenheight()

        # Paths
        self.background_image_path = "/home/admas/data/bg.png"
        self.sign_image_path_prefix = "/home/admas/data/"

        # Fonts and Colors
        self.title_font = font.Font(family="Courier New", size=13, weight="normal")
        self.datetime_font = font.Font(family="Courier New", size=12, weight="normal")
        self.section_font = font.Font(family="Helvetica", size=22, weight="bold")
        self.status_font = font.Font(family="Arial", size=19, weight="bold")
        self.text_color = "#0096FF"
        self.normal_color = "white"
        self.alert_color = "#FF5F1F"

        # Canvas
        self.canvas = tk.Canvas(self.root, width=self.window_width, height=self.window_height, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        # Background
        try:
            bg_img = Image.open(self.background_image_path)
            bg_img = bg_img.resize((self.window_width, self.window_height), Image.Resampling.LANCZOS)
            self.bg_photo = ImageTk.PhotoImage(bg_img)
            self.canvas.create_image(0, 0, image=self.bg_photo, anchor="nw")
        except Exception as e:
            self.canvas.configure(bg="black")
            self.get_logger().error(f"Background load error: {e}")

        # Header
        self.canvas.create_text(self.window_width / 2, 35, text="ADMAS GP", font=self.title_font, fill=self.text_color)
        self.datetime_id = self.canvas.create_text(self.window_width / 2, 55, text="", font=self.datetime_font, fill=self.text_color)
        self.update_datetime()

        # Fatigue section
        self.canvas.create_text(self.window_width / 4, 130, text="DRIVER STATUS", font=self.section_font, fill=self.text_color)
        self.fatigue_info_id = self.canvas.create_text(self.window_width / 4, 170, text="Status: OK", font=self.status_font, fill=self.normal_color)

        # Lane section
        self.canvas.create_text(3 * self.window_width / 4, 130, text="LANE DEPARTURE", font=self.section_font, fill=self.text_color)
        self.lane_info_id = self.canvas.create_text(3 * self.window_width / 4, 170, text="Status: None", font=self.status_font, fill=self.normal_color)

        # Sign section
        self.sign_image_id = self.canvas.create_image(self.window_width / 2 , 275, anchor="center")
        self.sign_info_id = self.canvas.create_text(self.window_width / 2 , 360, text="Detected sign: None", font=self.status_font, fill=self.normal_color, anchor="center")
        self.current_sign_tk_img = None

        # ROS 2 subscribers
        self.sign_subscription = self.create_subscription(String, "road_sign_topic", self.sign_callback, 10)
        self.fatigue_subscription = self.create_subscription(String, "/face_monitor", self.fatigue_callback, 10)
        self.lane_subscription = self.create_subscription(String, "/lane_warning", self.lane_callback, 10)

        # Timestamps
        self.last_sign_detection_time = self.get_clock().now()
        self.last_fatigue_detection_time = self.get_clock().now()
        self.last_lane_detection_time = self.get_clock().now()

        # Timers
        self.root.after(100, self.ros_spin)
        self.root.after(5000, self.check_timeout)
        self.root.after(1000, self.update_datetime)
        self.root.mainloop()

    def update_datetime(self):
        now = datetime.datetime.now()
        datetime_str = now.strftime("%B %d, %Y %I:%M %p")
        self.canvas.itemconfig(self.datetime_id, text=datetime_str)
        self.root.after(1000, self.update_datetime)

    def sign_callback(self, msg):
        sign = msg.data
        self.canvas.itemconfig(self.sign_info_id, text=f"Be careful: {sign}", fill='white')
        image_path = f"{self.sign_image_path_prefix}{sign}.png"
        self.current_sign_tk_img = None
        if os.path.exists(image_path):
            try:
                img = Image.open(image_path).resize((130, 145), Image.Resampling.LANCZOS)
                self.current_sign_tk_img = ImageTk.PhotoImage(img)
                self.canvas.itemconfig(self.sign_image_id, image=self.current_sign_tk_img)
                self.get_logger().info(f"Displayed sign: {sign}")
            except Exception as e:
                self.get_logger().error(f"Image error: {e}")
        else:
            self.canvas.itemconfig(self.sign_image_id, image='')
        self.last_sign_detection_time = self.get_clock().now()

    def fatigue_callback(self, msg):
        status = msg.data
        warning_text = "Warning: "
        if status == "SLEEPING":
            warning_text += "Driver is sleeping"
        elif status == "FAINTED":
            warning_text += "Driver is fainted"
        elif status == "RIGHT":
            warning_text += "Driver is looking right"
        elif status == "LEFT":
            warning_text += "Driver is looking left"
        elif status == "YAWN":
            warning_text += "Driver is yawning"
        else:
            warning_text += status
        self.canvas.itemconfig(self.fatigue_info_id, text=warning_text, fill=self.alert_color)
        self.get_logger().warn(f"Fatigue: {warning_text}")
        self.last_fatigue_detection_time = self.get_clock().now()

    def lane_callback(self, msg):
        status = msg.data
        lane_text = f"WARNING: {status}"
        self.canvas.itemconfig(self.lane_info_id, text=lane_text, fill=self.alert_color)
        self.get_logger().warn(f"Lane: {lane_text}")
        self.last_lane_detection_time = self.get_clock().now()

    def check_timeout(self):
        now = self.get_clock().now()
        if (now - self.last_sign_detection_time).nanoseconds / 1e9 > 6:
            self.canvas.itemconfig(self.sign_info_id, text="Detected sign: None", fill=self.normal_color)
            self.canvas.itemconfig(self.sign_image_id, image='')
        if (now - self.last_fatigue_detection_time).nanoseconds / 1e9 > 6:
            self.canvas.itemconfig(self.fatigue_info_id, text="Status: OK", fill=self.normal_color)
        if (now - self.last_lane_detection_time).nanoseconds / 1e9 > 6:
            self.canvas.itemconfig(self.lane_info_id, text="Status: None", fill=self.normal_color)
        self.root.after(5000, self.check_timeout)

    def ros_spin(self):
        try:
            rclpy.spin_once(self, timeout_sec=0.1)
        except Exception as e:
            self.get_logger().error(f"ROS spin error: {e}")
        self.root.after(100, self.ros_spin)

    def destroy_node(self):
        try:
            self.root.destroy()
        except:
            pass
        super().destroy_node()

if __name__ == "__main__":
    rclpy.init()
    try:
        node = Dashboard()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

