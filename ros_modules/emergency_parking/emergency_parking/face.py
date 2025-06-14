#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class FaceMonitoring(Node):
    def __init__(self):
        super().__init__('face_monitoring_publisher')
        self.cmd_status_pub_ = self.create_publisher(String, '/face_monitor', 10)
        self.fainted = False

     
        self.timer = self.create_timer(1.0, self.timer_callback)

        
        input_thread = threading.Thread(target=self.wait_for_input, daemon=True)
        input_thread.start()

        self.get_logger().info('Type 1 to start publishing "FAINTED" forever.')

    def wait_for_input(self):
        while True:
            user_input = input("Input: ").strip()
            if user_input == '1':
                self.fainted = True
                self.get_logger().info("driver is faintedd")

    def timer_callback(self):
        if self.fainted:
            msg = String()
            msg.data = "FAINTED"
            self.cmd_status_pub_.publish(msg)
            self.get_logger().info('Published: "FAINTED"')

def main(args=None):
    rclpy.init(args=args)
    node = FaceMonitoring()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
