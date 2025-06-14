######## 2 ir ######################
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO  
import time

# GPIO pin configuration
LEFT_SENSOR_PIN = 17 
RIGHT_SENSOR_PIN = 27

# Initiate the pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_SENSOR_PIN, GPIO.IN)
GPIO.setup(RIGHT_SENSOR_PIN, GPIO.IN)

class IRLaneDepartureNode(Node):
    def __init__(self):
        super().__init__('lane_departure')
        # Publisher
        self.publisher_ = self.create_publisher(String, '/lane_warning', 10)
        # Timer to check lane 0.1s
        self.timer = self.create_timer(0.1, self.check_lane_status)

    def check_lane_status(self):
        left_sensor = GPIO.input(LEFT_SENSOR_PIN)
        right_sensor = GPIO.input(RIGHT_SENSOR_PIN)
        if left_sensor == 1 and right_sensor == 0:
            message = "LEFT"
        elif left_sensor == 0 and right_sensor == 1:
            message = "RIGHT"
        elif left_sensor == 0 and right_sensor == 0:
            message = "CENTER"
        else:
            message = "OUT OF LANE"
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {message}")

    def destroy_node(self):
        """Clean up GPIO on shutdown."""
        GPIO.cleanup()
        super().destroy_node()  

def main(args=None):
    rclpy.init(args=args)
    node = IRLaneDepartureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
