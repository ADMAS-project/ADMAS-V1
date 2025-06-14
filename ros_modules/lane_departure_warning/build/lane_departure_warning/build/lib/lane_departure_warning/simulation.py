#########################
### a simulation ########
#########################
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os  

class IRLaneDepartureNode(Node):
    def __init__(self):
        super().__init__('lane_departure')
        
        # Publisher
        self.publisher_ = self.create_publisher(String, '/lane_warning', 10)

        self.left_ir_sensor = 0  
        self.right_ir_sensor = 0  

        self.timer = self.create_timer(0.1, self.check_lane_status)

    def check_lane_status(self):
        # we but here the real sensor data and we get it from that block ''
       
        # Simulate fake sensor data
        self.left_ir_sensor = 0
        self.right_ir_sensor = 1 # right sensor on white line

        # logic of getting lane status
        if self.left_ir_sensor == 0 and self.right_ir_sensor == 0:
            status = "OK"
        elif self.left_ir_sensor == 1 and self.right_ir_sensor == 0:
            status = "LEFT_DEPARTURE"
        elif self.left_ir_sensor == 0 and self.right_ir_sensor == 1:
            status = "RIGHT_DEPARTURE"
        else:
            status = "error"

        # Publish the status
        msg = String()
        msg.data = status
        self.publisher_.publish(msg)
        self.get_logger().info(f"Lane Status: {status} (Left: {self.left_ir_sensor}, Right: {self.right_ir_sensor})")

        # buzzer sound 
        if status != "OK":
           
            os.system("aplay /home/ros/ros2_ws/src/lane_departure_warning/beep.wav 2>/dev/null")  

    def destroy_node(self):
        """Clean up on shutdown."""
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
    
