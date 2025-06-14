#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import RPi.GPIO as GPIO
import time

class ParkingSystemNode(Node):
    def __init__(self):
        super().__init__("parking_system_node")
        self.get_logger().info("Parking System Node started")

        # Declare ROS parameters for flexibility
        self.declare_parameter('right_safe_distance', 20.0)  # Min safe right distance (cm)
        self.declare_parameter('linear_speed', 0.2)          # Forward speed (m/s) 
        self.declare_parameter('angular_speed', 0.3)         # Turning speed (rad/s)

        # Retrieve parameters
        self.right_safe_distance = self.get_parameter('right_safe_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # Subscriber for face_monitor topic (to start parking)
        self.face_sub = self.create_subscription(
            String, "/face_monitor", self.face_monitor_callback, 10
        )

        # Subscriber for ultrasonic sensor (front-right)
        self.right_sub = self.create_subscription(
            Float32, 'ultrasonic_right', self.right_callback, 10
        )

        # Motor GPIO setup
        self.ENA = 25  # PWM pin for left motors
        self.IN1 = 23  # Control pin for left motor direction
        self.IN2 = 24  # Control pin for left motor direction
        self.ENB = 21  # PWM pin for right motors
        self.IN3 = 16  # Control pin for right motor direction
        self.IN4 = 20  # Control pin for right motor direction

        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.ENA, GPIO.OUT)
            GPIO.setup(self.IN1, GPIO.OUT)
            GPIO.setup(self.IN2, GPIO.OUT)
            GPIO.setup(self.ENB, GPIO.OUT)
            GPIO.setup(self.IN3, GPIO.OUT)
            GPIO.setup(self.IN4, GPIO.OUT)
            self.pwm_left = GPIO.PWM(self.ENA, 100)
            self.pwm_right = GPIO.PWM(self.ENB, 100)
            self.pwm_left.start(0)
            self.pwm_right.start(0)
        except Exception as e:
            self.get_logger().error(f"Failed to setup motor GPIO: {e}")
            raise

        
        self.TRIG = 5
        self.ECHO = 6
        try:
            GPIO.setup(self.TRIG, GPIO.OUT)
            GPIO.setup(self.ECHO, GPIO.IN)
        except Exception as e:
            self.get_logger().error(f"Failed to setup ultrasonic GPIO: {e}")
            raise

        # State variables
        self.right_distance = None
        self.state = 'moving_straight'  # Possible states: idle, turning, aligning, parked
        self.parking_active = False
        self.align_start_time = None  # To track time in aligning state

        # Timer for main loop (runs every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.main_loop)

    def face_monitor_callback(self, msg):
        """Handle messages from face_monitor topic."""
        if msg.data == "FAINTED":
            self.get_logger().info("FAINTED received, starting parking system")
            self.parking_active = True
            self.state = 'turning'
            self.update_movement()

    def right_callback(self, msg):
        """Update right sensor distance."""
        self.right_distance = msg.data
        if self.parking_active:
            self.update_movement()

    def read_ultrasonic(self):
        """Read distance from ultrasonic sensor (for compatibility, but we'll use ROS topic)."""
        try:
            GPIO.output(self.TRIG, True)
            time.sleep(0.00001)
            GPIO.output(self.TRIG, False)

            start_time = time.time()
            timeout = 0.02
            while GPIO.input(self.ECHO) == 0:
                start_time = time.time()
                if time.time() - start_time > timeout:
                    self.get_logger().warn("Timeout waiting for ECHO to start")
                    return None

            while GPIO.input(self.ECHO) == 1:
                end_time = time.time()
                if time.time() - start_time > timeout:
                    self.get_logger().warn("Timeout waiting for ECHO to end")
                    return None

            duration = end_time - start_time
            distance = (duration * 34300) / 2  # Distance in cm
            if distance < 2.0 or distance > 400.0:
                self.get_logger().warn(f"Invalid distance: {distance:.2f} cm")
                return None

            self.right_distance = distance  # Keep in cm for consistency
            return distance

        except Exception as e:
            self.get_logger().error(f"Error reading distance: {e}")
            return None

    def update_movement(self):
        """Update vehicle movement based on state and sensor data."""
        # Ensure sensor data is available
        if self.right_distance is None:
            self.get_logger().warn(' Waiting for sensor data...')
            return

        # Variables for motor control
        linear = 0.0
        angular = 0.0

        # State machine for parking maneuver
        if self.state == 'moving_straight':
            linear = self.linear_speed
            angular = 0.0

        if self.state == 'turning':
            # Step 1: Move forward while turning right
            if self.right_distance > self.right_safe_distance:
                linear = self.linear_speed
                angular = -self.angular_speed
                self.get_logger().info(' Turning right while moving forward...')
            else:
                # Transition to aligning when right distance is close
                self.state = 'aligning'
                self.align_start_time = time.time()
                self.get_logger().info('Transitioning to alignment...')
        elif self.state == 'aligning':
            # Step 2: Move forward to align with curb
            
            if time.time() - self.align_start_time < 2.0:                      
                linear = self.linear_speed / 2  # Slower speed for precision
                angular = 0.0
                self.get_logger().info('Aligning with curb...')
            else:
                # Transition to parked after 2 seconds
                self.state = 'parked'
                self.get_logger().info(' Parking complete!')
                self.stop_vehicle()
        elif self.state == 'parked':
            # Already parked, do nothing
            return

        # Convert linear and angular speeds to motor speeds
        left_speed = linear + angular
        right_speed = linear - angular

        # Clamp speeds to [-1.0, 1.0]
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        # Set motor directions and PWM
        if left_speed >= 0:
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
            left_pwm = left_speed * 80  # Scale to PWM duty cycle (0-100)
        else:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)
            left_pwm = -left_speed * 80

        if right_speed >= 0:
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
            right_pwm = right_speed * 80
        else:
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.HIGH)
            right_pwm = -right_speed * 80

        # Apply PWM to motors
        self.pwm_left.ChangeDutyCycle(left_pwm)
        self.pwm_right.ChangeDutyCycle(right_pwm)
        self.get_logger().info(f"Motors: left={left_pwm:.1f}%, right={right_pwm:.1f}%")

    def stop_vehicle(self):
        """Stop the vehicle and reset state."""
        self.pwm_left.ChangeDutyCycle(0)
        self.pwm_right.ChangeDutyCycle(0)
        self.parking_active = False
        self.state = 'idle'

    def main_loop(self):
        """Main loop to run parking system tasks."""
        if self.parking_active:
            distance = self.read_ultrasonic()
            self.update_movement()

    def destroy_node(self):
        """Cleanup GPIO on node destruction."""
        try:
            GPIO.cleanup()
        except:
            pass
        self.stop_vehicle()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ParkingSystemNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
