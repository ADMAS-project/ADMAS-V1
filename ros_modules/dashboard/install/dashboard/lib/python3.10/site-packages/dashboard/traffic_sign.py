##############################################
##### publish sign to road_sign_topic ########
##############################################
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
from ultralytics import YOLO

class TrafficSignDetector(Node):
    def __init__(self):
        super().__init__('traffic_sign_detector')

        # ROS publisher
        self.publisher_ = self.create_publisher(String, 'road_sign_topic', 10)

        # Load ai model
        model_path = "/home/ros/ros2_ws/src/ai_model/best_int8.tflite"
        self.model = YOLO(model_path)

        # Open webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Error: no camera.")
            rclpy.shutdown()
            return

        self.timer = self.create_timer(0.1, self.detect_signs)

        self.get_logger().info("Traffic Sign Detector started.")

    def detect_signs(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Camera read failed.")
            return

        results = self.model(frame)
        detected = []

        for result in results:
            for box in result.boxes:
                conf = box.conf[0].item()
                if conf < 0.6:  # 60%min Ignore low confidence detections 
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                class_id = int(box.cls[0].item())
                class_name = result.names[class_id]

                detected.append(class_name)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{class_name} ({conf:.2f})",
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish detected signs
        if detected:
            msg = String()
            msg.data = ", ".join(detected)
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")

        # Display the frame
        cv2.imshow("Traffic Sign Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.shutdown()

    def shutdown(self):
        self.get_logger().info("Shutting down...")
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()

rclpy.init()
node = TrafficSignDetector()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    node.get_logger().info("Interrupted. Exiting...")
finally:
    node.shutdown()
