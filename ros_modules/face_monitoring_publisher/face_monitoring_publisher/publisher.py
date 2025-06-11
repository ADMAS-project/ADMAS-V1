#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .face_monitor import *
from subprocess import call

class FaceMonitoring(Node):
    sleep_signal = False
    def __init__(self):
        super().__init__("face_monitoring_publisher")
        self.monitor_obj_ = FaceMonitor()
        self.cmd_status_pub_ = self.create_publisher(String, "/face_monitor", 10)
        self.face_monitoring_workflow()
        self.sleep_signal = False

    def timer_callback(self):
        msg = String()
        msg.data = "HAHAHAHAHAHA"
        self.cmd_status_pub_.publish(msg)
        self.get_logger().info("Hello")

    def face_monitoring_workflow(self):
        global_timer = GlobalTimer()
        while True:
            _, frame = self.monitor_obj_.cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.monitor_obj_.detector(gray)

            for face in faces:

                landmarks = self.monitor_obj_.predictor(gray, face)

                left_eye_ratio = self.monitor_obj_.get_blinking_ratio([36, 37, 38, 39, 40, 41], landmarks, frame)
                right_eye_ratio = self.monitor_obj_.get_blinking_ratio([42, 43, 44, 45, 46, 47], landmarks, frame)
                blinking_ratio = (left_eye_ratio + right_eye_ratio) / 2

                left_eye_gaze = self.monitor_obj_.get_gaze_ratio([36, 37, 38, 39, 40, 41], landmarks, frame, gray)
                right_eye_gaze = self.monitor_obj_.get_gaze_ratio([42, 43, 44, 45, 46, 47], landmarks, frame, gray)
                gaze_ratio = (left_eye_gaze + right_eye_gaze) / 2

                mouth_ratio = self.monitor_obj_.mouth([48,49,50,51,52,53,54,55,56,57,58,59], landmarks, frame)

                #cv2.putText(frame, str(gaze_ratio), (50, 100), 0, 2, (0, 255, 0), 3)

                if blinking_ratio > 1.5:
                    cv2.putText(frame, "BLINKING", (0, 480), 0, 2, (255, 0, 0), 3)
                    if not self.sleep_signal:
                        sleep_timer = global_timer.init_if_none("blink", 3, self.sleep_callback)
                        self.sleep_signal = sleep_timer.sense()
                        global_timer.reset_except("blink")
                        continue
                    faint_timer = global_timer.init_if_none("faint", 3, self.faint_callback)
                    faint_timer.sense()
                    global_timer.reset_except("faint")
                    continue
                elif gaze_ratio < 0.5:
                    cv2.putText(frame, "RIGHT", (420, 50), 0, 2, (0, 0, 255), 3)
                    timer = global_timer.init_if_none("right", 3, self.right_dir_callback)
                    timer.sense()
                    global_timer.reset_except("right")
                    continue
                elif gaze_ratio > 2:
                    cv2.putText(frame, "LEFT", (420, 50), 0, 2, (0, 0, 255), 3)
                    timer = global_timer.init_if_none("left", 10, self.left_dir_callback)
                    timer.sense()
                    global_timer.reset_except("left")
                    continue
                else:
                    cv2.putText(frame, "CENTER", (420, 50), 0, 2, (0, 0, 255), 3)         
                    self.sleep_signal = False



                #cv2.putText(frame, str(mouth_ratio), (150, 300) ,0 ,4, (255, 0, 0), 3)
                if mouth_ratio < 1.2:
                    cv2.putText(frame, "yawning", (400, 460), 0, 2, (255, 0, 0), 3)
                    # CAN REPLACE THE BOTTOM WITH COUNTER
                    timer = global_timer.init_if_none("yawn", 3, self.yawn_callback)
                    timer.sense()
                    global_timer.reset_except("yawn")
                    continue


                global_timer.reset_all_timers()
                self.sleep_signal = False
            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1)
            if key == 27:
                break

        self.monitor_obj_.cap.release()
        cv2.destroyAllWindows()    

    def yawn_callback(self):
        msg = String()
        msg.data = "YAWN"
        self.cmd_status_pub_.publish(msg)

    
    def right_dir_callback(self):
        msg = String()
        msg.data = "RIGHT"
        self.cmd_status_pub_.publish(msg)

    def left_dir_callback(self):
        msg = String()
        msg.data = "LEFT"
        self.cmd_status_pub_.publish(msg)
    
    def sleep_callback(self):
        msg = String()
        msg.data = "SLEEPING"
        self.cmd_status_pub_.publish(msg)

    def faint_callback(self):
        msg = String()
        msg.data = "FAINTED"
        self.cmd_status_pub_.publish(msg)


def speak(phrase):
    call(["espeak", "-v", "mb-us1", "Wake up", "-s", "110", "-p", "120", phrase])




# Test receiver
class FaceMonitorRecv(Node):
    def __init__(self):
        super().__init__("face_monitoring_subscriper")
        self.subscriper_ = self.create_subscription(String, "/face_monitor", self.callback, 10)
    
    def callback(self, msg):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)

    node = FaceMonitoring()
    rclpy.spin(node)

    rclpy.shutdown()

def listener(args=None):
    rclpy.init(args=args)

    node = FaceMonitorRecv()
    rclpy.spin(node)

    rclpy.shutdown()



if __name__ == "__main__":
    main()
