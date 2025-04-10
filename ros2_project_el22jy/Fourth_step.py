import threading
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal

class Robot(Node):
    def __init__(self):
        super().__init__('robot')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)

        # Flags
        self.green_detected = False
        self.blue_detected = False
        self.move_forward = False
        self.move_backward = False

        self.sensitivity = 15

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges
        green_lower = np.array([60 - self.sensitivity, 100, 100])
        green_upper = np.array([60 + self.sensitivity, 255, 255])
        blue_lower = np.array([100, 100, 100])
        blue_upper = np.array([140, 255, 255])

        # Create masks
        green_mask = cv2.inRange(hsv, green_lower, green_upper)
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

        # Display camera feed
        cv2.imshow('Camera Feed', image)
        cv2.imshow('Green Mask', green_mask)
        cv2.imshow('Blue Mask', blue_mask)
        cv2.waitKey(3)

        # Reset flags
        self.green_detected = False
        self.blue_detected = False
        self.move_forward = False
        self.move_backward = False

        # Process green contours
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if green_contours:
            largest = max(green_contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 300:
                self.green_detected = True
                M = cv2.moments(largest)

                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)

                    print(f"[INFO] Green object detected at ({cx}, {cy}) | Area: {int(area)}")

                    if area > 2000:
                        self.move_backward = True
                    else:
                        self.move_forward = True

        # Process blue contours
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if blue_contours:
            largest_blue = max(blue_contours, key=cv2.contourArea)
            area_blue = cv2.contourArea(largest_blue)
            if area_blue > 300:
                self.blue_detected = True
                print(f"[INFO] Blue object detected. Stopping.")

    def walk_forward(self):
        twist = Twist()
        twist.linear.x = 0.2
        self.publisher.publish(twist)

    def walk_backward(self):
        twist = Twist()
        twist.linear.x = -0.2
        self.publisher.publish(twist)

    def stop(self):
        twist = Twist()
        self.publisher.publish(twist)

def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    rclpy.init(args=None)
    global robot
    robot = Robot()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            if robot.blue_detected:
                robot.stop()
            elif robot.green_detected:
                if robot.move_backward:
                    robot.walk_backward()
                elif robot.move_forward:
                    robot.walk_forward()
                else:
                    robot.stop()
            else:
                robot.stop()
            time.sleep(0.1)

    except ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

