import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import math
import signal

class Robot(Node):
    def __init__(self):
        super().__init__('robot')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sensitivity = 15
        self.green_lower = np.array([60 - self.sensitivity, 100, 100])
        self.green_upper = np.array([60 + self.sensitivity, 255, 255])
        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 100, 100])
        self.red_upper2 = np.array([179, 255, 255])
        self.blue_lower = np.array([110 - self.sensitivity, 150, 50])
        self.blue_upper = np.array([130 + self.sensitivity, 255, 255])

        self.blue_offset = 0
        self.moveForwardsFlag = False
        self.moveBackwardsFlag = False
        self.need_align_flag = False

        self.seen_green = False
        self.seen_red = False
        self.path_finished = False

        threading.Thread(target=self.run_path, daemon=True).start()

    def callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        red_mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        blue_mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)

        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if green_contours:
            area = cv2.contourArea(max(green_contours, key=cv2.contourArea))
            if area > 500:
                self.seen_green = True

        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if red_contours:
            area = cv2.contourArea(max(red_contours, key=cv2.contourArea))
            if area > 500:
                self.seen_red = True

        self.moveForwardsFlag = False
        self.moveBackwardsFlag = False
        self.need_align_flag = False

        if self.path_finished and self.seen_green and self.seen_red:
            blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            if blue_contours:
                c = max(blue_contours, key=cv2.contourArea)
                area = cv2.contourArea(c)

                if area > 500:
                    height, width, _ = image.shape
                    image_center_x = width // 2

                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        self.blue_offset = cx - image_center_x

                        if abs(self.blue_offset) > 20:
                            self.need_align_flag = True

                    if area > 280000:
                        self.moveBackwardsFlag = True
                    elif area < 260000:
                        self.moveForwardsFlag = True

        self.draw_coloured_object(image, red_mask, (0, 0, 255), "Red")
        self.draw_coloured_object(image, green_mask, (0, 255, 0), "Green")
        self.draw_coloured_object(image, blue_mask, (255, 0, 0), "Blue")
        cv2.imshow("Camera Feed", image)
        cv2.waitKey(3)

    def draw_coloured_object(self, image, mask, color, label):
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 500:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                cv2.putText(image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    def move(self, linear=0.0, angular=0.0, duration=1.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)
        time.sleep(duration)
        self.stop()
        time.sleep(0.5)

    def rotate_angle(self, angle_deg):
        angular_speed = 0.2
        angle_rad = math.radians(angle_deg)
        duration = abs(angle_rad / angular_speed)
        direction = 1.0 if angle_deg > 0 else -1.0
        self.move(angular=direction * angular_speed, duration=duration)

    def walk_forward(self):
        twist = Twist()
        twist.linear.x = 0.2
        self.publisher.publish(twist)

    def walk_backward(self):
        twist = Twist()
        twist.linear.x = -0.2
        self.publisher.publish(twist)

    def align_to_blue(self):
        twist = Twist()
        twist.angular.z = -0.002 * self.blue_offset
        self.publisher.publish(twist)

    def stop(self):
        self.publisher.publish(Twist())
        time.sleep(0.1)

    def run_path(self):
        self.rotate_angle(90)
        self.move(linear=0.5, duration=10)
        self.rotate_angle(-90)
        self.move(linear=0.5, duration=10)
        self.rotate_angle(-90)
        self.move(linear=0.5, duration=10)
        self.rotate_angle(-90)
        self.move(linear=0.5, duration=12)
        self.rotate_angle(90)
        self.move(linear=0.5, duration=10)
        self.rotate_angle(90)
        self.move(linear=0.5, duration=10)
        self.rotate_angle(-90)
        self.move(linear=0.5, duration=2)
        self.rotate_angle(-90)
        self.move(linear=0.5, duration=10)
        self.rotate_angle(-90)
        self.move(linear=0.5, duration=6)
        self.rotate_angle(90)
        self.move(linear=0.5, duration=12)
        self.rotate_angle(-90)
        self.move(linear=0.5, duration=4)
        self.rotate_angle(180)
        self.move(linear=0.5, duration=16)
        self.rotate_angle(90)

        self.get_logger().info("FINISH MISSION START BLUE SEARCHING")
        self.path_finished = True

def main():
    rclpy.init()
    robot = Robot()

    def shutdown(sig, frame):
        robot.stop()
        cv2.destroyAllWindows()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, shutdown)

    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            if robot.path_finished and robot.seen_green and robot.seen_red:
                if robot.need_align_flag:
                    robot.align_to_blue()
                elif robot.moveBackwardsFlag:
                    robot.walk_backward()
                elif robot.moveForwardsFlag:
                    robot.walk_forward()

            time.sleep(0.1)

    except KeyboardInterrupt:
        shutdown(None, None)

if __name__ == '__main__':
    main()

