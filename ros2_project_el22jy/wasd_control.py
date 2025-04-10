# Exercise 4 - following a colour (green) and stopping upon sight of another (blue).

import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.exceptions import ROSInterruptException
import signal
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from math import sin, cos


class Robot(Node):
    def __init__(self):
        super().__init__('robot')

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.sensitivity = 15

        # HSV ranges
        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 100, 100])
        self.red_upper2 = np.array([179, 255, 255])
        self.green_lower = np.array([60 - self.sensitivity, 100, 100])
        self.green_upper = np.array([60 + self.sensitivity, 255, 255])
        self.blue_lower = np.array([110 - self.sensitivity, 150, 50])
        self.blue_upper = np.array([130 + self.sensitivity, 255, 255])

        # Flags and states
        self.goal_done = False
        self.goal_sent = False
        self.colour1_flag = 0
        self.myColourFlag = False
        self.circle_around_blueFlag = False
        self.moveForwardsFlag = False
        self.moveBackwardsFlag = False
        self.blue_area = 0
        self.blue_offset = 0
        self.need_align_flag = False

        # ROS topics
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)

    def callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.namedWindow('camera_Feed', cv2.WINDOW_NORMAL)
        annotated_image = self.detect_and_draw_rgb(image)
        cv2.imshow('camera_Feed', annotated_image)
        cv2.resizeWindow('camera_Feed', 320, 240)
        cv2.waitKey(3)

        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        red_mask1 = cv2.inRange(Hsv_image, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(Hsv_image, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        green_mask = cv2.inRange(Hsv_image, self.green_lower, self.green_upper)
        blue_mask = cv2.inRange(Hsv_image, self.blue_lower, self.blue_upper)

        self.myColourFlag = False
        self.moveForwardsFlag = False
        self.moveBackwardsFlag = False
        self.need_align_flag = False

        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if blue_contours:
            c = max(blue_contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > 500:
                height, width, _ = image.shape
                image_center_x = width // 2

                M = cv2.moments(c)
                cx = image_center_x
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    self.blue_offset = cx - image_center_x

                if abs(self.blue_offset) > 20:
                    self.need_align_flag = True
                else:
                    self.need_align_flag = False

                if area > 280000:
                    self.moveBackwardsFlag = True
                    self.moveForwardsFlag = False
                    self.circle_around_blueFlag = False
                elif 260000 < area <= 280000:
                    self.moveBackwardsFlag = False
                    self.moveForwardsFlag = False
                    self.circle_around_blueFlag = False
                elif area < 260000:
                    self.moveForwardsFlag = True
                    self.moveBackwardsFlag = False
                    self.circle_around_blueFlag = False

    def walk_forward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2
        self.publisher.publish(desired_velocity)

    def walk_backward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.2
        self.publisher.publish(desired_velocity)

    def stop(self):
        self.publisher.publish(Twist())

    def align_to_blue(self):
        twist = Twist()
        twist.angular.z = -0.002 * self.blue_offset
        self.publisher.publish(twist)

    def detect_and_draw_rgb(self, image):
        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        red_mask1 = cv2.inRange(Hsv_image, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(Hsv_image, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        green_mask = cv2.inRange(Hsv_image, self.green_lower, self.green_upper)
        blue_mask = cv2.inRange(Hsv_image, self.blue_lower, self.blue_upper)

        self.draw_coloured_object(image, red_mask, (0, 0, 255), "Red")
        self.draw_coloured_object(image, green_mask, (0, 255, 0), "Green")
        self.draw_coloured_object(image, blue_mask, (255, 0, 0), "Blue")
        return image

    def draw_coloured_object(self, image, mask, color, label):
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 500:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                cv2.putText(image, label, (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        self.get_logger().info(f"send goal: ({x}, {y}, {yaw})")
        self.goal_done = False

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Navigation result: {result}")
        self.goal_done = True


def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    rclpy.init()
    robot = Robot()
    signal.signal(signal.SIGINT, signal_handler)

    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    # Slightly adjusted waypoints
    goals = [
        (-1.4, -5.4, 0.0),
        (0.3, -6.1, 0.0),
        (-4.8, -1.5, 0.0),
        (-8.1, -4.5, 0.0),
        (-6.0, -6.4, 0.0)
    ]

    current_goal_index = 0
    robot.goal_sent = False
    robot.goal_done = True

    try:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)

            if robot.need_align_flag:
                robot.align_to_blue()
            elif robot.moveBackwardsFlag:
                robot.walk_backward()
            elif robot.moveForwardsFlag:
                robot.walk_forward()
            else:
                robot.stop()

            if current_goal_index < len(goals):
                if not robot.goal_sent and robot.goal_done:
                    x, y, yaw = goals[current_goal_index]
                    robot.send_goal(x, y, yaw)
                    robot.goal_sent = True
                    robot.goal_done = False
                elif robot.goal_done:
                    current_goal_index += 1
                    robot.goal_sent = False

            time.sleep(0.1)

    except ROSInterruptException:
        pass

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

