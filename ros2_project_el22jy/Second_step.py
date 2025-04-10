import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal

class ColourIdentifier(Node):
    def __init__(self):
        super().__init__('colour_identifier')

        # 设置颜色识别灵敏度
        self.sensitivity = 15

        # 初始化 CvBridge 和图像订阅器
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10
        )

    def callback(self, data):
        try:
            # 将 ROS 图像转换为 OpenCV 格式
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        # 转换为 HSV 色彩空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 定义绿色范围
        green_lower = np.array([60 - self.sensitivity, 100, 100])
        green_upper = np.array([60 + self.sensitivity, 255, 255])

        # 定义红色范围（两个区段）
        red_lower1 = np.array([0, 120, 70])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([170, 120, 70])
        red_upper2 = np.array([180, 255, 255])

        # 定义蓝色（我们要过滤掉它，但这里只用于演示）
        blue_lower = np.array([100, 100, 100])
        blue_upper = np.array([140, 255, 255])

        # 创建颜色掩膜
        green_mask = cv2.inRange(hsv, green_lower, green_upper_

