import threading
import time
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

        # 颜色检测灵敏度
        self.sensitivity = 15

        # 面积阈值（根据摄像头距离自行调整）
        self.area_threshold = 1500

        # 初始化标志
        self.green_detected = False

        # CvBridge 与图像订阅
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10
        )

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 设置绿色的 HSV 范围
        green_lower = np.array([60 - self.sensitivity, 100, 100])
        green_upper = np.array([60 + self.sensitivity, 255, 255])

        # 创建绿色掩膜
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        # 查找绿色轮廓
        contours, _ = cv2.findContours(green_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > self.area_threshold:
                # 计算中心点
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # 画圆圈
                    (x, y), radius = cv2.minEnclosingCircle(c)
                    center = (int(x), int(y))
                    cv2.circle(image, center, int(radius), (0, 255, 0), 2)

                    if not self.green_detected:
                        self.green_detected = True
                        print(f"[INFO] 检测到绿色物体，位置: ({cx}, {cy})，面积: {int(area)}")
                else:
                    self.green_detected = False
            else:
                self.green_detected = False
        else:
            self.green_detected = False

        # 显示图像
        cv2.imshow("Camera Feed", image)
        cv2.imshow("Green Mask", green_mask)
        cv2.resizeWindow("Camera Feed", 320, 240)
        cv2.waitKey(3)

def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()

    rclpy.init(args=None)
    node = ColourIdentifier()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            time.sleep(0.1)
    except ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

