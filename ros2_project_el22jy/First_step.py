import threading
import sys, time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal

class ColourIdentifier(Node):
    def __init__(self):
        super().__init__('colour_identifier')

        # 初始化 CvBridge
        self.bridge = CvBridge()

        # 订阅摄像头图像话题
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # 根据实际情况调整话题
            self.callback,
            10
        )

    def callback(self, data):
        try:
            # 转换图像数据为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # 显示图像
            cv2.imshow("Camera Feed", cv_image)
            cv2.resizeWindow("Camera Feed", 320, 240)
            cv2.waitKey(3)

        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败: {e}")

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

