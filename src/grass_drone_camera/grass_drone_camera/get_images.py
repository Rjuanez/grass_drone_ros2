import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from datetime import datetime

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.subscription = self.create_subscription(Image, 'sky_cam', self.listener_callback, 10)
        self.br = CvBridge()
        self.last_capture_time = self.get_clock().now()
        self.capture_interval = rclpy.duration.Duration(seconds=5)  # Canvia això per ajustar l'interval

        self.folder = "/home/ruben/Documents/grass_drone_ros2/src/grass_drone_camera/grass_drone_camera/Calibration_Photos2"
        os.makedirs(self.folder, exist_ok=True)

    def listener_callback(self, data):
        current_time = self.get_clock().now()
        if (current_time - self.last_capture_time) < self.capture_interval:
            return

        img = self.br.imgmsg_to_cv2(data, desired_encoding="rgb8")
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.resize(img, (900, 580))

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.folder, f"capture_{timestamp}.png")
        cv2.imwrite(filename, img)
        self.get_logger().info(f"Imatge guardada: {filename}")

        self.last_capture_time = current_time


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
