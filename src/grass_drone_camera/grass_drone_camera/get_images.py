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

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        img = self.br.imgmsg_to_cv2(data, desired_encoding="rgb8")
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.resize(img,(900,580))

        folder = "/home/ruben/Documents/grass_drone_ros2/src/grass_drone_camera/grass_drone_camera/Calibration_Photos2"

        cv2.imshow("sky_cam",img)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(folder, f"capture_{timestamp}.png")
            cv2.imwrite(filename, img)
            print(f"Imatge guardada: {filename}")

        
        


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node
    rclpy.shutdown

if __name__ == 'main':
    main()