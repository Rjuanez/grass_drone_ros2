# publicador_gstreamer_ros2.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

class GStreamerPublisher(Node):
    def __init__(self):
        super().__init__('gstreamer_publisher_node')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        gst_str = (
            'appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=ultrafast ! '
            'rtph264pay ! udpsink host=127.0.0.1 port=5000'
        )
        self.out = cv2.VideoWriter(gst_str, cv2.CAP_GSTREAMER, 0, 30, (640, 480), True)

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image_np is not None and image_np.shape[1] == 640 and image_np.shape[0] == 480:
            self.out.write(image_np)

def main(args=None):
    rclpy.init(args=args)
    node = GStreamerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
