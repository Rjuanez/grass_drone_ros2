import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 10)
        self.publisher_pos = self.create_publisher(Point,"/coordenades_dron_aruco",1000)
        self.publisher_ang = self.create_publisher(Vector3, "/angles_dron_aruco",1000)
        self.br = CvBridge()

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        img = self.br.imgmsg_to_cv2(data, desired_encoding="rgb8")
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        camera_matrix = np.array([[747.47830424 , 0, 450.98253964],
                                [0, 856.17983209, 289.21665834],
                                [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.array([0.00060266, 0.01014434, -0.0002952, 0.00050071, -0.12741058])

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        ARUCO_ID = 101
        MARKERS_SIZE = 0.15



        img = cv2.resize(img, (980, 560))

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        search_area = gray
 
        corners, ids, _ = detector.detectMarkers(search_area)

        if ids is not None:
            for i in range(len(ids)):
                self.get_logger().info("detectando")
               



def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node
    rclpy.shutdown

if __name__ == 'main':
    main()
