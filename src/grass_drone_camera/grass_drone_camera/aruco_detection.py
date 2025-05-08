import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.subscription = self.create_subscription(Image, 'sky_cam', self.listener_callback, 10)
        self.publisher = self.create_publisher(Point,"/coordenades_dron_aruco",1000)
        self.br = CvBridge()

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        img = self.br.imgmsg_to_cv2(data, desired_encoding="rgb8")
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        p = Point()
        p.x = 0.0
        p.y = 0.0
        p.z = 0.0

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
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            for i in range(len(ids)):
                self.get_logger().info("detectando")
                #if ids[i][0] == ARUCO_ID:
                obj_points = np.array([
                    [-MARKERS_SIZE/2,  MARKERS_SIZE/2, 0],
                    [ MARKERS_SIZE/2,  MARKERS_SIZE/2, 0],
                    [ MARKERS_SIZE/2, -MARKERS_SIZE/2, 0],
                    [-MARKERS_SIZE/2, -MARKERS_SIZE/2, 0]
                ], dtype=np.float32)

                ret, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], camera_matrix, dist_coeffs)

                if ret:
                    cv2.drawFrameAxes(img, camera_matrix, dist_coeffs, rvec, tvec, MARKERS_SIZE)

                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    rpy_angles = cv2.RQDecomp3x3(rotation_matrix)[0]

                    x, y, z = tvec.flatten()
                    roll, pitch, yaw = rpy_angles

                    info = f"X: {x:.2f}m, Y: {y:.2f}m, Z: {z:.2f}m"
                    inclination = f"Roll: {roll:.1f}°, Pitch: {pitch:.1f}°, Yaw: {yaw:.1f}°"

                    cv2.putText(img, info, (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(img, inclination, (50,80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    
                    p.x = x
                    p.y = y
                    p.z = z

        else:
            self.get_logger().info("NO DETECTANT")
            

        
        self.publisher.publish(p)
        self.get_logger().info(f'Coordenades: x = {p.x}, y = {p.y}, z = {p.z}')

        cv2.imshow("sky_cam",img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node
    rclpy.shutdown

if __name__ == 'main':
    main()