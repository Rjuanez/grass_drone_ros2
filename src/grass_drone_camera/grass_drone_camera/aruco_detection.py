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
        self.subscription = self.create_subscription(Image, 'camera_down', self.listener_callback, 10)
        self.publisher_pos = self.create_publisher(Point,"/coordenades_dron_aruco",1000)
        self.publisher_ang = self.create_publisher(Vector3, "/angles_dron_aruco",1000)
        self.br = CvBridge()

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        img = self.br.imgmsg_to_cv2(data, desired_encoding="rgb8")
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        p = Point()
        p.x = 0.0
        p.y = 0.0
        p.z = 0.0

        angles = Vector3()
        angles.x = 0.0
        angles.y = 0.0
        angles.z = 0.0

        camera_matrix = np.array([[747.47830424 , 0, 450.98253964],
                                [0, 856.17983209, 289.21665834],
                                [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.array([0.00060266, 0.01014434, -0.0002952, 0.00050071, -0.12741058])

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        ARUCO_ID = 101
        MARKERS_SIZE = 0.15

        roi_active = False
        roi_margin = 50
        roi_coords = None
        frame_count = 0
        detect_every_n = 30

        #comença loop
        frame_count += 1

        img = cv2.resize(img, (980, 560))

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        search_area = gray
        
        if roi_active:
            x_min, y_min, x_max, y_max = roi_coords
            search_area = gray[y_min:y_max, x_min:x_max]
            corners, ids, _ = detector.detectMarkers(search_area)

            if ids is not None:
                for i in range(len(ids)):
                    self.get_logger().info("detectando")
                    if ids[i][0] == ARUCO_ID:
                        #ajustar coordenades
                        corners[i][0][:, 0] += x_min
                        corners[i][0][:, 1] += y_min

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

                            angles.x = roll
                            angles.y = pitch
                            angles.z = yaw

                            x_coords = corners[i][0][:, 0]
                            y_coords = corners[i][0][:, 1]
                            x_min = max(0, int(np.min(x_coords)) - roi_margin)
                            x_max = min(gray.shape[1], int(np.max(x_coords)) + roi_margin)
                            y_min = max(0, int(np.min(y_coords)) - roi_margin)
                            y_max = min(gray.shape[0], int(np.max(y_coords)) + roi_margin)

                            roi_coords = (x_min, y_min, x_max, y_max)
                            roi_active = True
                        break
            
            else:
                roi_active = False
        else:
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None:
                for i in range(len(ids)):
                    if ids[i][0] == ARUCO_ID:
                        obj_points = np.array([
                            [-MARKER_SIZE/2,  MARKER_SIZE/2, 0],
                            [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
                            [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
                            [-MARKER_SIZE/2, -MARKER_SIZE/2, 0]
                        ], dtype=np.float32)

                        ret, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], camera_matrix, dist_coeffs)

                        if ret:
                            cv2.drawFrameAxes(img, camera_matrix, dist_coeffs, rvec, tvec, MARKER_SIZE)

                            rotation_matrix, _ = cv2.Rodrigues(rvec)
                            rpy_angles = cv2.RQDecomp3x3(rotation_matrix)[0]

                            x, y, z = tvec.flatten()
                            roll, pitch, yaw = rpy_angles

                            info = f"X: {x:.2f}m, Y: {y:.2f}m, Z: {z:.2f}m"
                            inclination = f"Roll: {roll:.1f}°, Pitch: {pitch:.1f}°, Yaw: {yaw:.1f}°"

                            cv2.putText(img, info, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            cv2.putText(img, inclination, (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                            p.x = x
                            p.y = y
                            p.z = z

                            angles.x = roll
                            angles.y = pitch
                            angles.z = yaw

                            x_coords = corners[i][0][:, 0]
                            y_coords = corners[i][0][:, 1]
                            x_min = max(0, int(np.min(x_coords)) - roi_margin)
                            x_max = min(gray.shape[1], int(np.max(x_coords)) + roi_margin)
                            y_min = max(0, int(np.min(y_coords)) - roi_margin)
                            y_max = min(gray.shape[0], int(np.max(y_coords)) + roi_margin)

                            roi_coords = (x_min, y_min, x_max, y_max)
                            roi_active = True
                        break

        if frame_count % detect_every_n == 0:
            roi_active = False
        
        self.publisher_pos.publish(p)
        self.publisher_ang.publish(angles)
        self.get_logger().info(f'Coordenades: x = {p.x}, y = {p.y}, z = {p.z}')
        self.get_logger().info(f'Angles: x = {angles.x}, y = {angles.y}, z = {angles.z}')

        cv2.imshow("camera_down",img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node
    rclpy.shutdown

if __name__ == 'main':
    main()
