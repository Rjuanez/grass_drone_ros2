# publicador_gstreamer.py
import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

class GStreamerPublisher:
    def __init__(self):
        self.sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback, queue_size=1)
        
        # Define GStreamer pipeline (ajústalo a tu IP/puerto si es vía UDP)
        gst_str = ('appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 speed-preset=ultrafast ! '
                   'rtph264pay ! udpsink host=127.0.0.1 port=5000')
        self.out = cv2.VideoWriter(gst_str, cv2.CAP_GSTREAMER, 0, 30, (640, 480), True)

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image_np is not None and image_np.shape[1] == 640 and image_np.shape[0] == 480:
            self.out.write(image_np)

if __name__ == "__main__":
    rospy.init_node("gstreamer_publisher_node")
    GStreamerPublisher()
    rospy.spin()
