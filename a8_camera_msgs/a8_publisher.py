import os
import cv2
import json
import rclpy
import time

import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge

TOPIC = 'A8Camera/camera'
QUEUE_SIZE = 100
PERIOD = 0.001

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"

class CameraPublisher(Node):

    def __init__(self, capture, topic = TOPIC, queue=QUEUE_SIZE, period=PERIOD ):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, topic, queue)
        timer_period = period
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.capture = capture
        self.i = 0
        self.bridge = CvBridge()

    def timer_callback(self):
        
        ret, frame = self.capture.read()
        
        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        msg.header.frame_id = 'A8_Camera'
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.height = np.shape(frame)[0]
        msg.width = np.shape(frame)[1]
        msg.step = np.shape(frame)[2]*np.shape(frame)[1]
        
        """
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = 'A8_Camera'
        msg.height = np.shape(frame)[0]
        msg.width = np.shape(frame)[1]
        msg.encoding =  "H.264"
        msg.is_bigendian = True
        msg.step = np.shape(frame)[2]*np.shape(frame)[1]
        msg.data = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        """

        self.publisher.publish(msg)
        self.get_logger().info('%d Images Published' % self.i)

        self.i += 1

        return None

def main(args=None):


    capture = cv2.VideoCapture('rtsp://192.168.144.25:8554/main.264')

    rclpy.init(args=args)
    camera_publisher = CameraPublisher(capture)

    rclpy.spin(camera_publisher)

    camera_publisher.destroy_node()
    rclpy.shutdown()
    capture.release()

    return None

if __name__ == '__main__':
    main()

