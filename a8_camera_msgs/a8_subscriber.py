import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, 'A8Camera/camera', self.listener_callback, 10)
        
        self.subscription

    def listener_callback(self, msg):
        height = msg.height
        width = msg.width
        channel = msg.step//msg.width
        frame = np.reshape(msg.data, (height, width, channel))
        self.get_logger().info("Image Received")

        cv.imshow('Frame',frame)

        if cv.waitKey(25) & 0xFF == ord('q'):
                cv.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()