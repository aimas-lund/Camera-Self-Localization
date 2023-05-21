import os 
import cv2
import rclpy
import sys

# sys.path.insert(1, "/home/mbzirc/repositories/Camera-Self-Localization-MBZIRC/pc control/siyi_control")
sys.path.insert(1, "/home/mbzirc/ros2_ws/src/A8Camera/siyi_control")

from siyi_sdk import SIYISDK
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped

TOPIC = 'A8Camera/gimbal'
QUEUE_SIZE = 100
PERIOD = 0.03

class GimbalPublisher(Node):
    
    def __init__(self, cam, topic=TOPIC, queue=QUEUE_SIZE, period=PERIOD):
        super().__init__('gimbal_publisher')
        self.publisher = self.create_publisher(Vector3Stamped, topic, queue)
        timer_period = period
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cam = cam
        self.i = 0

    def timer_callback(self):

        msg = Vector3Stamped()
        
        # Yaw, pitch, roll
        x, y, z = self.cam.getAttitude()
        msg.vector.x = x
        msg.vector.y = y
        msg.vector.z = z
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.header.frame_id = 'A8_Gimbal_Camera'

        self.publisher.publish(msg)
        self.get_logger().info('%d Data Published' % self.i)

        self.i += 1


def main(args=None):

    cam = SIYISDK(server_ip="192.168.144.25", port=37260)
    cam.connect()
    rclpy.init(args=args)
    gimbal_publisher = GimbalPublisher(cam)
    
    rclpy.spin(gimbal_publisher)

    gimbal_publisher.destroy_node()
    rclpy.shutdown()
    cam.disconnect()
    
    return None

if __name__ == '__main__':
    main() 
