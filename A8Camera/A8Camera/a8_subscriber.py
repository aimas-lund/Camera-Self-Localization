import numpy as np
import cv2 as cv

import rclpy
import sys
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped

sys.path.insert(1, "//home/mbzirc/ros2_ws/src/A8Camera/marker_localization_main/src")
sys.path.insert(1, "/home/mbzirc/px4_ros2/src") 


from localization.localization import localization
from localization.marker_detection.marker_detection import marker_detection

from px4_msgs.msg import VehicleAirData, VehicleOdometry, VehicleLocalPosition

PERIOD = 0.001
"""
    |736.9205       0       643.7345|
K = |   0       738.6126    372.1893|
    |   0           0           1   |
"""
IMAGE_FX = 736.9205
IMAGE_FY = 738.6126
MARKER_SIZE = 0.28*1

class CameraSubscriber(Node):

    def __init__(self):

        super().__init__('camera_subscriber')
        
        qos = QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)


        self.subscription_image = self.create_subscription(Image, 'A8Camera/camera', self.listener_callback_image, 100)
        self.subscription_image
        
        self.subscription_gimbal = self.create_subscription(Vector3Stamped, 'A8Camera/gimbal', self.listener_callback_gimbal, 100)
        self.subscription_gimbal
        
        self.subscription_vehicle_air_data = self.create_subscription(VehicleAirData, '/fmu/out/vehicle_air_data', self.listener_callback_vehicle_air_data, qos)
        self.subscription_vehicle_air_data

        self.subscription_vehicle_odometry = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.listener_callback_vehicle_odometry, qos)
        self.subscription_vehicle_odometry

        self.subscription_vehicle_local_position = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.listener_callback_vehicle_local_position, qos)
        self.subscription_vehicle_local_position

        self.get_im = False
        self.get_gimbal = False
        self.get_air_data = False
        self.get_odometry = False
        self.get_position = False
        self.marker_created = False

    def process_data(self):
        print(f'GET_IM: {self.get_im}\t GET_GIMBAL:{self.get_gimbal}\t GET_AIR_DATA: {self.get_air_data}\t GET_ODOMETRY: {self.get_odometry}\t GET_POSITION: {self.get_position}')
        
        if self.get_im and self.get_gimbal and self.get_air_data and self.get_odometry and self.get_position:
            
            if self.marker_created == False:
                self.marker_localization = localization(image_fx=IMAGE_FX, image_fy=IMAGE_FY, image_height=self.height, image_width=self.width, marker_size=MARKER_SIZE, detection_class=marker_detection)
                self.marker_created = True
                self.x = 0
                self.y = 0
                self.estimate_x = 0
                self.estimate_y = 0


            self.old_x = self.x
            self.old_y = self.y
            self.x, self.y = self.marker_localization.update(self.frame, self.t, self.estimate_x, self.estimate_y, 10, 10, 0, 0, 0, self.local_z)       

            if self.x is None:
                self.x = self.old_x
                self.y = self.old_y

            if self.x is not None:
                self.estimate_x = self.x
                self.estimate_y = self.y
                
            print(f'Estimate_x: {self.x}, Estimate_y: {self.y}')

    def listener_callback_image(self, msg):
        self.t = time.time()
        self.height = msg.height
        self.width = msg.width
        channel = msg.step//msg.width
        self.frame = np.reshape(msg.data, (self.height, self.width, channel))
        self.get_logger().info("Image Received")
        self.get_im = True
        self.process_data()

    def listener_callback_gimbal(self, msg):
        self.gimbal_yaw = msg.vector.x
        self.gimbal_pitch = msg.vector.y
        self.gimbal_roll = msg.vector.z
        self.get_logger().info("Gimbal angle received")
        self.get_gimbal = True
        self.process_data()
        # print(f'Yaw={self.yaw}\t Pitch={self.pitch}\t Roll={self.roll}')

    def listener_callback_vehicle_air_data(self, msg):
        self.altitude_press = msg.baro_alt_meter
        self.pressuer = msg.baro_pressure_pa
        self.get_logger().info("Air data received")
        self.get_air_data = True
        self.process_data()

    def listener_callback_vehicle_local_position(self, msg):
        self.local_x = msg.x
        self.local_y = msg.y
        self.local_z = msg.z

        self.local_vx = msg.vx
        self.local_vy = msg.vy
        self.local_vz = msg.vz

        self.local_ax = msg.ax
        self.local_ay = msg.ay
        self.local_az = msg.az
        self.get_logger().info("Local position received")
        self.get_position = True
        self.process_data()

    def listener_callback_vehicle_odometry(self, msg):
        self.odometry_position = msg.position
        self.quaternion_rotation = msg.q
        self.odometry_velocity = msg.velocity
        self.get_logger().info("Odometry received")
        self.get_odometry = True
        self.process_data()



def main(args=None):
    

    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()

    rclpy.spin(camera_subscriber)

    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main() 