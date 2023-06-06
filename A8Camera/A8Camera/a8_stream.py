import numpy as np
import cv2 as cv

import rclpy
import pickle
import struct
import socket

from rclpy.node import Node
from sensor_msgs.msg import Image

PERIOD = 0.001
FPS = 5
TEORIC_FPS = 30


class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('streaming')
        
        # Initialize subscription to topic
        self.subscription_image = self.create_subscription(Image, 'A8Camera/camera', self.listener_callback_image, 100)
        self.subscription_image

        # Intialize socket
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host_name = socket.gethostname()
        # host_ip = '10.209.229.178'
        host_ip = '192.168.10.20' # paste your IP addres here
        port = 9999
        print(f'HOST IP: {host_ip}\t HOST NAME: {host_name}\t PORT: {port}')
        socket_address = (host_ip,port)
        server_socket.bind(socket_address)
        server_socket.listen(5)
        print(f"LISTENING AT: {socket_address}")

        self.client_socket, addr = server_socket.accept()
        print(f'GOT CONNECTION FROM: {addr}')

        self.skip_frame = TEORIC_FPS/FPS
        self.actual_frame = 0

    def listener_callback_image(self, msg):
        
        print()
        # Recive Image
        height = msg.height
        width = msg.width
        channel = msg.step//msg.width
        self.frame = np.reshape(msg.data, (height, width, channel))
        self.get_logger().info("Image Received")

        # Stream Image
        if self.client_socket:
            if self.actual_frame % self.skip_frame == 0:
                a = pickle.dumps(self.frame)
                message = struct.pack("Q", len(a))+a
                self.client_socket.sendall(message)        
                # cv.imshow('TRANSMITTING VIDEO', self.frame)
        
        self.actual_frame += 1
            


def main(args=None):
    

    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()

    rclpy.spin(camera_subscriber)

    camera_subscriber.client_socket.close()
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main() 


