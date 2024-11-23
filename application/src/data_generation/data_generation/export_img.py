#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import cv2
import os


bridge = CvBridge()

class Img_exporter(Node):

    def __init__(self):
        super().__init__('img_exporter')

        self.img = None
        self.i = 0
        self.export_dir = os.path.join(os.path.join(os.path.abspath(os.getcwd())), 'data', 'img')

        if not os.path.exists(self.export_dir):
            os.makedirs(self.export_dir)

        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            # '/rgbd_camera/image',
            self.camera_callback,
            10)
        self.subscription 

    def camera_callback(self, data):
        self.img = bridge.imgmsg_to_cv2(data, "bgr8")

    def timer_callback(self):
        cv2.imwrite(os.path.join(self.export_dir, '00'+str(self.i)+'.jpeg'), self.img)
        self.i += 1

def main(args=None):
    rclpy.init(args=None)
    img_exporter = Img_exporter()
    rclpy.spin(img_exporter)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

