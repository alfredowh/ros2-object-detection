#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from rclpy.task import Future


import cv2
import os

bridge = CvBridge()

class Img_exporter(Node):

    def __init__(self):
        super().__init__('img_exporter')

        # Initialize indicating task completion
        self.done_future = Future()

        self.img = None
        self.i = 0
        self.export_dir = os.path.join(os.path.join(os.path.abspath(os.getcwd())), 'application', 'data', 'img')

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

        self.driving_done_subscription = self.create_subscription(
            Bool,
            '/driving_done',
            # '/rgbd_camera/image',
            self.driving_done_callback,
            10)
        self.driving_done_subscription 

    def camera_callback(self, data):
        self.img = bridge.imgmsg_to_cv2(data, "bgr8")

    def driving_done_callback(self, data):
        self.done_future.set_result(data.data) 

    def timer_callback(self):
        if self.img is not None:
            cv2.imwrite(os.path.join(self.export_dir, '00'+str(self.i)+'.jpeg'), self.img)
            self.i += 1

def main(args=None):
    rclpy.init(args=None)
    img_exporter = Img_exporter()
    # rclpy.spin(img_exporter)
    rclpy.spin_until_future_complete(img_exporter, img_exporter.done_future)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

