import numpy as np
import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2
import ctypes
import struct
import os 

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import PointCloud2, PointField


class PclSubscriber(Node):

    def __init__(self):
        super().__init__('pcl_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/scan/points',
            self.listener_callback,
            10)
        self.subscription  

        self.i = 0
        self.export_dir = os.path.join(os.path.join(os.path.abspath(os.getcwd())), 'data', 'pcl')

        if not os.path.exists(self.export_dir):
            os.makedirs(self.export_dir)


    
    def listener_callback(self, msg):
        self.i += 1
        xyz = np.array([[0,0,0]])
        rgb = np.array([[0,0,0]])
        gen = pc2.read_points(msg, skip_nans=True)
        int_data = list(gen)

        for x in int_data:
            test = x[3] 
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # Get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)

            xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
            rgb = np.append(rgb,[[r,g,b]], axis = 0)

        out_pcd = o3d.geometry.PointCloud()    
        out_pcd.points = o3d.utility.Vector3dVector(xyz)
        out_pcd.colors = o3d.utility.Vector3dVector(rgb)
        print(f'ss: {type(out_pcd)}')
        o3d.io.write_point_cloud(os.path.join(self.export_dir, '00' + str(self.i) + '.pcd'), out_pcd)


def main(args=None):
    rclpy.init(args=args)
    pcl_subscriber = PclSubscriber()
    rclpy.spin_once(pcl_subscriber)
    pcl_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

