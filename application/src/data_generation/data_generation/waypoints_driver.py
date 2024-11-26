import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from rclpy.task import Future

class WaypointDriver(Node):
    def __init__(self):
        super().__init__('waypoint_driver')
        
        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Loop rate for driving controll 
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialize indicating task completion
        self.done_future = Future()
        
        # Subscriber for robot odometry (pose)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  
            self.pose_callback,
            10
        )
        
        # Set waypoints (x, y)
        self.waypoints = [
            {'x': 5.0, 'y': 0.0},
            {'x': 5.0, 'y': 5.0},
            {'x': 0.0, 'y': 0.0}
        ]
        self.current_waypoint_index = 0
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.velocity = 0.5 # m/s
        self.angular_gain = 1.0

    def pose_callback(self, msg):
        self.robot_pose['x'] = msg.pose.pose.position.x
        self.robot_pose['y'] = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.robot_pose['yaw'] = yaw

    def timer_callback(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('Done!')
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)

            self.done_future.set_result(True) 
            return

        goal = self.waypoints[self.current_waypoint_index]
        dx = goal['x'] - self.robot_pose['x']
        dy = goal['y'] - self.robot_pose['y']
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        twist = Twist()
        if distance > 0.1:  # Distance threshold to the waypoint
            twist.linear.x = self.velocity
            twist.angular.z = self.angular_gain * (angle_to_goal - self.robot_pose['yaw'])
        else:
            self.current_waypoint_index += 1
            self.get_logger().info(f'Waypoint ({str(goal["x"])}, {str(goal["y"])}) reached')

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointDriver()
    rclpy.spin_until_future_complete(node, node.done_future)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
