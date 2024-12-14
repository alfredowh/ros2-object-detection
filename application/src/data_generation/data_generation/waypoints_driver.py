import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion
from rclpy.task import Future

class WaypointDriver(Node):
    def __init__(self):
        super().__init__('waypoint_driver')

        self.declare_parameter('x_pose_init', 0.0)
        self.declare_parameter('y_pose_init', 0.0)

        self.driving_done_publisher = self.create_publisher(Bool, '/driving_done', 10)
        
        # Publisher for velocity commands
        self.controll_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Loop rate for driving controll 
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Data export timer
        self.export_timer = self.create_timer(5, self.export_timer_callback)

        # Export handling
        self.export = Bool()
        self.export.data = False
        self.export_done = True
        self.export_trigger_publisher = self.create_publisher(Bool, '/export_trigger', 10)
        self.export_done_subcriber = self.create_subscription(Bool, '/export_done', self.export_done_callback, 10)

        # Initialize indicating task completion
        self.done_future = Future()
        
        # Subscriber for robot odometry (pose)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  
            self.pose_callback,
            10
        )
        
        # # Set waypoints (x, y)
        self.waypoints = [
            {'x': -6.5, 'y': 9.2},
            {'x': -3.7, 'y': 9.2},
            {'x': -3.7, 'y': -9.44},
            {'x': 1.77, 'y': -9.44},
            {'x': 1.77, 'y': 2.24},
            {'x': 6.22, 'y': 2.24}
        ] # for test hall

        self.current_waypoint_index = 0
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.velocity = .3 # m/s
        self.angular_gain = .7

    def export_timer_callback(self):
        self.export.data = True
        self.export_trigger_publisher.publish(self.export)
    
    def export_done_callback(self, msg):
        self.export_done = msg.data
        if self.export_done:
            self.export.data = False

    def pose_callback(self, msg):
        x_pose_init = self.get_parameter('x_pose_init').get_parameter_value().double_value
        y_pose_init = self.get_parameter('y_pose_init').get_parameter_value().double_value
        self.robot_pose['x'] = msg.pose.pose.position.x + x_pose_init
        self.robot_pose['y'] = msg.pose.pose.position.y + y_pose_init

        # DEBUG
        # self.get_logger().info(f"{x_pose_init}, {y_pose_init}")

        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.robot_pose['yaw'] = yaw

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.controll_publisher.publish(twist)

    def timer_callback(self):
        if self.export.data:
            self.get_logger().info('Exporting...')
            self.stop()
            return

        elif self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('Done!')
            self.stop()
            self.done_future.set_result(True) 

            driving_done = Bool()
            driving_done.data = True
            self.driving_done_publisher.publish(driving_done)
            return

        goal = self.waypoints[self.current_waypoint_index]
        dx = goal['x'] - self.robot_pose['x']
        dy = goal['y'] - self.robot_pose['y']
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        # DEBUG
        # self.get_logger().info(f"Goal: ({goal['x']}, {goal['y']}), Pose: ({self.robot_pose['x']}, {self.robot_pose['y']}), Distance: {distance}, Yaw: {self.robot_pose['yaw']}, Angle to Goal: {angle_to_goal}\n")

        twist = Twist()
        if distance > 0.2:  # Distance threshold to the waypoint
            twist.linear.x = self.velocity
            twist.angular.z = self.angular_gain * (angle_to_goal - self.robot_pose['yaw'])
        else:
            self.current_waypoint_index += 1
            self.get_logger().info(f'Waypoint ({str(goal["x"])}, {str(goal["y"])}) reached')

        self.controll_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointDriver()
    rclpy.spin_until_future_complete(node, node.done_future)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
