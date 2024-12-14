#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    launch_file_dir = os.path.join(
        get_package_share_directory("turtlebot_gazebo"), "launch"
    )
    # pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="-6.5")
    y_pose = LaunchConfiguration("y_pose", default="9.2")
    export_img = LaunchConfiguration("export_img", default='true')
    drive = LaunchConfiguration("drive", default='true')
    world_file = LaunchConfiguration(
        "world",
        default=os.path.join(
            get_package_share_directory("turtlebot_gazebo"), "worlds", "objects.world"
        ),
    )

    declare_world_file_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            get_package_share_directory("turtlebot_gazebo"), "worlds", "objects.world"
        ),
        description="Full path to the world file to load in Gazebo",
    )

    declare_export_img_cmd = DeclareLaunchArgument(
        "export_img",
        default_value='true',
        description="Export images from gazebo",
    )

    declare_drive_cmd = DeclareLaunchArgument(
        "drive",
        default_value='true',
        description="Drive the robot",
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(get_package_share_directory("turtlebot_gazebo"), "models"),
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -s -v4 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-g -v4 "}.items(),
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    waypoints_drive_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    waypoints_drive_cmd = Node(
        package='data_generation',
        executable='waypoints_driver',
        output='screen',
        emulate_tty=True,
        parameters=[
        {'x_pose_init': x_pose,
         'y_pose_init': y_pose}],
        condition=IfCondition(drive)
    )

    export_img_cmd = Node(
        package='data_generation',
        executable='export_img',
        condition=IfCondition(export_img))
    
    
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_drive_cmd)
    ld.add_action(declare_export_img_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(waypoints_drive_cmd)
    ld.add_action(export_img_cmd)

    return ld
