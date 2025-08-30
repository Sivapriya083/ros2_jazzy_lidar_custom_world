#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
   
    tortoisebot_gazebo_dir = get_package_share_directory('tortoisebot_gazebo')
    tortoisebot_description_dir = get_package_share_directory('tortoisebot_description')

  
    world_file = os.path.join(tortoisebot_gazebo_dir, 'worlds', 'custom_obstacles.world')

  
    gazebo_launch = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-v', '4'],
        output='screen'
    )

   
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('tortoisebot_description'),
            'urdf',
            'tortoisebot.urdf.xacro'
        ])
    ])

  
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }],
        output='screen'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',  
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan', 
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

   
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'tortoisebot',
                    '-topic', '/robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1',
                ],
                output='screen'
            )
        ]
    )

   
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'tortoisebot/base_footprint/lidar_sensor'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        ros_gz_bridge,
        spawn_robot,
        static_transform,  
        
    ])