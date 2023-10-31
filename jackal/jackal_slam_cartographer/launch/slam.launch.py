import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_path = '/home/jackal/ROS2_nodes/jackal/jackal_slam_cartographer/config/' # TODO get relative path
    urdf_file_name = 'ouster.urdf.xml'
    urdf = os.path.join(config_path, urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description' : robot_desc}],
            arguments=[urdf]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            remappings=[
                ('scan', '/ouster/scan'),
                ('odom', '/odom'),
                ('imu', '/ouster/imu')
            ],
            arguments=[
                '-configuration_directory', config_path,
                '-configuration_basename', 'jackal.lua',
            ],
        ),
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.05'],
        ),
    ])
