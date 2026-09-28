import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration('device')
    receiver_config = os.path.join(
        get_package_share_directory('receiver'), 'config', 'receiver.yaml')
    sender_config = os.path.join(
        get_package_share_directory('sender'), 'config', 'sender.yaml')
    mpc_config = os.path.join(
        get_package_share_directory('test_control'), 'config', 'mpc.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/ttyACM0'),
        Node(package='receiver', executable='receiver_node', name='receiver',
             parameters=[receiver_config, {'device': device}], output='screen'),
        Node(package='sender', executable='sender_node', name='sender',
             parameters=[sender_config, {'device': device}], output='screen'),
        Node(package='test_control', executable='mpc_control', name='mpc_control',
             parameters=[mpc_config], output='screen'),
    ])
