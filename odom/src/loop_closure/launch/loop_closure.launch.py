from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg = FindPackageShare('loop_closure')
    params = PathJoinSubstitution([pkg, 'config', 'loop.yaml'])
    return LaunchDescription([
        Node(
            package='loop_closure', executable='loop_detector', name='loop_detector',
            output='screen', parameters=[params],
            remappings=[
                ('keyframe', '/dlio/odom_node/keyframe'),
                ('loop_constraint', '/loop_closure/constraint'),
            ]),
        Node(
            package='loop_closure', executable='pose_graph_backend',
            name='pose_graph_backend', output='screen', parameters=[params],
            remappings=[
                ('keyframe', '/dlio/odom_node/keyframe'),
                ('loop_constraint', '/loop_closure/constraint'),
                ('optimized_path', '/mapping/optimized_path'),
                ('save_map', '/mapping/save_map'),
            ]),
    ])
