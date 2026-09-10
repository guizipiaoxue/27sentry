from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('map_ws'), 'config', 'map.yaml'
    ])
    return LaunchDescription([
        Node(
            package='map_ws',
            executable='kdtree_map',
            name='kdtree_map',
            output='screen',
            parameters=[params],
            remappings=[
                ('keyframe', '/dlio/odom_node/pointcloud/keyframe'),
                ('save_map', '/dlio/save_kdtree_map'),
                ('clear_map', '/dlio/clear_kdtree_map'),
            ],
        ),
    ])
