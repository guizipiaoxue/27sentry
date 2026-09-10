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
                ('keyframes', '/dlio/odom_node/keyframes'),
                ('keyframe_cloud', '/dlio/odom_node/pointcloud/keyframe'),
                ('loop_constraint', '/loop_closure/constraint'),
            ])
    ])
