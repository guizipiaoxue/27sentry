from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    odom_yaml = PathJoinSubstitution([FindPackageShare('odom_ws'), 'config', 'odom.yaml'])
    loop_yaml = PathJoinSubstitution([FindPackageShare('loop_closure'), 'config', 'loop.yaml'])
    return LaunchDescription([
        Node(
            package='loop_closure', executable='loop_detector', name='loop_detector', output='screen',
            parameters=[loop_yaml], remappings=[
                ('keyframe', '/dlio/odom_node/keyframe'),
                ('loop_constraint', '/loop_closure/constraint')]),
        Node(
            package='loop_closure', executable='pose_graph_backend',
            name='pose_graph_backend', output='screen', parameters=[loop_yaml],
            remappings=[
                ('keyframe', '/dlio/odom_node/keyframe'),
                ('loop_constraint', '/loop_closure/constraint'),
                ('optimized_path', '/mapping/optimized_path'),
                ('map', '/mapping/map'),
                ('save_map', '/mapping/save_map')]),
        TimerAction(period=1.0, actions=[
            Node(
                package='odom_ws', executable='odom', name='odom',
                output='screen', parameters=[odom_yaml], remappings=[
                    ('pointcloud', '/gimbal/cloud_fused'),
                    ('imu', '/gimbal/imu_fused'), ('path', '/path'),
                    ('pose', '/dlio/odom_node/pose'),
                    ('odom', '/dlio/odom_node/odom'),
                    ('kf_pose', '/dlio/odom_node/keyframes'),
                    ('kf_cloud', '/dlio/odom_node/pointcloud/keyframe'),
                    ('keyframe', '/dlio/odom_node/keyframe'),
                    ('deskewed', '/fusion_pcl')])
        ]),
    ])
