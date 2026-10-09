from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameters = PathJoinSubstitution(
        [FindPackageShare("plio"), "config", "point_lio.yaml"]
    )
    loop_parameters = PathJoinSubstitution(
        [FindPackageShare("loop_closure"), "config", "loop.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="fusion_ws",
                executable="fusion_pcl",
                name="fusion_pcl",
                output="screen",
            ),
            Node(
                package="loop_closure",
                executable="loop_detector",
                name="loop_detector",
                output="screen",
                parameters=[loop_parameters],
                remappings=[
                    ("keyframe", "/point_lio/keyframe"),
                    ("loop_constraint", "/loop_closure/constraint"),
                ],
            ),
            Node(
                package="loop_closure",
                executable="pose_graph_backend",
                name="pose_graph_backend",
                output="screen",
                parameters=[loop_parameters],
                remappings=[
                    ("keyframe", "/point_lio/keyframe"),
                    ("loop_constraint", "/loop_closure/constraint"),
                    ("optimized_path", "/mapping/optimized_path"),
                    ("save_map", "/mapping/save_map"),
                ],
            ),
            TimerAction(period=1.0, actions=[Node(
                package="plio",
                executable="point_lio",
                name="point_lio",
                output="screen",
                parameters=[parameters],
                remappings=[
                    ("pointcloud", "/gimbal/cloud_fused"),
                    ("imu", "/gimbal/imu_fused"),
                    ("odom", "/point_lio/odom"),
                    ("path", "/path"),
                    ("registered", "/cloud_registered"),
                    ("registered_body", "/cloud_registered_body"),
                    ("keyframe", "/point_lio/keyframe"),
                    ("kf_cloud", "/point_lio/keyframe_cloud"),
                ],
            )]),
        ]
    )
