from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    parameters = PathJoinSubstitution(
        [FindPackageShare("plio"), "config", "point_lio.yaml"]
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
                ],
            )
        ]
    )
