import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 1    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_config_2.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code},
    {"use_sim_time": False},
    {"require_ptp_sync": ParameterValue(
        LaunchConfiguration("require_ptp_sync"), value_type=bool)},
    {"ptp_utc_offset_seconds": ParameterValue(
        LaunchConfiguration("ptp_utc_offset_seconds"), value_type=int)},
    {"ptp_max_host_skew_seconds": ParameterValue(
        LaunchConfiguration("ptp_max_host_skew_seconds"), value_type=float)},
    {"ptp_stream_timeout_seconds": ParameterValue(
        LaunchConfiguration("ptp_stream_timeout_seconds"), value_type=float)}
]


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    return LaunchDescription([
        DeclareLaunchArgument("require_ptp_sync", default_value="true"),
        # PHC PTP on this robot uses TAI; software PTP with a UTC source uses 0.
        DeclareLaunchArgument("ptp_utc_offset_seconds", default_value="37"),
        DeclareLaunchArgument("ptp_max_host_skew_seconds", default_value="2.0"),
        DeclareLaunchArgument("ptp_stream_timeout_seconds", default_value="0.5"),
        livox_driver,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
