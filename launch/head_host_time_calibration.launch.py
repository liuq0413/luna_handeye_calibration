"""显式启用头部主机接收时间流，并启动只读标定采集器。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = str(Path(get_package_share_directory('luna_handeye_calibration')) /
                 'config/head_camera.yaml')
    image_topic = '/camera/camera/color/image_raw_host_time'
    info_topic = '/camera/camera/color/camera_info_host_time'
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('output', default_value='calibration_data/head_samples.yaml',
                             description='工作区根目录下的采样输出路径'),
        DeclareLaunchArgument('allow_overwrite', default_value='true',
                             description='是否允许覆盖已有采样文件'),
        DeclareLaunchArgument('tag_id', default_value='0'),
        DeclareLaunchArgument('tag_size', default_value='0.02', description='黑色方框实测边长，米'),
        Node(package='luna_handeye_calibration', executable='head_host_time',
             name='luna_head_host_time', output='screen'),
        Node(package='luna_handeye_calibration', executable='collect_apriltag',
             name='luna_apriltag_collector', output='screen', parameters=[
                 LaunchConfiguration('params_file'), {
                     'output': LaunchConfiguration('output'),
                     'allow_overwrite': ParameterValue(
                         LaunchConfiguration('allow_overwrite'), value_type=bool),
                     'image_topic': image_topic,
                     'camera_info_topic': info_topic,
                     'timestamp_source': 'ros_host_arrival',
                     'tag_id': ParameterValue(LaunchConfiguration('tag_id'), value_type=int),
                     'tag_size': ParameterValue(LaunchConfiguration('tag_size'), value_type=float),
                 }]),
    ])
