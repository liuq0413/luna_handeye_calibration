from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(
            Path(get_package_share_directory('luna_handeye_calibration')) / 'config/head_camera.yaml')),
        DeclareLaunchArgument('output', description='新采集文件路径，不能覆盖已有文件'),
        Node(
            package='luna_handeye_calibration',
            executable='collect_apriltag',
            name='luna_apriltag_collector',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {'output': LaunchConfiguration('output')}],
        ),
    ])
