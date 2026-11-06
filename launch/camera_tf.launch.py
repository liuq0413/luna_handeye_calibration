from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file'),
        DeclareLaunchArgument('joint_topic', default_value='/tj/joint_states'),
        DeclareLaunchArgument('mode', default_value='dynamic'),
        Node(package='luna_handeye_calibration', executable='camera_tf_broadcaster',
             parameters=[{key: LaunchConfiguration(key) for key in
                          ['calibration_file', 'joint_topic', 'mode']}], output='screen'),
    ])
