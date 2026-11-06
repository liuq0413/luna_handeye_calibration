"""同时启动只读采集器和实时预览前端。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 通过 camera 统一选择三套预置参数；params_file/output 非空时仍可显式覆盖。
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera', default_value='left_hand',
            description='标定相机：head、left_hand 或 right_hand',
        ),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument(
            'output',
            default_value='',
            description='采集文件路径',
        ),
        DeclareLaunchArgument(
            'allow_overwrite', default_value='true',
            description='是否允许覆盖已有采样文件',
        ),
        DeclareLaunchArgument('tag_id', default_value='0'),
        DeclareLaunchArgument('tag_size', default_value='0.02'),
        DeclareLaunchArgument('timestamp_source', default_value='ros_host_arrival'),
        DeclareLaunchArgument('preview_fps', default_value='12.0'),
        DeclareLaunchArgument(
            'preview', default_value='true',
            description='是否在 launch 中启动图像预览；false 时可在独立 TTY 运行 capture_preview',
        ),
        OpaqueFunction(function=_make_nodes),
    ])


def _make_nodes(context):
    package_share = Path(get_package_share_directory('luna_handeye_calibration'))
    presets = {
        'head': ('head_camera.yaml', 'calibration_data/head_samples.yaml'),
        'left_hand': ('left_hand_camera.yaml', 'calibration_data/left_hand_samples.yaml'),
        'right_hand': ('right_hand_camera.yaml', 'calibration_data/right_hand_samples.yaml'),
    }
    camera = LaunchConfiguration('camera').perform(context).strip().lower()
    if camera not in presets:
        raise ValueError(
            f"camera 必须是 head、left_hand 或 right_hand，实际为 {camera!r}"
        )
    preset_file, preset_output = presets[camera]
    params_arg = LaunchConfiguration('params_file').perform(context).strip()
    output_arg = LaunchConfiguration('output').perform(context).strip()
    params_file = params_arg or str(package_share / 'config' / preset_file)
    output = output_arg or preset_output
    nodes = [
        Node(
            package='luna_handeye_calibration',
            executable='collect_apriltag',
            name='luna_apriltag_collector',
            output='screen',
            parameters=[params_file, {
                'output': output,
                'allow_overwrite': ParameterValue(
                    LaunchConfiguration('allow_overwrite'), value_type=bool),
                'tag_id': ParameterValue(LaunchConfiguration('tag_id'), value_type=int),
                'tag_size': ParameterValue(LaunchConfiguration('tag_size'), value_type=float),
                'timestamp_source': LaunchConfiguration('timestamp_source'),
            }],
        ),
    ]
    preview_enabled = LaunchConfiguration('preview').perform(context).strip().lower()
    if preview_enabled not in ('false', '0', 'off', 'no'):
        nodes.append(Node(
            package='luna_handeye_calibration',
            executable='capture_preview',
            name='luna_capture_preview',
            output='screen',
            parameters=[{
                'preview_fps': ParameterValue(
                    LaunchConfiguration('preview_fps'), value_type=float),
            }],
        ))
    return nodes
