#!/usr/bin/env python3
"""只读订阅；每次调用 ~/capture 保存一组停稳的 AprilTag 手眼数据。"""
import json
from collections import deque
from pathlib import Path

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, JointState
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from luna_handeye_calibration.apriltag import AprilTagDetector
from luna_handeye_calibration.intrinsics import camera_info_from_yaml, validate_camera_info
from luna_handeye_calibration.kinematics import Kinematics
from luna_handeye_calibration.sampling import match_stationary
from luna_handeye_calibration.se3 import atomic_yaml, inverse
from scipy.spatial.transform import Rotation


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Collector(Node):
    def __init__(self):
        super().__init__('luna_apriltag_collector')
        defaults = {
            'image_topic': '/camera/camera/color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'joint_topic': '/tj/joint_states', 'output': '', 'allow_overwrite': True,
            'intrinsics_file': '',
            'timestamp_source': 'image_header',
            'model_path': '', 'base_frame': 'base_link', 'mount_frame': 'Head_J2_Link',
            'joint_aliases': '{"Head_J2_Joint": "Head_J2_joint"}',
            'tag_id': 0, 'tag_size': 0.08, 'max_reprojection_px': 1.5,
            'ambiguity_ratio': 1.5, 'sync_tolerance': 0.05, 'max_age': 0.5,
            'stationary_window': 0.3, 'stationary_motion': 0.003,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if self.p['timestamp_source'] not in ('image_header', 'ros_host_arrival'):
            raise ValueError('timestamp_source 只能为 image_header 或 ros_host_arrival')
        for key in ['sync_tolerance', 'max_age', 'stationary_window', 'stationary_motion']:
            if not np.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(f'{key} 必须为正有限值')
        if not self.p['output']:
            raise ValueError('必须指定 output 文件，每次会话使用新路径')
        self.output = Path(self.p['output']).expanduser().resolve()
        if self.output.exists() and not self.p['allow_overwrite']:
            raise ValueError(f'拒绝覆盖已有数据: {self.output}')
        path = self.p['model_path'] or str(Path(get_package_share_directory('marvin_description')) /
                                          'mjcf/Gento_Luna/Gento_Luna.xml')
        self.fk = Kinematics(path, self.p['base_frame'], self.p['mount_frame'],
                             json.loads(self.p['joint_aliases']))
        self.detector = AprilTagDetector(self.p['tag_id'], self.p['tag_size'],
                                        self.p['max_reprojection_px'], self.p['ambiguity_ratio'])
        self.bridge = CvBridge()
        self.image = self.info = None
        self.file_intrinsics = None
        if self.p['intrinsics_file']:
            with open(Path(self.p['intrinsics_file']).expanduser(), encoding='utf-8') as stream:
                self.file_intrinsics = yaml.safe_load(stream)
        self.history = deque(maxlen=4000)
        self.samples = []
        self.metadata = None
        for msg_type, topic, callback in [
            (Image, self.p['image_topic'], self.image_cb),
            (CameraInfo, self.p['camera_info_topic'], self.info_cb),
            (JointState, self.p['joint_topic'], self.joint_cb),
        ]:
            self.create_subscription(msg_type, topic, callback, qos_profile_sensor_data)
        self.create_service(Trigger, '~/capture', self.capture)
        self.get_logger().info(f'只读采集已启动；停稳后调用 ~/capture；输出 {self.output}')

    def image_cb(self, msg):
        self.image = msg

    def info_cb(self, msg):
        self.info = msg

    def joint_cb(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= 0 or len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        if self.history and stamp < self.history[-1][0]:
            self.history.clear()
        self.history.append((stamp, dict(zip(msg.name, msg.position))))

    def capture(self, request, response):
        del request
        try:
            if self.image is None or (self.info is None and self.file_intrinsics is None):
                raise ValueError('未收到原始图像或 CameraInfo')
            image, info = self.image, self.info
            if self.file_intrinsics is not None:
                # 腕部自定义驱动只发图像时，可显式提供对应设备/分辨率的标准内参 YAML。
                info = camera_info_from_yaml(self.file_intrinsics)
            stamp = stamp_seconds(image.header.stamp)
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if stamp <= 0 or age < 0 or age > self.p['max_age']:
                raise ValueError('图像时间戳过旧/无效；请确认各设备时钟同步')
            K, D = validate_camera_info(image, info)
            joints, delta = match_stationary(
                self.history, stamp, self.fk.positions, self.p['sync_tolerance'],
                self.p['stationary_window'], self.p['stationary_motion'])
            mount = self.fk.pose(joints)
            for previous in self.samples:
                relative = inverse(np.asarray(previous['mount_pose'])) @ mount
                if (np.linalg.norm(relative[:3, 3]) < 0.005 and
                        Rotation.from_matrix(relative[:3, :3]).magnitude() < np.deg2rad(3)):
                    raise ValueError('与已有姿态过于接近，请改变姿态后再采样')
            gray = self.bridge.imgmsg_to_cv2(image, desired_encoding='mono8')
            camera_tag, error, pixels = self.detector.detect(
                gray, K, D, info.distortion_model)
            metadata = {
                'schema_version': 1, 'convention': 'T_A_B maps B coordinates into A',
                'timestamp_source': self.p['timestamp_source'],
                'base_frame': self.fk.base, 'mount_frame': self.fk.mount,
                'camera_frame': image.header.frame_id, 'model_path': self.fk.path,
                'model_sha256': self.fk.sha256, 'joint_aliases': self.fk.aliases,
                'tag_family': '36h11', 'tag_id': self.p['tag_id'], 'tag_size_m': self.p['tag_size'],
                'camera_info': {'K': K.tolist(), 'D': D.tolist(), 'width': info.width,
                                'height': info.height, 'distortion_model': info.distortion_model},
            }
            if self.metadata is not None and metadata != self.metadata:
                raise ValueError('内参/坐标系/标定板参数发生变化，请开启新会话')
            sample = {'timestamp': stamp, 'joint_time_error_s': float(delta), 'joints': joints,
                      'mount_pose': mount.tolist(), 'camera_tag': camera_tag.tolist(),
                      'corners_px': pixels.tolist(), 'reprojection_px': error}
            payload = dict(metadata, samples=self.samples + [sample])
            atomic_yaml(self.output, payload)
            self.samples.append(sample)
            self.metadata = metadata
            response.success = True
            response.message = f'已保存第 {len(self.samples)} 组，重投影 RMS={error:.3f}px'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = None
    try:
        node = Collector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
