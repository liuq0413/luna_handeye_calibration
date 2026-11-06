#!/usr/bin/env python3
"""默认按实际关节反馈发布 base_link 到独立相机 frame 的动态 TF。"""
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from luna_handeye_calibration.kinematics import Kinematics
from luna_handeye_calibration.se3 import transform_from_dict, transform_to_dict


class CameraTF(Node):
    def __init__(self):
        super().__init__('luna_camera_tf')
        for key, value in {'calibration_file': '', 'joint_topic': '/tj/joint_states',
                           'model_path': '', 'mode': 'dynamic', 'max_age': 0.5}.items():
            self.declare_parameter(key, value)
        with open(Path(self.get_parameter('calibration_file').value).expanduser(), encoding='utf-8') as stream:
            self.config = yaml.safe_load(stream)
        c = self.config
        if c.get('schema_version') != 1 or c.get('validation_passed') is not True:
            raise ValueError('只允许发布已通过验证的当前格式标定文件')
        if c['child'] in (c['camera_frame'], c['base_frame'], c['mount_frame']):
            raise ValueError('必须使用独立 child frame，不能覆盖已有 TF')
        self.X = transform_from_dict(c['T_mount_camera'])
        self.max_age = self.get_parameter('max_age').value
        if self.max_age <= 0:
            raise ValueError('max_age 必须为正')
        mode = self.get_parameter('mode').value
        if mode == 'static_mount':
            # 仅在安装 link 已有正确动态 TF 时使用，绝不能固定 base_link 到移动相机。
            self.broadcaster = StaticTransformBroadcaster(self)
            self.publish(c['mount_frame'], self.X, self.get_clock().now().to_msg())
        elif mode == 'dynamic':
            path = self.get_parameter('model_path').value or c['model_path']
            self.fk = Kinematics(path, c['base_frame'], c['mount_frame'], c['joint_aliases'])
            if self.fk.sha256 != c['model_sha256']:
                raise ValueError('模型与采集时不同；请核对模型并重新验证/标定')
            self.broadcaster = TransformBroadcaster(self)
            self.create_subscription(JointState, self.get_parameter('joint_topic').value,
                                     self.joints, qos_profile_sensor_data)
        else:
            raise ValueError('mode 只能为 dynamic 或 static_mount')

    def joints(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if stamp <= 0 or not 0 <= age <= self.max_age:
            return
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        try:
            T = self.fk.pose(dict(zip(msg.name, msg.position))) @ self.X
            # 保留关节测量时间，不能给旧状态打上新的当前时间。
            self.publish(self.config['base_frame'], T, msg.header.stamp)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5)

    def publish(self, parent, T, stamp):
        value = transform_to_dict(T)
        msg = TransformStamped()
        msg.header.stamp, msg.header.frame_id = stamp, parent
        msg.child_frame_id = self.config['child']
        t, q = msg.transform.translation, msg.transform.rotation
        t.x, t.y, t.z = value['translation']
        q.x, q.y, q.z, q.w = value['quaternion']
        self.broadcaster.sendTransform(msg)


def main():
    rclpy.init()
    node = None
    try:
        node = CameraTF()
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
