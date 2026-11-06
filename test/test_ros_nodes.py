"""隔离 ROS domain 下的消息/采样/TF 冒烟测试，不连接或控制真实机器人。"""
from pathlib import Path
import time

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import Image, CameraInfo, JointState
from std_srvs.srv import Trigger

from luna_handeye_calibration.collect_apriltag import Collector
from luna_handeye_calibration.camera_tf_broadcaster import CameraTF
from luna_handeye_calibration.se3 import atomic_yaml, transform_to_dict
from luna_handeye_calibration.kinematics import Kinematics
from luna_handeye_calibration.head_host_time import HeadHostTime, stamp_ns


MODEL = Path(__file__).resolve().parents[2] / 'marvin_description/mjcf/Gento_Luna/Gento_Luna.xml'


def stamp(msg, seconds):
    msg.header.stamp.sec = int(seconds)
    msg.header.stamp.nanosec = int((seconds - int(seconds))*1e9)


@pytest.fixture
def ros(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '173')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.mark.parametrize('use_config_output', [False, True])
def test_collector_parameters_capture_duplicates_and_stale(tmp_path, ros, monkeypatch, use_config_output):
    output = tmp_path / ('calibration_data/head_samples.yaml' if use_config_output else 'samples.yaml')
    args = ['--ros-args', '-p', f'model_path:={MODEL}',
            '-p', 'image_topic:=/test/image', '-p', 'tag_id:=7']
    if use_config_output:
        # 模拟从工作区根目录启动；真实样本只写入 pytest 临时目录。
        config = Path(__file__).resolve().parents[1] / 'config/head_camera.yaml'
        args = ['--ros-args', '--params-file', str(config)] + args[1:]
        monkeypatch.chdir(tmp_path)
    else:
        args += ['-p', f'output:={output}']
    rclpy.init(args=args)
    node = Collector()
    try:
        assert node.output == output
        assert node.p['image_topic'] == '/test/image'
        assert node.detector.tag_id == 7
        image, info = Image(), CameraInfo()
        image.width = info.width = 640
        image.height = info.height = 480
        image.header.frame_id = info.header.frame_id = 'test_optical'
        image.encoding, image.step = 'mono8', 640
        image.data = bytes(640*480)
        info.k = [600., 0., 320., 0., 600., 240., 0., 0., 1.]
        info.d, info.distortion_model = [0.]*5, 'plumb_bob'
        t = time.time() - .1
        stamp(image, t)
        joint_names = ['Leg_J1_Joint', 'Leg_J2_Joint', 'Leg_J3_Joint',
                       'Leg_J4_Joint', 'Leg_J5_Joint', 'Leg_J6_Joint',
                       'Head_J1_joint', 'Head_J2_joint']
        for instant in np.arange(t-.4, t+.05, .01):
            msg = JointState()
            stamp(msg, instant)
            msg.name = joint_names
            msg.position = [0.] * len(joint_names)
            node.joint_cb(msg)
        node.image_cb(image)
        node.info_cb(info)
        node.detector.detect = lambda *_: (np.eye(4), .1, np.zeros((4, 2)))
        response = node.capture(Trigger.Request(), Trigger.Response())
        assert response.success, response.message
        assert output.exists()
        response = node.capture(Trigger.Request(), Trigger.Response())
        assert not response.success and '接近' in response.message
        stamp(image, t-10)
        response = node.capture(Trigger.Request(), Trigger.Response())
        assert not response.success and '时间戳' in response.message
        assert len(node.samples) == 1
    finally:
        node.destroy_node()


def test_dynamic_tf_uses_feedback_timestamp_and_rejects_old_data(tmp_path, ros):
    fk = Kinematics(MODEL, 'base_link', 'Head_J2_Link', {'Head_J2_Joint': 'Head_J2_joint'})
    path = tmp_path / 'calibration.yaml'
    atomic_yaml(path, {'schema_version': 1, 'validation_passed': True,
                      'base_frame': 'base_link', 'mount_frame': 'Head_J2_Link',
                      'camera_frame': 'real_optical', 'child': 'head_calibrated',
                      'model_path': str(MODEL), 'model_sha256': fk.sha256,
                      'joint_aliases': fk.aliases, 'T_mount_camera': transform_to_dict(np.eye(4))})
    rclpy.init(args=['--ros-args', '-p', f'calibration_file:={path}'])
    node = CameraTF()
    published = []
    node.broadcaster.sendTransform = published.append
    try:
        msg = JointState()
        msg.name = ['Leg_J1_Joint', 'Leg_J2_Joint', 'Leg_J3_Joint',
                    'Leg_J4_Joint', 'Leg_J5_Joint', 'Leg_J6_Joint',
                    'Head_J1_joint', 'Head_J2_joint']
        msg.position = [0.] * len(msg.name)
        stamp(msg, time.time() - .05)
        node.joints(msg)
        assert len(published) == 1
        assert published[0].header.stamp == msg.header.stamp
        assert published[0].header.frame_id == 'base_link'
        assert published[0].child_frame_id == 'head_calibrated'
        stamp(msg, time.time()-10)
        node.joints(msg)
        assert len(published) == 1
    finally:
        node.destroy_node()


def camera_pair(seconds):
    image, info = Image(), CameraInfo()
    image.width = info.width = 640
    image.height = info.height = 480
    image.header.frame_id = info.header.frame_id = 'camera_color_optical_frame'
    stamp(image, seconds)
    stamp(info, seconds)
    image.encoding, image.step = 'mono8', 640
    image.data = bytes(640 * 480)
    info.k = [600., 0., 320., 0., 600., 240., 0., 0., 1.]
    info.d, info.distortion_model = [0.] * 5, 'plumb_bob'
    return image, info


@pytest.mark.parametrize('info_first', [False, True])
def test_host_time_pairs_future_frames_and_preserves_calibration(ros, info_first):
    rclpy.init(args=[])
    node = HeadHostTime()
    images, infos = [], []
    node.image_publisher.publish = images.append
    node.info_publisher.publish = infos.append
    try:
        image, info = camera_pair(time.time() + 34.4)
        original = stamp_ns(image.header.stamp)
        before = node.get_clock().now().nanoseconds
        if info_first:
            node.info_cb(info)
            assert not images
            node.image_cb(image)
        else:
            node.image_cb(image)
            assert not images
            node.info_cb(info)
        after = node.get_clock().now().nanoseconds
        assert len(images) == len(infos) == 1
        assert before <= stamp_ns(images[0].header.stamp) <= after
        assert images[0].header.stamp == infos[0].header.stamp
        assert stamp_ns(image.header.stamp) == stamp_ns(info.header.stamp) == original
        assert images[0].data == image.data
        assert images[0].encoding == image.encoding
        assert images[0].header.frame_id == info.header.frame_id
        np.testing.assert_array_equal(infos[0].k, info.k)
        np.testing.assert_array_equal(infos[0].d, info.d)
        assert infos[0].distortion_model == info.distortion_model
        node.image_cb(image)
        node.info_cb(info)
        assert len(images) == 1
    finally:
        node.destroy_node()


def test_host_time_rejects_mismatched_stale_and_backward_frames(ros):
    rclpy.init(args=[])
    node = HeadHostTime()
    published = []
    node.image_publisher.publish = published.append
    node.info_publisher.publish = lambda _: None
    clock = [0.]
    node._monotonic = lambda: clock[0]
    try:
        image, info = camera_pair(100.)
        stamp(info, 101.)
        node.image_cb(image)
        node.info_cb(info)
        assert not published
        clock[0] = 1.
        stamp(info, 100.)
        node.info_cb(info)
        assert not published and not node.images

        for index, mismatch in enumerate(['frame', 'size']):
            image, info = camera_pair(102. + index)
            if mismatch == 'frame':
                info.header.frame_id = 'wrong_optical'
            else:
                info.width = 320
            node.image_cb(image)
            node.info_cb(info)
            assert not published

        image, info = camera_pair(99.)
        node.info_cb(info)
        node.image_cb(image)
        assert not published and not node.images and not node.infos
        image, info = camera_pair(99.1)
        node.info_cb(info)
        node.image_cb(image)
        assert len(published) == 1

        image, info = camera_pair(0.)
        node.image_cb(image)
        node.info_cb(info)
        assert len(published) == 1
        for index in range(12):
            image, _ = camera_pair(105. + index)
            node.image_cb(image)
        assert len(node.images) <= node.p['max_pending_frames']
    finally:
        node.destroy_node()


@pytest.mark.parametrize('args', [
    ['-p', 'image_output_topic:=/camera/camera/color/image_raw'],
    ['-p', 'max_pair_delay:=0.0'],
    ['-p', 'max_pending_frames:=0'],
    ['-p', 'use_sim_time:=true'],
])
def test_host_time_rejects_unsafe_configuration(ros, args):
    rclpy.init(args=['--ros-args'] + args)
    with pytest.raises(ValueError):
        HeadHostTime()
