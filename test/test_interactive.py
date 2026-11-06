"""键盘/预览回归；隔离 ROS domain，不连接真实硬件、不采集真实数据。"""
import os
import termios
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger

from luna_handeye_calibration.apriltag import AprilTagDetector
from luna_handeye_calibration.capture_keyboard import KeyboardCapture, KeyboardTerminal
from luna_handeye_calibration.capture_preview import (
    CapturePreview, CONFIG_NAMES, analyze_image, short_reason, window_key, window_closed, process_inputs,
)
from luna_handeye_calibration.intrinsics import camera_info_from_yaml, validate_camera_info


@pytest.fixture
def ros(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '173')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=[])
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.mark.parametrize('fail', [False, True])
def test_terminal_keys_and_restores_on_exception(fail):
    master, slave = os.openpty()
    with os.fdopen(slave, 'r') as stream:
        original = termios.tcgetattr(slave)
        try:
            with KeyboardTerminal(stream) as terminal:
                assert not termios.tcgetattr(slave)[3] & termios.ICANON
                os.write(master, b'c q')
                assert terminal.read(.2) == ['c', ' ', 'q']
                assert terminal.decode(b'\x1b[') == []
                assert terminal.decode(b'C\x1bODh') == ['h']
                if fail:
                    raise RuntimeError('test')
        except RuntimeError:
            assert fail
        assert termios.tcgetattr(slave) == original
    os.close(master)


def test_terminal_rejects_non_tty(tmp_path):
    with (tmp_path / 'not_terminal').open('w+') as stream:
        with pytest.raises(ValueError, match='真实交互终端'):
            with KeyboardTerminal(stream):
                pass


def test_terminal_eof_is_not_converted_to_ctrl_d(monkeypatch):
    master, slave = os.openpty()
    try:
        with os.fdopen(slave, 'r') as stream, KeyboardTerminal(stream) as terminal:
            monkeypatch.setattr('select.select', lambda *_: ([terminal.fd], [], []))
            monkeypatch.setattr(os, 'read', lambda *_: b'')
            assert terminal.read(timeout=0) == []
            assert terminal.eof
    finally:
        os.close(master)


class FakeClient:
    def __init__(self):
        self.ready, self.requests, self.future = False, [], None

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        self.future = Future()
        return self.future


def test_keyboard_request_guard_results_and_unknown_timeout(ros):
    node = KeyboardCapture()
    fake, clock = FakeClient(), [0.]
    node.client = fake
    node._monotonic = lambda: clock[0]
    try:
        node.handle_key('c')
        assert not fake.requests and '未就绪' in node.last_result
        fake.ready = True
        node.handle_key(' ')
        node.handle_key('C')
        assert len(fake.requests) == 1
        clock[0] = 6.
        node.poll_response()
        assert node.timed_out and node.pending is fake.future
        node.handle_key('c')
        assert len(fake.requests) == 1
        fake.future.set_result(Trigger.Response(success=True, message='已保存第 7 组'))
        node.poll_response()
        assert node.pending is None and node.successes == 1
        assert node.last_success and node.last_result == '已保存第 7 组'
        node.handle_key('c')
        fake.future.set_result(Trigger.Response(success=False, message='关节尚未停稳'))
        node.poll_response()
        assert node.failures == 1 and not node.last_success
        node.handle_key('c')
        fake.future.set_exception(RuntimeError('network'))
        node.poll_response()
        assert node.last_success is None and '状态未知' in node.last_result
        assert not node.handle_key('q')
        assert not node.handle_key('\x03')
        assert node.handle_key('h')
        assert len(fake.requests) == 3
    finally:
        node.destroy_node()


@pytest.mark.parametrize('code', [-1, 65361, 65362, 65363, 0x250000])
def test_window_arrow_keys_do_not_capture_or_quit(code):
    assert window_key(code) is None
    assert window_key(ord('c')) == 'c'
    assert window_key(ord('q')) == 'q'


@pytest.mark.parametrize('visible,autosize,closed', [
    (-1., 0., False), (-1., -1., True), (0., 0., True), (1., 0., False),
])
def test_window_close_detection_supports_gtk(monkeypatch, visible, autosize, closed):
    monkeypatch.setattr(cv2, 'getWindowProperty',
                        lambda name, prop: visible if prop == cv2.WND_PROP_VISIBLE else autosize)
    assert window_closed('test') is closed


def test_window_close_detection_handles_destroyed_window(monkeypatch):
    def destroyed(*_):
        raise cv2.error('window destroyed')
    monkeypatch.setattr(cv2, 'getWindowProperty', destroyed)
    assert window_closed('test')


def test_combined_input_terminal_and_window_share_one_capture(ros):
    node = CapturePreview()
    params = dict(zip(CONFIG_NAMES, ['/test/image', '/test/info', '', 0, .02, 1.5, 1.5, .5,
                                    'ros_host_arrival']))
    node.configure(params)
    fake = FakeClient()
    fake.ready = True
    node.client = fake
    master, slave = os.openpty()
    try:
        with os.fdopen(slave, 'r') as stream:
            original = termios.tcgetattr(slave)
            with KeyboardTerminal(stream) as terminal:
                os.write(master, b'c')
                running, selected = process_inputs(node, -1, terminal)
                assert running and selected is terminal
                assert node.last_input == 'terminal: C'
                assert len(fake.requests) == 1
                # 同时来的图像窗口按键不能产生第二个尚未完成的请求。
                running, selected = process_inputs(node, ord('c'), selected)
                assert running and selected is terminal and len(fake.requests) == 1
                fake.future.set_result(Trigger.Response(success=True, message='已保存第 1 组'))
                node.poll_response()
                assert node.successes == 1
                os.write(master, b'h')
                assert process_inputs(node, -1, terminal)[0]
                assert node.last_input == 'terminal: H'
                assert node.render() is not None
                os.write(master, b'q')
                assert not process_inputs(node, -1, terminal)[0]
                assert node.context.ok()
            assert termios.tcgetattr(slave) == original
    finally:
        os.close(master)
        node.destroy_node()


@pytest.mark.parametrize('broken', ['eof', 'error'])
def test_combined_input_stdin_failure_does_not_close_preview(broken):
    handled, warnings = [], []
    node = SimpleNamespace(
        handle_input_key=lambda key, source: handled.append((key, source)) or True,
        get_logger=lambda: SimpleNamespace(warning=warnings.append),
    )
    terminal = SimpleNamespace(eof=broken == 'eof')
    if broken == 'eof':
        terminal.read = lambda **_: []
    else:
        def unreadable(**_):
            raise OSError('input closed')
        terminal.read = unreadable
    running, selected = process_inputs(node, ord('c'), terminal)
    assert running and selected is None
    assert handled == [('c', 'window')] and len(warnings) == 1
    assert process_inputs(node, ord('h'), selected)[0]
    assert handled[-1] == ('h', 'window') and len(warnings) == 1


def test_preview_reads_actual_collector_config_and_captures(ros):
    server = Node('luna_apriltag_collector')
    params = dict(zip(CONFIG_NAMES, [
        '/test/image_host_time', '/test/info_host_time', '', 0, .02, 1.5, 1.5, .5,
        'ros_host_arrival',
    ]))
    for name, value in params.items():
        server.declare_parameter(name, value)
    calls = []

    def capture(request, response):
        calls.append(request)
        response.success, response.message = True, '已保存第 1 组'
        return response

    server.create_service(Trigger, '~/capture', capture)
    node = CapturePreview()
    try:
        # 未读到配置不能静默用错误的相机/尺寸触发采样。
        node.handle_key('c')
        assert node.pending is None
        deadline = time.monotonic() + 5
        while node.p is None and time.monotonic() < deadline:
            rclpy.spin_once(server, timeout_sec=.01)
            rclpy.spin_once(node, timeout_sec=.01)
            node.poll_configuration()
        assert node.p == params
        assert node.detector.tag_id == 0 and node.detector.size == .02
        assert node.subscriptions_owned[0].topic_name == '/test/image_host_time'
        node.handle_key('c')
        node.handle_key('c')
        deadline = time.monotonic() + 5
        while node.pending is not None and time.monotonic() < deadline:
            rclpy.spin_once(server, timeout_sec=.01)
            rclpy.spin_once(node, timeout_sec=.01)
            node.poll_response()
        assert len(calls) == 1 and node.successes == 1
        assert not node.handle_key('q')
        assert server.context.ok()
    finally:
        node.destroy_node()
        server.destroy_node()


def tag_image(two_targets=False):
    detector = AprilTagDetector(0, .02)
    marker = (cv2.aruco.generateImageMarker(detector.dictionary, 0, 120)
              if hasattr(cv2.aruco, 'generateImageMarker')
              else cv2.aruco.drawMarker(detector.dictionary, 0, 120))
    gray = np.full((480, 640), 255, np.uint8)
    if two_targets:
        gray[180:300, 120:240] = marker
        gray[180:300, 400:520] = marker
    else:
        # 放在光轴正中，避免偏轴透视让其中一个 PnP 解明显占优。
        gray[180:300, 260:380] = marker
    bridge = CvBridge()
    image = bridge.cv2_to_imgmsg(gray, encoding='mono8')
    image.header.frame_id = 'test_optical'
    info = CameraInfo()
    info.header.frame_id, info.width, info.height = 'test_optical', 640, 480
    info.k, info.d = [600., 0., 320., 0., 600., 240., 0., 0., 1.], [0.]*5
    info.distortion_model = 'plumb_bob'
    return detector, bridge, image, info


def test_preview_draws_tag_even_without_intrinsics_or_with_ambiguous_pnp():
    detector, bridge, image, info = tag_image()
    before = bytes(image.data)
    result = analyze_image(image, None, detector, bridge)
    assert result['ids'] == [0] and not result['valid']
    assert 'CameraInfo' in result['message']
    assert np.any(result['canvas'][:, :, 0] != result['canvas'][:, :, 1])
    # 正视方形的双解不能因预览而放宽；角点仍可见。
    result = analyze_image(image, info, detector, bridge)
    assert result['ids'] == [0] and not result['valid']
    assert '双解' in result['message']
    assert bytes(image.data) == before


def test_preview_unique_target_and_pose_status():
    detector, bridge, image, info = tag_image(two_targets=True)
    result = analyze_image(image, info, detector, bridge)
    assert result['ids'] == [0, 0] and not result['valid']
    detector.tag_id = 1
    result = analyze_image(image, info, detector, bridge)
    assert not result['valid']
    detector, bridge, image, info = tag_image()
    pose = np.eye(4)
    pose[2, 3] = .2
    detector.pose = lambda pixels, *_: (pose, .2, pixels)
    result = analyze_image(image, info, detector, bridge)
    assert result['valid'] and result['error'] == .2


def test_shared_intrinsics_checks_and_file_conversion():
    _, _, image, info = tag_image()
    intr = {'optical_frame': 'test_optical', 'image_width': 640, 'image_height': 480,
            'camera_matrix': {'data': list(info.k)}, 'distortion_coefficients': {'data': list(info.d)},
            'distortion_model': 'plumb_bob'}
    file_info = camera_info_from_yaml(intr)
    K, D = validate_camera_info(image, file_info)
    assert K[0, 0] == 600 and D.size == 5
    for field, wrong, message in [
        ('width', 320, '分辨率'), ('distortion_model', 'equidistant', 'Brown-Conrady'),
        ('binning_x', 2, 'ROI/binning'), ('k', [0.]*9, '未标定'), ('d', [0.]*3, '畸变系数'),
    ]:
        invalid = camera_info_from_yaml(intr)
        setattr(invalid, field, wrong)
        with pytest.raises(ValueError, match=message):
            validate_camera_info(image, invalid)
    file_info.header.frame_id = 'wrong'
    with pytest.raises(ValueError, match='optical frame'):
        validate_camera_info(image, file_info)


def test_preview_stale_frames_are_not_reported_as_live(ros):
    node = CapturePreview()
    detector, bridge, image, info = tag_image()
    params = dict(zip(CONFIG_NAMES, ['/test/image', '/test/info', '', 0, .02, 1.5, 1.5, .5,
                                    'ros_host_arrival']))
    node.configure(params)
    pose = np.eye(4)
    pose[2, 3] = .2
    node.detector.pose = lambda pixels, *_: (pose, .2, pixels)
    clock = [1.]
    node._monotonic = lambda: clock[0]
    now = node.get_clock().now().to_msg()
    image.header.stamp = now
    node.info_cb(info)
    node.image_cb(image)
    try:
        frame = node.render()
        assert frame.shape == (670, 800, 3)
        assert node.analysis['valid']
        cached = node.analysis
        clock[0] = 2.
        frame_stale = node.render()
        assert node.analysis is cached
        assert not np.array_equal(frame[480:], frame_stale[480:])
        assert node.render() is None
        assert short_reason('平面 PnP 双解不明确').startswith('PnP ambiguous')
    finally:
        node.destroy_node()
