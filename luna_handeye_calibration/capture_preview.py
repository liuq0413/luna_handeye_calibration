#!/usr/bin/env python3
"""实时标签预览和按键采样；只读取采集器配置/图像，不打开设备或控制运动。"""
import math
import os
import sys
from contextlib import ExitStack
from pathlib import Path

import cv2
import numpy as np
import yaml
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo

from luna_handeye_calibration.apriltag import AprilTagDetector
from luna_handeye_calibration.capture_keyboard import KeyboardCapture, KeyboardTerminal
from luna_handeye_calibration.collect_apriltag import stamp_seconds
from luna_handeye_calibration.intrinsics import camera_info_from_yaml, validate_camera_info


WINDOW = 'Luna AprilTag - terminal or window: C / Space capture, Q quit'
CONFIG_NAMES = [
    'image_topic', 'camera_info_topic', 'intrinsics_file', 'tag_id', 'tag_size',
    'max_reprojection_px', 'ambiguity_ratio', 'max_age', 'timestamp_source',
]
GREEN, ORANGE, RED = (60, 220, 60), (0, 180, 255), (50, 50, 255)


def short_reason(message):
    # OpenCV Hershey 字体不能显示中文；完整中文原因保留在终端日志。
    reasons = [
        ('双解', 'PnP ambiguous: tilt view / enlarge tag'),
        ('恰好', 'Need exactly one target ID'),
        ('未收到', 'Waiting for image / CameraInfo'),
        ('分辨率', 'Image / CameraInfo resolution mismatch'),
        ('optical frame', 'Optical frame mismatch'),
        ('Brown-Conrady', 'Unsupported distortion model'),
        ('ROI/binning', 'Unsupported ROI / binning'),
        ('未标定', 'Invalid camera intrinsics'),
        ('畸变系数', 'Unsupported distortion coefficients'),
        ('PnP', 'PnP failed / reprojection error too large'),
        ('时间戳', 'Invalid / stale image timestamp'),
        ('未覆盖', 'Joint history does not bracket image time'),
        ('停稳', 'Not stationary / insufficient joint history'),
        ('接近', 'Duplicate pose: change viewpoint'),
        ('缺少关节', 'Missing joint feedback'),
        ('内参/坐标系', 'Session calibration parameters changed'),
        ('尚未采样', 'No capture requested'),
        ('未就绪', 'Capture service unavailable'),
        ('正在采样', 'Capturing: stay still'),
        ('状态未知', 'Save status UNKNOWN: check collector log'),
        ('已保存', 'Saved: see collector log for sample number'),
    ]
    return next((text for fragment, text in reasons if fragment in message),
                'See terminal for full details')


def window_key(code):
    # waitKeyEx 保留方向键的完整编码，不能取低 8 位（左箭头可能误变成 Q）。
    return chr(code) if 0 <= code < 128 else None


def window_closed(name):
    try:
        visible = cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE)
        if visible >= 0:
            return visible == 0
        # 本机 GTK 不支持 VISIBLE，正常窗口也返回 -1；AUTOSIZE 可区分窗口是否存在。
        return cv2.getWindowProperty(name, cv2.WND_PROP_AUTOSIZE) < 0
    except cv2.error:
        return True


def process_inputs(node, window_code, terminal):
    """统一处理两个输入源；终端 EOF 只禁用终端，不关闭仍可操作的图像窗口。"""
    keys = []
    key = window_key(window_code)
    if key is not None:
        keys.append((key, 'window'))
    if terminal is not None:
        try:
            keys.extend((key, 'terminal') for key in terminal.read(timeout=0))
            if terminal.eof:
                node.get_logger().warning('终端输入已关闭；预览继续运行，可在图像窗口按键采样')
                terminal = None
        except (OSError, ValueError) as exc:
            node.get_logger().warning(f'终端输入不可用：{exc}；图像窗口按键仍可使用')
            terminal = None
    for key, source in keys:
        if not node.handle_input_key(key, source):
            return False, terminal
    return True, terminal


def analyze_image(image, info, detector, bridge):
    """在原始分辨率检测；即使内参缺失或 PnP 被拒绝也保留标签框。"""
    canvas = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8').copy()
    gray = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
    corners, ids = detector.detect_markers(gray)
    values = [] if ids is None else ids.reshape(-1).tolist()
    matches = [i for i, value in enumerate(values) if value == detector.tag_id]
    valid, error, pose = False, None, None
    message = '图像中必须恰好出现一个指定 ID 的 AprilTag'
    if len(matches) == 1:
        pixels = corners[matches[0]].reshape(4, 2).astype(np.float64)
        try:
            K, D = validate_camera_info(image, info)
            pose, error, _ = detector.pose(pixels, K, D, info.distortion_model)
            valid, message = True, 'PnP OK'
            draw_D = np.zeros(5) if info.distortion_model == 'inverse_brown_conrady' else D
            cv2.drawFrameAxes(canvas, K, draw_D, cv2.Rodrigues(pose[:3, :3])[0],
                              pose[:3, 3], detector.size * 0.5)
        except ValueError as exc:
            message = str(exc)
    for i, tag_id in enumerate(values):
        color = ((GREEN if valid else ORANGE) if tag_id == detector.tag_id else (160, 160, 160))
        points = np.rint(corners[i].reshape(4, 2)).astype(np.int32)
        cv2.polylines(canvas, [points], True, color, 2, cv2.LINE_AA)
        cv2.circle(canvas, tuple(points[0]), 4, color, -1)
        cv2.putText(canvas, f'ID {tag_id}', tuple(points[0] + [0, -8]),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, color, 2, cv2.LINE_AA)
    return {'canvas': canvas, 'ids': values, 'valid': valid, 'message': message,
            'error': error, 'pose': pose}


class CapturePreview(KeyboardCapture):
    def __init__(self):
        super().__init__('luna_capture_preview')
        self.declare_parameter('collector_node', '')
        self.declare_parameter('preview_fps', 12.0)
        self.fps = self.get_parameter('preview_fps').value
        if not math.isfinite(self.fps) or not 0 < self.fps <= 60:
            raise ValueError('preview_fps 必须在 (0, 60] 内')
        remote = self.get_parameter('collector_node').value
        resolved_service = self.client.srv_name
        if not remote and not resolved_service.endswith('/capture'):
            raise ValueError('自定义 capture 服务名时必须显式指定 collector_node')
        self.remote = remote or resolved_service.rsplit('/', 1)[0]
        self.config_client = self.create_client(GetParameters, self.remote.rstrip('/') + '/get_parameters')
        self.config_future = None
        self.config_requested_at = None
        self.p = None
        self.image = self.info = self.file_info = None
        self.image_received = None
        self.bridge = CvBridge()
        self.detector = None
        self.analysis = None
        self.analyzed_image = None
        self.last_draw = -math.inf
        self.last_analysis_at = None
        self.actual_fps = 0.
        self.last_detection_message = None
        self.last_input = 'none'
        self.subscriptions_owned = []
        self.get_logger().info('预览与采样已合并；正在读取采集器实际参数。')

    def poll_configuration(self):
        if self.p is not None:
            return
        now = self._monotonic()
        if self.config_future is None:
            if self.config_client.service_is_ready():
                request = GetParameters.Request(names=CONFIG_NAMES)
                self.config_future = self.config_client.call_async(request)
                self.config_requested_at = now
            else:
                self.get_logger().warning(f'等待采集器参数服务：{self.remote}/get_parameters',
                                          throttle_duration_sec=5)
        elif self.config_future.done():
            response = self.config_future.result()
            self.config_future = None
            if response is None or len(response.values) != len(CONFIG_NAMES):
                raise ValueError('采集器参数响应不完整；请使用本包 collect_apriltag')
            self.configure(dict(zip(CONFIG_NAMES, map(parameter_value_to_python, response.values))))
        elif now - self.config_requested_at >= self.timeout:
            # 这里只重试只读参数请求，绝不重试有保存副作用的 capture。
            self.config_client.remove_pending_request(self.config_future)
            self.config_future = None
            self.get_logger().warning('读取采集器参数超时，将重新查询；请检查 DDS/domain')

    def configure(self, params):
        if any(value is None for value in params.values()):
            raise ValueError('采集器缺少预览所需参数；请使用当前版本 collect_apriltag')
        detector = AprilTagDetector(params['tag_id'], params['tag_size'],
                                    params['max_reprojection_px'], params['ambiguity_ratio'])
        file_info = None
        if params['intrinsics_file']:
            with open(Path(params['intrinsics_file']).expanduser(), encoding='utf-8') as stream:
                file_info = camera_info_from_yaml(yaml.safe_load(stream))
        # 检测较慢时只保留最新图像，不为预览积压相机流。
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions_owned = [
            self.create_subscription(Image, params['image_topic'], self.image_cb, qos),
            self.create_subscription(CameraInfo, params['camera_info_topic'], self.info_cb, qos),
        ]
        self.detector, self.file_info, self.p = detector, file_info, params
        self.get_logger().info(
            f"预览已连接 {params['image_topic']}；36h11 ID={params['tag_id']}，"
            f"边长={params['tag_size']*1000:.1f}mm，时间策略={params['timestamp_source']}。"
            '绿色只表示图像/PnP 通过，保存还需采集器同步、停稳及姿态检查。')

    def image_cb(self, msg):
        self.image = msg
        self.image_received = self._monotonic()

    def info_cb(self, msg):
        self.info = msg

    def handle_key(self, key):
        if self.p is None and key in ('c', 'C', ' '):
            self.get_logger().warning('尚未读取采集器配置，不发送采样请求')
            return True
        return super().handle_key(key)

    def handle_input_key(self, key, source):
        if key in ('c', 'C', ' ', 'h', 'H', '?', 'q', 'Q', '\x03', '\x04'):
            label = 'Space' if key == ' ' else 'Ctrl-C' if key == '\x03' else 'Ctrl-D' if key == '\x04' else key.upper()
            self.last_input = f'{source}: {label}'
            origin = '启动终端' if source == 'terminal' else '图像窗口'
            self.get_logger().info(f'收到{origin}按键 {label}')
        return self.handle_key(key)

    def render(self):
        now = self._monotonic()
        if now - self.last_draw < 1. / self.fps:
            return None
        self.last_draw = now
        if self.image is not None and self.image is not self.analyzed_image:
            try:
                self.analysis = analyze_image(self.image, self.file_info or self.info,
                                               self.detector, self.bridge)
                if self.analysis['message'] != self.last_detection_message:
                    self.last_detection_message = self.analysis['message']
                    self.get_logger().info(
                        f"实时检测 IDs={self.analysis['ids']}：{self.analysis['message']}")
            except Exception as exc:
                self.analysis = None
                self.get_logger().error(f'预览图像处理失败：{exc}', throttle_duration_sec=5)
            if self.last_analysis_at is not None and now > self.last_analysis_at:
                self.actual_fps = 1. / (now - self.last_analysis_at)
            self.last_analysis_at, self.analyzed_image = now, self.image
        canvas = (self.analysis['canvas'] if self.analysis is not None
                  else np.zeros((480, 640, 3), np.uint8))
        height, width = canvas.shape[:2]
        output = np.zeros((height + 190, max(width, 800), 3), np.uint8)
        output[:height, :width] = canvas
        lines = [('Terminal OR window: C / Space capture | H help | Q quit | Stop >= 0.5s',
                  (220, 220, 220))]
        if self.p is None:
            lines.append((f'Waiting for collector: {self.remote}', ORANGE))
        else:
            lines.append((f"36h11 ID={self.p['tag_id']} size={self.p['tag_size']*1000:.1f}mm | "
                          f"{self.p['timestamp_source']} | Preview {self.actual_fps:.1f}Hz", (220, 220, 220)))
        if self.analysis is not None:
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp_seconds(self.image.header.stamp)
            fresh = (stamp_seconds(self.image.header.stamp) > 0 and 0 <= age <= self.p['max_age']
                     and now - self.image_received <= self.p['max_age'])
            details = (f"PnP OK, RMS={self.analysis['error']:.3f}px" if self.analysis['valid']
                       else short_reason(self.analysis['message']))
            if not fresh:
                details = 'STALE / INVALID TIMESTAMP - do not use cached image'
            lines.append((f"IDs: {self.analysis['ids']} | age={age*1000:.1f}ms | {details}",
                          GREEN if self.analysis['valid'] and fresh else ORANGE if fresh else RED))
        else:
            lines.append(('Waiting for valid image', ORANGE))
        if self.pending is not None:
            capture = 'TIMEOUT: save status UNKNOWN' if self.timed_out else 'Capturing: stay still'
        else:
            capture = short_reason(self.last_result)
        lines.append((f'Capture [{self.last_input}]: {capture}', GREEN if self.last_success else ORANGE))
        lines.append((f'This window: saved={self.successes}, rejected={self.failures} (not session total)',
                      (220, 220, 220)))
        lines.append(('PnP OK is NOT sample acceptance. Full capture reason / sample number: terminal.',
                      (160, 160, 160)))
        for index, (text, color) in enumerate(lines):
            cv2.putText(output, text, (8, height + 24 + 29*index), cv2.FONT_HERSHEY_SIMPLEX,
                        .48, color, 1, cv2.LINE_AA)
        return output


def main():
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        raise SystemExit('实时窗口需要图形桌面/DISPLAY；无桌面时请用 capture_keyboard。')
    rclpy.init()
    node, window_open = None, False
    try:
        node = CapturePreview()
        with ExitStack() as inputs:
            terminal = None
            if sys.stdin.isatty():
                try:
                    terminal = inputs.enter_context(KeyboardTerminal(sys.stdin))
                except (OSError, ValueError) as exc:
                    node.get_logger().warning(f'无法启用终端单键输入：{exc}')
            if terminal is not None:
                node.get_logger().info(
                    '终端+图像窗口按键均已启用：在任一处按 c/空格采样、h 帮助、q 退出；无需回车。')
            else:
                node.get_logger().warning('未启用终端输入；请聚焦图像窗口按 c/空格采样、q 退出')
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            window_open = True
            cv2.resizeWindow(WINDOW, 1000, 800)
            running = True
            while running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.01)
                node.poll_configuration()
                node.poll_response()
                frame = node.render()
                if frame is not None:
                    cv2.imshow(WINDOW, frame)
                running, terminal = process_inputs(node, cv2.waitKeyEx(1), terminal)
                if window_closed(WINDOW):
                    break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except cv2.error as exc:
        raise SystemExit(f'OpenCV 图像窗口异常，请检查桌面/GUI 支持：{exc}') from exc
    finally:
        if window_open:
            cv2.destroyAllWindows()
        if node:
            if node.context.ok():
                if node.pending is not None:
                    node.get_logger().warning('退出时仍有采样请求，保存状态以采集器日志/文件为准')
                node.get_logger().info('图像窗口退出；采集器继续运行，不删除数据、不自动求解。')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
