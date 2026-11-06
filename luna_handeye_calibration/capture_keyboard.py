#!/usr/bin/env python3
"""真实终端按 c/空格调用已有采样服务，不移动机器人、不自动重复采样。"""
import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import Trigger


HELP = '按 c 或空格采集一组，h 显示帮助，q / Ctrl-C 退出；无需回车。请先停稳至少 0.5 秒。'


class KeyboardTerminal:
    """临时启用单键输入，退出或异常时恢复终端；忽略方向键转义序列。"""
    def __init__(self, stream):
        self.fd = stream.fileno()
        self.settings = None
        self.escape = None
        self.eof = False

    def __enter__(self):
        if not os.isatty(self.fd):
            raise ValueError('需要真实交互终端，不能把键盘程序放在 ros2 launch 或管道中')
        self.settings = termios.tcgetattr(self.fd)
        try:
            tty.setcbreak(self.fd)
        except BaseException:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.settings)
            raise
        return self

    def __exit__(self, *_):
        if self.settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.settings)

    def decode(self, data):
        keys = []
        for key in data.decode('ascii', errors='ignore'):
            if self.escape == 'prefix':
                self.escape = 'sequence' if key in '[O' else None
            elif self.escape == 'sequence':
                if '@' <= key <= '~':
                    self.escape = None
            elif key == '\x1b':
                self.escape = 'prefix'
            else:
                keys.append(key)
        return keys

    def read(self, timeout=0.05):
        if not select.select([self.fd], [], [], timeout)[0]:
            return []
        data = os.read(self.fd, 32)
        if not data:
            self.eof = True
            return []
        return self.decode(data)


class KeyboardCapture(Node):
    def __init__(self, node_name='luna_capture_keyboard'):
        super().__init__(node_name)
        self.declare_parameter('capture_service', '/luna_apriltag_collector/capture')
        self.declare_parameter('request_timeout', 5.0)
        self.timeout = self.get_parameter('request_timeout').value
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('request_timeout 必须为正有限值')
        self.service = self.get_parameter('capture_service').value
        self.client = self.create_client(Trigger, self.service)
        self.pending = None
        self.requested_at = None
        self.timed_out = False
        self.successes = 0
        self.failures = 0
        self.last_result = '尚未采样'
        self.last_success = None
        self._monotonic = time.monotonic
        self.get_logger().info(f'交互采样连接 {self.service}；只调用采样服务，不启动或控制机器人。')
        self.get_logger().info(HELP)

    def handle_key(self, key):
        if key in ('q', 'Q', '\x03', '\x04'):
            return False
        if key in ('h', 'H', '?'):
            self.get_logger().info(HELP)
        elif key in ('c', 'C', ' '):
            if self.pending is not None:
                self.get_logger().warning('上次采样尚未返回，忽略本次按键；不会排队或自动重试',
                                          throttle_duration_sec=1)
            elif not self.client.service_is_ready():
                self.last_result = '采样服务未就绪'
                self.last_success = False
                self.get_logger().warning(f'采样服务未就绪：{self.service}；请先启动采集器')
            else:
                self.pending = self.client.call_async(Trigger.Request())
                self.requested_at = self._monotonic()
                self.timed_out = False
                self.last_result = '正在采样，请保持姿态不动'
                self.last_success = None
                self.get_logger().info('正在采样，请保持姿态不动……')
        return True

    def poll_response(self):
        if self.pending is None:
            return
        if self.pending.done():
            future, self.pending = self.pending, None
            try:
                response = future.result()
                self.last_result = response.message
                self.last_success = response.success
                if response.success:
                    self.successes += 1
                    self.get_logger().info(f'采集成功：{response.message}')
                else:
                    self.failures += 1
                    self.get_logger().warning(f'未保存样本：{response.message}')
            except Exception as exc:
                self.last_result = f'保存状态未知：{exc}'
                self.last_success = None
                self.get_logger().error(f'采样响应异常，保存状态未知：{exc}；请核对采集器日志/文件')
        elif not self.timed_out and self._monotonic() - self.requested_at >= self.timeout:
            # 超时不代表服务未保存；保留该请求，禁止静默重发导致重复采样。
            self.timed_out = True
            self.last_result = '响应超时，保存状态未知'
            self.last_success = None
            self.get_logger().warning(
                '采样响应超时，保存状态未知；继续等待且不接受新采样。'
                '请核对采集器日志/文件，可按 q 退出键盘终端。')


def main():
    if not sys.stdin.isatty():
        raise SystemExit('请在真实终端运行 capture_keyboard；不支持 ros2 launch、后台运行或管道输入。')
    node = None
    try:
        # 在 ROS 初始化前管理终端，ROS 退出异常也不会遗留无回显模式。
        with KeyboardTerminal(sys.stdin) as terminal:
            rclpy.init()
            node = KeyboardCapture()
            running = True
            while running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
                node.poll_response()
                for key in terminal.read():
                    if not node.handle_key(key):
                        running = False
                        break
                if terminal.eof:
                    node.get_logger().warning('标准输入已关闭，结束终端采样；采集器继续运行')
                    running = False
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            # Ctrl-C 可能已关闭 ROS context，不能再向失效的 rosout 发布日志。
            if node.context.ok():
                if node.pending is not None:
                    node.get_logger().warning('退出时仍有采样请求，保存状态以采集器日志/文件为准')
                node.get_logger().info(
                    f'键盘终端退出：本终端成功 {node.successes} 次、被拒绝 {node.failures} 次；'
                    '采集器仍运行，已保存数据不删除，也不会自动求解。')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
