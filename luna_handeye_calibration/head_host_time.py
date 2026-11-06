#!/usr/bin/env python3
"""按主机 ROS 接收时刻重发头部图像/内参，不占用相机、不修改原始流。"""
from collections import OrderedDict
from copy import deepcopy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


class HeadHostTime(Node):
    def __init__(self):
        super().__init__('luna_head_host_time')
        defaults = {
            'image_input_topic': '/camera/camera/color/image_raw',
            'info_input_topic': '/camera/camera/color/camera_info',
            'image_output_topic': '/camera/camera/color/image_raw_host_time',
            'info_output_topic': '/camera/camera/color/camera_info_host_time',
            'max_pair_delay': 0.2,
            'max_pending_frames': 4,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if self.get_parameter('use_sim_time').value:
            raise ValueError('主机接收时间模式不支持 use_sim_time')
        topics = [self.p[key] for key in defaults if key.endswith('_topic')]
        resolved = [self.resolve_topic_name(topic) for topic in topics]
        if any(not topic for topic in topics) or len(set(resolved)) != len(resolved):
            raise ValueError('输入/输出话题必须互不相同，不能循环重发或覆盖原始流')
        if not math.isfinite(self.p['max_pair_delay']) or self.p['max_pair_delay'] <= 0:
            raise ValueError('max_pair_delay 必须为正有限值')
        if self.p['max_pending_frames'] < 1:
            raise ValueError('max_pending_frames 必须至少为 1')
        # 深度 1 避免积压旧帧后再打新时间戳；缓存仅用于同帧图像与内参配对。
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.image_publisher = self.create_publisher(Image, self.p['image_output_topic'], qos)
        self.info_publisher = self.create_publisher(CameraInfo, self.p['info_output_topic'], qos)
        self.images = OrderedDict()
        self.infos = OrderedDict()
        self.last_source_stamp = None
        self._monotonic = time.monotonic
        self.create_subscription(Image, self.p['image_input_topic'], self.image_cb, qos)
        self.create_subscription(CameraInfo, self.p['info_input_topic'], self.info_cb, qos)
        self.get_logger().info(
            '头部主机接收时间流已启动；保留原始 optical frame/像素/内参。'
            '该时间包含 ROS 传输延迟，不代表曝光时间；仅用于停稳采样。')

    def prune(self, now):
        for pending in (self.images, self.infos):
            for key, (_, _, received) in list(pending.items()):
                if now - received > self.p['max_pair_delay']:
                    del pending[key]
            while len(pending) > self.p['max_pending_frames']:
                pending.popitem(last=False)

    def image_cb(self, msg):
        # 必须记录接收时刻，而非等待 CameraInfo 或发布时刻。
        arrival = self.get_clock().now().to_msg()
        now = self._monotonic()
        key = stamp_ns(msg.header.stamp)
        self.prune(now)
        if key <= 0:
            return
        if self.last_source_stamp is not None and key <= self.last_source_stamp:
            if key < self.last_source_stamp:
                self.images.clear()
                self.infos.clear()
                self.last_source_stamp = key
                self.get_logger().warning('相机源时间回跳，清空配对缓存并丢弃该帧',
                                          throttle_duration_sec=5)
            # 重复图像不能被重新打戳成新鲜帧。
            return
        self.last_source_stamp = key
        self.images[key] = (msg, arrival, now)
        self.prune(now)
        self.publish_pair(key, now)

    def info_cb(self, msg):
        now = self._monotonic()
        key = stamp_ns(msg.header.stamp)
        self.prune(now)
        if key <= 0:
            return
        self.infos[key] = (msg, None, now)
        self.prune(now)
        self.publish_pair(key, now)

    def publish_pair(self, key, now):
        if key not in self.images or key not in self.infos:
            return
        image, arrival, received = self.images.pop(key)
        info, _, _ = self.infos.pop(key)
        if now - received > self.p['max_pair_delay']:
            return
        if ((image.width, image.height) != (info.width, info.height) or
                not image.header.frame_id or image.header.frame_id != info.header.frame_id):
            self.get_logger().warning('图像与同帧 CameraInfo 的分辨率/frame 不匹配，拒绝重发',
                                      throttle_duration_sec=5)
            return
        image_out, info_out = deepcopy(image), deepcopy(info)
        image_out.header.stamp = arrival
        info_out.header.stamp = arrival
        self.info_publisher.publish(info_out)
        self.image_publisher.publish(image_out)


def main():
    rclpy.init()
    node = None
    try:
        node = HeadHostTime()
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
