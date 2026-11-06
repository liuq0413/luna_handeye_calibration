"""无 ROS 依赖的关节时间匹配和停稳检查。"""
import numpy as np


def match_stationary(history, stamp, positions, tolerance=0.05, window=0.3, motion=0.003):
    near = sorted([(t, q) for t, q in history if abs(t - stamp) <= window], key=lambda x: x[0])
    if len(near) < 3:
        raise ValueError('关节历史不足，请停稳后等待再采样')
    before = [(t, q) for t, q in near if t <= stamp]
    after = [(t, q) for t, q in near if t >= stamp]
    if not before or not after:
        raise ValueError('关节时间戳未覆盖图像时间，请等待下一帧后重试')
    t0, q0 = before[-1]
    t1, q1 = after[0]
    if stamp - t0 > tolerance or t1 - stamp > tolerance:
        raise ValueError('图像与关节反馈时间差过大')
    if near[0][0] > stamp - window * 0.8:
        raise ValueError('停稳历史时长不足')
    if np.max(np.diff([t for t, _ in near])) > 2 * tolerance:
        raise ValueError('关节反馈存在长时间缺口')
    values = np.asarray([positions(q) for _, q in near])
    if np.max(np.ptp(values, axis=0)) > motion:
        raise ValueError('关节尚未停稳，拒绝采样（使用位置差，不依赖驱动速度字段）')
    nearest = q0 if stamp - t0 <= t1 - stamp else q1
    return dict(nearest), min(stamp - t0, t1 - stamp)
