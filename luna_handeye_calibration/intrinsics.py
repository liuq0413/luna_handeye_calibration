"""采集与预览共用的内参读取和一致性检查。"""
import numpy as np
from sensor_msgs.msg import CameraInfo


def camera_info_from_yaml(intr):
    info = CameraInfo()
    info.header.frame_id = intr['optical_frame']
    info.width, info.height = int(intr['image_width']), int(intr['image_height'])
    info.k = [float(v) for v in intr['camera_matrix']['data']]
    info.d = [float(v) for v in intr['distortion_coefficients']['data']]
    info.distortion_model = intr['distortion_model']
    return info


def validate_camera_info(image, info):
    if info is None:
        raise ValueError('未收到 CameraInfo')
    if (image.width, image.height) != (info.width, info.height):
        raise ValueError('CameraInfo 分辨率与原始图像不匹配')
    if not image.header.frame_id or image.header.frame_id != info.header.frame_id:
        raise ValueError('图像和 CameraInfo 的 optical frame 不匹配')
    if info.distortion_model not in ('plumb_bob', 'rational_polynomial',
                                     'inverse_brown_conrady'):
        raise ValueError('仅支持 Brown-Conrady、RealSense inverse Brown-Conrady，不能混用鱼眼或已校正图像')
    if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
        raise ValueError('不支持未折算内参的 ROI/binning 图像')
    K, D = np.asarray(info.k).reshape(3, 3), np.asarray(info.d)
    if not np.isfinite(K).all() or not np.isfinite(D).all() or K[0, 0] <= 0 or K[1, 1] <= 0:
        raise ValueError('CameraInfo 未标定或数值非法')
    if D.size not in (0, 4, 5, 8, 12, 14):
        raise ValueError('畸变系数长度不支持')
    return K, D
