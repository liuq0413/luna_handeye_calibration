#!/usr/bin/env python3
"""离线 Eye-in-Hand 求解：G_i X C_i = Y，输出 X=T_mount_camera。"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from luna_handeye_calibration.se3 import (
    inverse, make_transform, transform_to_dict, validate_transform, atomic_yaml,
)


def pack(T):
    return np.r_[T[:3, 3], Rotation.from_matrix(T[:3, :3]).as_rotvec()]


def unpack(x):
    return make_transform(Rotation.from_rotvec(x[3:6]).as_matrix(), x[:3])


def excitation(poses):
    # 多组绕非平行轴的相对转动才能约束完整平移；6x2 速度雅可比的秩不是此处的判据。
    motions = [inverse(poses[0]) @ pose for pose in poses[1:]]
    matrix = np.vstack([motion[:3, :3] - np.eye(3) for motion in motions])
    singular = np.linalg.svd(matrix, compute_uv=False)
    if singular[-1] < 0.05 or singular[-1] / max(singular[0], 1e-12) < 0.01:
        raise ValueError('运动退化/角度跨度不足：需要两个以上非平行旋转轴的多姿态组合')
    return singular.tolist()


def errors(G, C, X, Y):
    predicted = [g @ X @ c for g, c in zip(G, C)]
    translation = [float(np.linalg.norm(p[:3, 3] - Y[:3, 3])) for p in predicted]
    rotation = [float(np.rad2deg(Rotation.from_matrix(Y[:3, :3].T @ p[:3, :3]).magnitude()))
                for p in predicted]
    return {'translation_m': translation, 'rotation_deg': rotation,
            'translation_rms_m': float(np.sqrt(np.mean(np.square(translation)))),
            'rotation_rms_deg': float(np.sqrt(np.mean(np.square(rotation)))),
            'translation_max_m': max(translation), 'rotation_max_deg': max(rotation)}


def calibrate(payload, translation_limit=0.02, rotation_limit_deg=3.0):
    if translation_limit <= 0 or rotation_limit_deg <= 0:
        raise ValueError('验收阈值必须为正')
    if payload.get('schema_version') != 1:
        raise ValueError('不支持的数据格式；请用当前采集器重新采集')
    samples = payload['samples']
    if len(samples) < 12:
        raise ValueError('至少 12 组；建议 20–40 组不同姿态')
    G = [validate_transform(s['mount_pose']) for s in samples]
    C = [validate_transform(s['camera_tag']) for s in samples]
    # 固定留出每四组中的一组，未参与拟合的数据用于验收。
    validation = list(range(3, len(samples), 4))
    train = [i for i in range(len(samples)) if i not in validation]
    singular = excitation([G[i] for i in train])
    R, t = cv2.calibrateHandEye(
        [G[i][:3, :3] for i in train], [G[i][:3, 3] for i in train],
        [C[i][:3, :3] for i in train], [C[i][:3, 3] for i in train],
        method=cv2.CALIB_HAND_EYE_PARK)
    X0 = make_transform(R, t)
    targets = [G[i] @ X0 @ C[i] for i in train]
    Y0 = make_transform(Rotation.from_matrix(np.array([p[:3, :3] for p in targets])).mean().as_matrix(),
                        np.median([p[:3, 3] for p in targets], axis=0))

    def residual(parameters):
        X, Y = unpack(parameters[:6]), unpack(parameters[6:])
        values = []
        for i in train:
            P = G[i] @ X @ C[i]
            # 分开归一化米和弧度，避免两种量纲直接混合。
            values.extend((P[:3, 3] - Y[:3, 3]) / 0.005)
            values.extend(Rotation.from_matrix(Y[:3, :3].T @ P[:3, :3]).as_rotvec() / np.deg2rad(1))
        return np.asarray(values)

    fit = least_squares(residual, np.r_[pack(X0), pack(Y0)], loss='soft_l1', max_nfev=2000)
    if not fit.success or not np.isfinite(fit.x).all():
        raise ValueError(f'优化未收敛: {fit.message}')
    X, Y = unpack(fit.x[:6]), unpack(fit.x[6:])
    train_errors = errors([G[i] for i in train], [C[i] for i in train], X, Y)
    test_errors = errors([G[i] for i in validation], [C[i] for i in validation], X, Y)
    accepted = all(e['translation_max_m'] <= translation_limit and
                   e['rotation_max_deg'] <= rotation_limit_deg for e in [train_errors, test_errors])
    report = {'accepted': accepted, 'train_indices': train, 'validation_indices': validation,
              'excitation_singular_values': singular, 'training': train_errors, 'validation': test_errors,
              'translation_limit_m': translation_limit, 'rotation_limit_deg': rotation_limit_deg,
              'T_base_tag': transform_to_dict(Y), 'solver': 'OpenCV PARK + scipy soft_l1 refinement'}
    return X, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('samples')
    parser.add_argument('--output', required=True)
    parser.add_argument('--child-frame', required=True, help='使用未被其他节点发布的独立标定 frame')
    parser.add_argument('--translation-limit', type=float, default=0.02)
    parser.add_argument('--rotation-limit-deg', type=float, default=3.0)
    args = parser.parse_args()
    output = Path(args.output).expanduser().resolve()
    report_path = Path(str(output) + '.report.yaml')
    if output.exists() or report_path.exists():
        parser.error('输出或报告已存在，拒绝覆盖；请使用新文件名')
    try:
        with open(args.samples, encoding='utf-8') as stream:
            payload = yaml.safe_load(stream)
        if args.child_frame in (payload['camera_frame'], payload['base_frame'], payload['mount_frame']):
            raise ValueError('child-frame 必须为独立名称，避免覆盖相机驱动或机器人 TF')
        X, report = calibrate(payload, args.translation_limit, args.rotation_limit_deg)
        atomic_yaml(report_path, report)
        if not report['accepted']:
            raise ValueError(f'训练/留出验证未通过；只保存报告 {report_path}，不生成部署外参')
        result = {key: payload[key] for key in [
            'schema_version', 'base_frame', 'mount_frame', 'camera_frame', 'model_path',
            'model_sha256', 'joint_aliases', 'tag_family', 'tag_id', 'tag_size_m']}
        result.update({'parent': payload['mount_frame'], 'child': args.child_frame,
                       'timestamp_source': payload.get('timestamp_source', 'image_header'),
                       'T_mount_camera': transform_to_dict(X), 'validation_passed': True,
                       'report_file': str(report_path)})
        atomic_yaml(output, result)
        print(f'已保存 {output}\n留出验证: {report["validation"]}')
    except (ValueError, KeyError, OSError, cv2.error) as exc:
        parser.exit(1, f'标定失败: {exc}\n')


if __name__ == '__main__':
    main()
