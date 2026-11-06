"""T_A_B 将 B 中的点变换到 A，四元数顺序 xyzw。"""
import os
import tempfile
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def validate_transform(value):
    T = np.asarray(value, dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError('变换必须为有限的 4x4 矩阵')
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-7):
        raise ValueError('非法齐次矩阵末行')
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-6) or np.linalg.det(R) < 0.999999:
        raise ValueError('非法旋转矩阵')
    return T


def make_transform(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return validate_transform(T)


def transform_from_dict(value):
    q = np.asarray(value['quaternion'], dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-10:
        raise ValueError('非法 xyzw 四元数')
    return make_transform(Rotation.from_quat(q).as_matrix(), value['translation'])


def transform_to_dict(T):
    T = validate_transform(T)
    return {'translation': T[:3, 3].tolist(),
            'quaternion': Rotation.from_matrix(T[:3, :3]).as_quat().tolist()}


def inverse(T):
    T = validate_transform(T)
    return make_transform(T[:3, :3].T, -T[:3, :3].T @ T[:3, 3])


def atomic_yaml(path, payload):
    # 先写临时文件再替换，避免进程中断留下半个 YAML。
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(payload, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
