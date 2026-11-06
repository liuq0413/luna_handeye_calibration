"""只读加载现有模型，不添加相机或修改 URDF/MJCF。"""
import hashlib
from pathlib import Path
import numpy as np
import pinocchio as pin


class Kinematics:
    def __init__(self, path, base, mount, aliases=None):
        self.path = str(Path(path).expanduser().resolve())
        self.sha256 = hashlib.sha256(Path(self.path).read_bytes()).hexdigest()
        self.model = (pin.buildModelFromMJCF(self.path) if self.path.endswith('.xml')
                      else pin.buildModelFromUrdf(self.path))
        self.data = self.model.createData()
        self.base, self.mount = base, mount
        self.aliases = aliases or {}
        for name in [base, mount]:
            if not self.model.existFrame(name):
                raise ValueError(f'模型缺少 frame: {name}；请指定已有 frame，不会自动猜测')
        self.base_id = self.model.getFrameId(base)
        self.mount_id = self.model.getFrameId(mount)

        def ancestors(frame_id):
            joint_id = self.model.frames[frame_id].parentJoint
            result = set()
            while joint_id:
                result.add(joint_id)
                joint_id = int(self.model.parents[joint_id])
            return result

        # 公共祖先运动在相对变换中抵消，不要求不相关手臂的反馈。
        self.required = sorted(ancestors(self.base_id) ^ ancestors(self.mount_id))
        if not self.required:
            raise ValueError('base 与 mount 之间没有活动关节')
        for j in self.required:
            if self.model.joints[j].nq != 1:
                raise ValueError('当前采集器仅支持标量转动/移动关节')

    def positions(self, joints):
        values = []
        for j in self.required:
            name = self.model.names[j]
            source = self.aliases.get(name, name)
            if source not in joints:
                raise ValueError(f'缺少关节反馈 {source}（模型关节 {name}）')
            value = float(joints[source])
            if not np.isfinite(value):
                raise ValueError(f'非法关节反馈: {source}')
            values.append(value)
        return np.asarray(values)

    def pose(self, joints):
        q = pin.neutral(self.model)
        for j, value in zip(self.required, self.positions(joints)):
            q[self.model.joints[j].idx_q] = value
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return (self.data.oMf[self.base_id].inverse() * self.data.oMf[self.mount_id]).homogeneous.copy()
