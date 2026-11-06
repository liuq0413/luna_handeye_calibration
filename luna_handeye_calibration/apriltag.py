"""AprilTag 36h11 四角检测及双解筛选，兼容 OpenCV 4.5/4.7+。"""
import cv2
import numpy as np
from luna_handeye_calibration.se3 import make_transform


class AprilTagDetector:
    def __init__(self, tag_id, size, max_error=1.5, ambiguity_ratio=1.5):
        if size <= 0 or not np.isfinite(size) or tag_id < 0:
            raise ValueError('标签边长必须为正数（米），ID 必须非负')
        if max_error <= 0 or ambiguity_ratio <= 1:
            raise ValueError('重投影阈值必须为正，双解误差比必须大于 1')
        self.tag_id, self.size = tag_id, size
        self.max_error, self.ambiguity_ratio = max_error, ambiguity_ratio
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        if tag_id >= len(self.dictionary.bytesList):
            raise ValueError('标签 ID 超出 36h11 字典范围')
        self.params = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'DetectorParameters')
                       else cv2.aruco.DetectorParameters_create())
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = (cv2.aruco.ArucoDetector(self.dictionary, self.params)
                         if hasattr(cv2.aruco, 'ArucoDetector') else None)
        h = size / 2
        self.points = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)

    def detect_markers(self, gray):
        """返回全部标签角点/ID，预览与采样共用同一检测器。"""
        if self.detector:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        return corners, ids

    @staticmethod
    def _inverse_brown_pixels(pixels, K, D):
        """将 RealSense inverse Brown-Conrady 像素校正到无畸变像素平面。"""
        pixels = np.asarray(pixels, dtype=np.float64)
        K = np.asarray(K, dtype=np.float64)
        D = np.asarray(D, dtype=np.float64)
        if D.size < 5:
            raise ValueError('RealSense inverse Brown-Conrady 畸变系数不足 5 个')
        fx, fy, cx, cy = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
        x = (pixels[:, 0] - cx) / fx
        y = (pixels[:, 1] - cy) / fy
        r2 = x * x + y * y
        k1, k2, p1, p2, k3 = [float(value) for value in D[:5]]
        radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        xu = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        yu = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        return np.column_stack((fx * xu + cx, fy * yu + cy))

    @classmethod
    def _pnp_inputs(cls, pixels, K, D, distortion_model):
        if distortion_model == 'inverse_brown_conrady':
            return cls._inverse_brown_pixels(pixels, K, D), K, np.zeros(5, dtype=np.float64)
        return pixels, K, D

    def detect(self, gray, K, D, distortion_model='plumb_bob'):
        corners, ids = self.detect_markers(gray)
        matches = [] if ids is None else np.flatnonzero(ids.reshape(-1) == self.tag_id)
        if len(matches) != 1:
            raise ValueError('图像中必须恰好出现一个指定 ID 的 AprilTag')
        pixels = corners[int(matches[0])].reshape(4, 2).astype(np.float64)
        if distortion_model == 'plumb_bob':
            # 保持旧版调用方对 pose(pixels, K, D) 的兼容性。
            return self.pose(pixels, K, D)
        return self.pose(pixels, K, D, distortion_model)

    def pose(self, pixels, K, D, distortion_model='plumb_bob'):
        pnp_pixels, pnp_K, pnp_D = self._pnp_inputs(
            np.asarray(pixels, dtype=np.float64), K, D, distortion_model)
        result = cv2.solvePnPGeneric(self.points, pnp_pixels, pnp_K, pnp_D,
                                     flags=cv2.SOLVEPNP_IPPE_SQUARE)
        candidates = []
        for rvec, tvec in zip(result[1], result[2]):
            R = cv2.Rodrigues(rvec)[0]
            if not np.isfinite(R).all() or not np.isfinite(tvec).all():
                continue
            if np.min((R @ self.points.T + tvec)[2]) <= 0:
                continue
            projected = cv2.projectPoints(self.points, rvec, tvec, pnp_K, pnp_D)[0].reshape(-1, 2)
            error = float(np.sqrt(np.mean(np.sum((pnp_pixels - projected)**2, axis=1))))
            candidates.append((error, make_transform(R, tvec)))
        candidates.sort(key=lambda item: item[0])
        if not candidates or candidates[0][0] > self.max_error:
            raise ValueError('PnP 失败、负深度或重投影误差过大')
        if len(candidates) > 1 and candidates[1][0] < self.ambiguity_ratio * max(candidates[0][0], 0.1):
            raise ValueError('平面 PnP 双解不明确，请改变观察角度后重试')
        return candidates[0][1], candidates[0][0], pixels
