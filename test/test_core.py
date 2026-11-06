from pathlib import Path
import os
import subprocess
import sys
import yaml
import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from luna_handeye_calibration.apriltag import AprilTagDetector
from luna_handeye_calibration.kinematics import Kinematics
from luna_handeye_calibration.se3 import (
    inverse, make_transform, transform_from_dict, transform_to_dict, validate_transform,
)
from luna_handeye_calibration.solve_handeye import calibrate, excitation
from luna_handeye_calibration.sampling import match_stationary


MODEL = Path(__file__).resolve().parents[2] / 'marvin_description/mjcf/Gento_Luna/Gento_Luna.xml'


def synthetic(head=False):
    rng = np.random.default_rng(42)
    X = make_transform(Rotation.from_euler('xyz', [.2, -.3, .1]).as_matrix(), [.08, -.02, .13])
    Y = make_transform(Rotation.from_euler('xyz', [.5, .1, -.2]).as_matrix(), [.6, -.1, .3])
    samples = []
    fk = Kinematics(MODEL, 'base_link', 'Head_J2_Link') if head else None
    for _ in range(28):
        if fk:
            # 仅改变头部两个轴，验证无需躯干运动也能恢复完整外参。
            joints = {fk.model.names[j]: (rng.uniform(-0.3, 0.3)
                                         if fk.model.names[j].startswith('Head') else 0.)
                      for j in fk.required}
            G = fk.pose(joints)
        else:
            G = make_transform(Rotation.from_rotvec(rng.normal(size=3)).as_matrix(), rng.normal(size=3)*.2)
        C = inverse(X) @ inverse(G) @ Y
        samples.append({'mount_pose': G.tolist(), 'camera_tag': C.tolist()})
    return {'schema_version': 1, 'samples': samples}, X


@pytest.mark.parametrize('head', [False, True])
def test_exact_handeye_direction_and_head_observability(head):
    payload, X = synthetic(head)
    result, report = calibrate(payload)
    np.testing.assert_allclose(result, X, atol=1e-7)
    assert report['accepted']
    assert set(report['train_indices']).isdisjoint(report['validation_indices'])


def test_holdout_rejects_bad_observation():
    payload, _ = synthetic()
    payload['samples'][3]['camera_tag'][0][3] += 0.3
    _, report = calibrate(payload)
    assert not report['accepted']
    assert report['validation']['translation_max_m'] > .1


def test_noisy_data():
    payload, X = synthetic()
    rng = np.random.default_rng(12)
    for sample in payload['samples']:
        C = np.asarray(sample['camera_tag'])
        C[:3, 3] += rng.normal(0, .0005, 3)
        C[:3, :3] = C[:3, :3] @ Rotation.from_rotvec(rng.normal(0, .0005, 3)).as_matrix()
        sample['camera_tag'] = C.tolist()
    result, report = calibrate(payload)
    assert report['accepted']
    assert np.linalg.norm(result[:3, 3] - X[:3, 3]) < .005


def test_single_axis_and_identical_poses_fail():
    for poses in [[np.eye(4)]*12, [make_transform(Rotation.from_euler('z', a).as_matrix(), [0, 0, 0])
                                  for a in np.linspace(-1, 1, 12)]]:
        with pytest.raises(ValueError, match='退化'):
            excitation(poses)


def test_invalid_inputs():
    with pytest.raises(ValueError):
        validate_transform(np.zeros((4, 4)))
    with pytest.raises(ValueError):
        transform_from_dict({'translation': [0, 0, 0], 'quaternion': [0]*4})
    with pytest.raises(ValueError):
        calibrate({'schema_version': 1, 'samples': []})
    for angle in [0, np.pi, -np.pi, .7]:
        T = make_transform(Rotation.from_euler('x', angle).as_matrix(), [.1, .2, .3])
        np.testing.assert_allclose(transform_from_dict(transform_to_dict(T)), T, atol=1e-12)


def test_fk_model_frames_missing_joint_and_alias():
    fk = Kinematics(MODEL, 'base_link', 'Head_J2_Link', {'Head_J2_Joint': 'Head_J2_joint'})
    assert len(fk.required) == 8
    with pytest.raises(ValueError, match='缺少关节'):
        fk.pose({'Head_J1_joint': 0.0})
    joints = {fk.model.names[j]: 0.0 for j in fk.required}
    joints['Head_J2_joint'] = joints.pop('Head_J2_Joint')
    T = fk.pose(joints)
    # base_link 包含基座到骨盆的固定高度及六个腿部关节。
    np.testing.assert_allclose(T[:3, 3], [-.01349145, 0, 1.36398005], atol=1e-5)
    for mount in ['Arm_L7_Link', 'Arm_R7_Link']:
        assert len(Kinematics(MODEL, 'base_link', mount).required) == 13
    with pytest.raises(ValueError, match='缺少 frame'):
        Kinematics(MODEL, 'missing', 'Head_J2_Link')


def test_stationary_sync_and_motion_rejection():
    history = [(float(t), {'q': 0.1}) for t in np.arange(.5, 1.2, .01)]
    positions = lambda q: [q['q']]
    joints, delta = match_stationary(history, 1., positions)
    assert delta < .01 and joints['q'] == .1
    with pytest.raises(ValueError):
        match_stationary(history, 3., positions)
    with pytest.raises(ValueError):
        match_stationary(history[:20], 1., positions)
    moving = [(t, {'q': t}) for t, _ in history]
    with pytest.raises(ValueError, match='停稳'):
        match_stationary(moving, 1., positions)


def test_apriltag_pnp_projection_direction():
    detector = AprilTagDetector(0, .08)
    K = np.array([[600., 0, 320], [0, 600, 240], [0, 0, 1]])
    R = Rotation.from_euler('xyz', [2.8, .25, .1]).as_matrix()
    t = np.array([.04, .01, .6])
    pixels = cv2.projectPoints(detector.points, cv2.Rodrigues(R)[0], t, K, np.zeros(5))[0].reshape(4, 2)
    result, error, _ = detector.pose(pixels, K, np.zeros(5))
    np.testing.assert_allclose(result, make_transform(R, t), atol=1e-7)
    assert error < 1e-6


def test_apriltag_pnp_supports_realsense_inverse_brown_conrady():
    detector = AprilTagDetector(0, .08)
    K = np.array([[394.077453613281, 0, 319.891723632812],
                  [0, 393.114654541016, 232.059906005859], [0, 0, 1.]])
    D = np.array([-.0516790971159935, .0563772432506084,
                  .000254452315857634, .000547626637853682,
                  -.0189068876206875])
    R = Rotation.from_euler('xyz', [2.8, .25, .1]).as_matrix()
    t = np.array([.04, .01, .6])
    ideal = cv2.projectPoints(detector.points, cv2.Rodrigues(R)[0], t, K, np.zeros(5))[0]
    ideal = ideal.reshape(-1, 2)
    # 反解 SDK inverse（distorted -> undistorted）得到应传入检测器的原始像素。
    raw = ideal.copy()
    for _ in range(30):
        corrected = detector._inverse_brown_pixels(raw, K, D)
        raw += ideal - corrected
    result, error, _ = detector.pose(raw, K, D, 'inverse_brown_conrady')
    np.testing.assert_allclose(result, make_transform(R, t), atol=1e-6)
    assert error < 1e-6


def test_apriltag_image_detection_and_wrong_id():
    detector = AprilTagDetector(0, .08)
    dictionary = detector.dictionary
    tag = (cv2.aruco.generateImageMarker(dictionary, 0, 200) if hasattr(cv2.aruco, 'generateImageMarker')
           else cv2.aruco.drawMarker(dictionary, 0, 200))
    src = np.array([[0, 0], [199, 0], [199, 199], [0, 199]], np.float32)
    dst = np.array([[210, 160], [400, 165], [390, 340], [225, 320]], np.float32)
    image = cv2.warpPerspective(tag, cv2.getPerspectiveTransform(src, dst), (640, 480), borderValue=255)
    K = np.array([[600., 0, 320], [0, 600, 240], [0, 0, 1]])
    # 此图只验证检测；透视四边形不要求精确匹配上述虚构内参。
    detector.pose = lambda pixels, k, d: pixels
    assert detector.detect(image, K, np.zeros(5)).shape == (4, 2)
    detector.tag_id = 1
    with pytest.raises(ValueError):
        detector.detect(image, K, np.zeros(5))


def test_solver_cli_outputs_and_no_overwrite(tmp_path):
    payload, truth = synthetic()
    payload.update({'base_frame': 'base_link', 'mount_frame': 'Head_J2_Link',
                    'camera_frame': 'raw_optical', 'model_path': str(MODEL),
                    'model_sha256': 'test', 'joint_aliases': {}, 'tag_family': '36h11',
                    'tag_id': 0, 'tag_size_m': .08, 'timestamp_source': 'ros_host_arrival'})
    source = tmp_path / 'samples.yaml'
    source.write_text(yaml.safe_dump(payload))
    output = tmp_path / 'result.yaml'
    cmd = [sys.executable, '-m', 'luna_handeye_calibration.solve_handeye', str(source),
           '--output', str(output), '--child-frame', 'test_calibrated_optical']
    env = dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[1]))
    first = subprocess.run(cmd, env=env, capture_output=True, text=True)
    assert first.returncode == 0, first.stderr
    data = yaml.safe_load(output.read_text())
    np.testing.assert_allclose(transform_from_dict(data['T_mount_camera']), truth, atol=1e-7)
    assert data['validation_passed']
    assert data['timestamp_source'] == 'ros_host_arrival'
    saved = output.read_bytes()
    second = subprocess.run(cmd, env=env, capture_output=True, text=True)
    assert second.returncode != 0
    assert output.read_bytes() == saved
