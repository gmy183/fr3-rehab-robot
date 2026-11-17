#!/usr/bin/env python3
"""FR3+D435i hand-eye *candidate* verification and TF preview; never drives robot."""
import argparse
import json
import math
import shlex
import sys
from pathlib import Path

import numpy as np


def transform(value, name):
    T = np.asarray(value, dtype=float)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise ValueError(f'{name} must be a finite 4x4 matrix')
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6):
        raise ValueError(f'{name}: invalid homogeneous last row')
    R = T[:3, :3]
    if np.linalg.norm(R.T @ R - np.eye(3)) > .01 or abs(np.linalg.det(R) - 1) > .01:
        raise ValueError(f'{name}: rotation is not in SO(3)')
    return T


def read_json(p):
    with open(p, encoding='utf-8') as f:
        return json.load(f)


def so3_angle(A, B):
    d = A.T @ B
    return math.degrees(math.acos(float(np.clip((np.trace(d) - 1) / 2, -1, 1))))


def load_samples(folder):
    found = []
    for path in sorted(folder.glob('sample_*.json')):
        s = read_json(path)
        if s.get('calibration_target') != 'aruco' or s.get('marker_dictionary') != 'DICT_6X6_50' or s.get('marker_id') != 0 or abs(float(s.get('marker_size_m', -1)) - .075) > 1e-5:
            raise ValueError(f'{path}: marker mismatch; require DICT_6X6_50/ID0/75mm')
        if s.get('base_frame') != 'fr3_link0' or s.get('flange_frame') != 'fr3_link8' or s.get('camera_frame') != 'camera_color_optical_frame':
            raise ValueError(f'{path}: coordinate frames do not match calibrated geometry')
        A = transform(s['base_T_gripper'], f'{path}:base_T_gripper')
        B = transform(s['camera_T_marker'], f'{path}:camera_T_marker')
        found.append((path.name, A, B, s))
    if not found:
        raise ValueError(f'No sample_*.json in {folder}')
    return found


def marker_estimates(samples, handeye):
    return [(name, A @ handeye @ B) for name, A, B, _ in samples]


def positional_stats(poses, reference_mm):
    pts = np.array([T[:3, 3] * 1000 for _, T in poses])
    dist = np.linalg.norm(pts - reference_mm, axis=1)
    return pts, dist


def unit_quat(R):
    # q as [x, y, z, w], Eigen-like stable branches
    m = np.asarray(R, dtype=float)
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(1 + t) * 2
        qw = .25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s = math.sqrt(max(0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) * 2
            qw = (m[2, 1] - m[1, 2]) / s
            qx = .25 * s
            qy = (m[0, 1] + m[1, 0]) / s
            qz = (m[0, 2] + m[2, 0]) / s
        elif i == 1:
            s = math.sqrt(max(0, 1 + m[1, 1] - m[0, 0] - m[2, 2])) * 2
            qw = (m[0, 2] - m[2, 0]) / s
            qx = (m[0, 1] + m[1, 0]) / s
            qy = .25 * s
            qz = (m[1, 2] + m[2, 1]) / s
        else:
            s = math.sqrt(max(0, 1 + m[2, 2] - m[0, 0] - m[1, 1])) * 2
            qw = (m[1, 0] - m[0, 1]) / s
            qx = (m[0, 2] + m[2, 0]) / s
            qy = (m[1, 2] + m[2, 1]) / s
            qz = .25 * s
    q = np.array([qx, qy, qz, qw]); q /= np.linalg.norm(q)
    if q[3] < 0: q *= -1
    return q


def report_stats(title, poses, center_mm, orient_ref=None):
    pts, deviations = positional_stats(poses, center_mm)
    print(f'\n=== {title} ({len(poses)} samples) ===')
    print('reference marker center [mm]:', np.round(center_mm, 3).tolist())
    print('measured mean [mm]:', np.round(pts.mean(axis=0), 3).tolist())
    print('position mean/median/max [mm]:', *(f'{x:.3f}' for x in [deviations.mean(), np.median(deviations), deviations.max()]))
    print('XYZ standard deviation [mm]:', np.round(pts.std(axis=0), 3).tolist())
    angular = []
    for (name, T), deviation in zip(poses, deviations):
        ang = so3_angle(orient_ref, T[:3, :3]) if orient_ref is not None else None
        angular.append(ang)
        print(f'{name:20s} p={np.round(T[:3,3]*1000,2).tolist()} mm | position_error={deviation:.2f}mm' + (f' | rotation_deviation={ang:.2f}deg' if ang is not None else ''))
    if angular and angular[0] is not None:
        print(f'rotation median/max [deg]: {np.median(angular):.3f}/{max(angular):.3f}')
    return {'sample_count': len(poses), 'reference_center_mm': center_mm.tolist(), 'mean_error_mm': float(deviations.mean()), 'median_error_mm': float(np.median(deviations)), 'max_error_mm': float(deviations.max()), 'per_sample_error_mm': deviations.tolist(), 'xyz_std_mm': pts.std(axis=0).tolist(), 'rotation_deviation_deg': angular}


def main():
    p = argparse.ArgumentParser(description='Offline FR3+D435i ArUco candidate calibration workflow; NO robot commands.')
    p.add_argument('--train', type=Path, required=True, help='The completed 12-pose calibration directory')
    p.add_argument('--validation', type=Path, help='Separate directory of new observations; must NOT have been used in solver')
    p.add_argument('--camera-point', type=float, nargs=3, metavar=('X', 'Y', 'Z'), help='Camera optical-frame point in metres; transform using the first validation sample')
    p.add_argument('--tf-preview', action='store_true', help='Print ROS 2 static TF preview command; alias frame, never publishes')
    p.add_argument('--save-report', type=Path, help='Write read-only audit summary to JSON')
    args = p.parse_args()
    train = args.train.resolve()
    result_file = train / 'handeye_aruco_candidate.json'
    if not result_file.exists():
        p.error(f'Missing {result_file}')
    result = read_json(result_file)
    if result.get('frame_convention') != 'flange_T_camera = fr3_link8_T_camera_color_optical_frame':
        raise ValueError('Hand-eye frame convention unexpected; will not guess')
    X = transform(result['flange_T_camera'], 'flange_T_camera')
    train_samples = load_samples(train)
    train_poses = marker_estimates(train_samples, X)
    train_pts = np.array([T[:3, 3]*1000 for _, T in train_poses])
    center = train_pts.mean(axis=0)
    reference_R = train_poses[0][1][:3,:3]
    print('FR3+D435i HAND-EYE FINISH / READ ONLY')
    print('STATUS:', result.get('quality_status'), '| TRAIN:', str(train))
    print('Calibrated gripper frame: fr3_link8; optical camera: camera_color_optical_frame')
    print('HORAUD candidate transform fr3_link8_T_camera_color_optical_frame:')
    print(np.array2string(X, precision=6, suppress_small=True))
    print('translation mm:', np.round(X[:3,3]*1000, 3).tolist())
    q = unit_quat(X[:3,:3]); print('quaternion [x,y,z,w]:', np.round(q, 8).tolist())
    print('Note: this is a candidate, not accuracy-certified; do not use for autonomous motion.')
    all_reports = {'quality_status': result.get('quality_status'), 'matrix_link8_T_camera': X.tolist(), 'training': report_stats('TRAIN INTERNAL CONSISTENCY (NOT INDEPENDENT)', train_poses, center, reference_R)}
    v_samples = None
    if args.validation:
        valid = args.validation.resolve()
        if valid == train:
            raise ValueError('--validation must differ from --train (training data is not a held-out test)')
        v_samples = load_samples(valid)
        val_poses = marker_estimates(v_samples, X)
        all_reports['validation'] = report_stats('NEW OBSERVATIONS vs FIXED TRAIN CENTER', val_poses, center, reference_R)
        print('WARNING: Separate files alone do not establish different robot poses or independent ground truth.')
    if args.camera_point:
        if v_samples is None:
            raise ValueError('--camera-point requires --validation; no live robot pose assumed')
        name, A, _, _ = v_samples[0]
        point_h = np.r_[args.camera_point, 1.0]
        base = A @ X @ point_h
        print(f'\n[POINT DEMO] Using {name}: camera point {args.camera_point}m -> base point {base[:3].tolist()}m')
        print('This numerical conversion is NOT robot motion permission and is not independent ground truth.')
        all_reports['camera_point_demo'] = {'source_sample': name, 'camera_m': args.camera_point, 'base_m': base[:3].tolist()}
    if args.tf_preview:
        t = X[:3,3]
        print('\n=== TF PREVIEW (DOES NOT RUN AUTOMATICALLY) ===')
        print('Candidate optical-frame *alias* is intentionally distinct from the D435i driver optical frame.')
        print('Using the existing optical frame as child may create a TF multiple-parent conflict; alias avoids it.')
        print('source ~/robot_env/rehab.sh')
        print('export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp')
        command = ['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', f'{t[0]:.9f}', '--y', f'{t[1]:.9f}', '--z', f'{t[2]:.9f}', '--qx', f'{q[0]:.9f}', '--qy', f'{q[1]:.9f}', '--qz', f'{q[2]:.9f}', '--qw', f'{q[3]:.9f}', '--frame-id', 'fr3_link8', '--child-frame-id', 'handeye_camera_color_optical_candidate']
        print(shlex.join(command))
        print('In a second terminal: ros2 run tf2_ros tf2_echo fr3_link0 handeye_camera_color_optical_candidate')
        all_reports['tf_preview'] = command
    if args.save_report:
        out = args.save_report.expanduser().resolve()
        if out == result_file or out.name.startswith('sample_'):
            raise ValueError('Report destination must not overwrite calibration inputs')
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(all_reports, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print('[REPORT]', out)
    print('\n[COMPLETE] Offline workflow processed. No robot motion, TF publishing or hardware changes.')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, OSError) as exc:
        print(f'[FAIL] {exc}', file=sys.stderr)
        sys.exit(2)
