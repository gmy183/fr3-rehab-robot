#!/usr/bin/env python3
"""Offline hand-eye solve with internal static-board consistency checks.

No ROS, no hardware motion, no TF publishing. Result is a candidate, not validated.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
import cv2
from handeye_math import relative_pose

METHODS = {
    'TSAI': cv2.CALIB_HAND_EYE_TSAI,
    'PARK': cv2.CALIB_HAND_EYE_PARK,
    'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
    'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
    'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def valid_transform(T):
    T = np.asarray(T, dtype=float)
    return (T.shape == (4, 4) and np.isfinite(T).all()
            and np.allclose(T[3], [0, 0, 0, 1], atol=1e-6)
            and np.allclose(T[:3, :3] @ T[:3, :3].T, np.eye(3), atol=1e-2)
            and abs(np.linalg.det(T[:3, :3]) - 1) < .02)


def load_samples(directory):
    data = []
    for path in sorted(directory.glob('sample_*.json')):
        item = json.loads(path.read_text(encoding='utf-8'))
        if item.get('schema') != 'fr3_handeye_v2' or item.get('status') != 'candidate':
            raise ValueError(f'{path}: not V2 candidate data')
        B_G = np.asarray(item['base_T_gripper'], dtype=float)
        C_T = np.asarray(item['camera_T_board'], dtype=float)
        if not valid_transform(B_G) or not valid_transform(C_T):
            raise ValueError(f'{path}: invalid homogeneous matrix')
        if item.get('flange_frame') != 'fr3_link8' or item.get('base_frame') != 'fr3_link0':
            raise ValueError(f'{path}: incompatible robot frames')
        if item.get('board_inner_corners') != [11, 8] or item.get('square_size_m') != .025:
            raise ValueError(f'{path}: incompatible board model')
        if float(item['reprojection_rmse_px']) > .8:
            raise ValueError(f'{path}: checkerboard RMSE outside gate')
        data.append((path.name, B_G, C_T))
    return data


def analyze(data, X):
    boards = [G @ X @ C for _, G, C in data]
    # Use a medoid reference to avoid quaternion averaging ambiguities.
    total_errors = []
    for candidate in boards:
        ds, rs = zip(*(relative_pose(candidate, b) for b in boards))
        total_errors.append(float(np.median(ds)) * 1000 + 2 * float(np.median(rs)))
    ref = boards[int(np.argmin(total_errors))]
    dist = np.array([relative_pose(ref, p) for p in boards])
    return {
        'median_translation_mm': float(np.median(dist[:, 0])) * 1000,
        'p95_translation_mm': float(np.percentile(dist[:, 0], 95)) * 1000,
        'median_rotation_deg': float(np.median(dist[:, 1])),
        'p95_rotation_deg': float(np.percentile(dist[:, 1], 95)),
        'heuristic_score': float(np.median(dist[:, 0])) * 1000 + 2 * float(np.median(dist[:, 1])),
    }


def main():
    ap = argparse.ArgumentParser(description='Offline solve only; no hardware I/O')
    ap.add_argument('directory', type=Path)
    args = ap.parse_args()
    if not args.directory.is_dir():
        raise SystemExit(f'not a directory: {args.directory}')
    data = load_samples(args.directory)
    if len(data) < 15:
        raise SystemExit(f'[REJECT] Need >=15 pose-diverse v2 samples; got {len(data)}')
    all_rot = [relative_pose(data[0][1], d[1])[1] for d in data[1:]]
    max_rotation = max(all_rot) if all_rot else 0.
    if max_rotation < 15.:
        raise SystemExit(f'[REJECT] Insufficient robot rotation diversity ({max_rotation:.1f}deg < 15deg)')
    gripper2base = [b for _, b, _ in data]
    target2cam = [c for _, _, c in data]
    results = {}
    for method, code in METHODS.items():
        try:
            R, t = cv2.calibrateHandEye(
                [p[:3, :3] for p in gripper2base],
                [p[:3, 3] for p in gripper2base],
                [p[:3, :3] for p in target2cam],
                [p[:3, 3] for p in target2cam],
                method=code,
            )
            X = np.eye(4)
            X[:3, :3], X[:3, 3] = R, np.asarray(t).reshape(3)
            if not valid_transform(X):
                raise ValueError('invalid computed transform')
            metrics = analyze(data, X)
            results[method] = {'gripper_T_camera': X.tolist(), 'internal_consistency': metrics}
            print(f'{method}: median {metrics["median_translation_mm"]:.2f}mm, '
                  f'{metrics["median_rotation_deg"]:.2f}deg (internal only)')
        except (cv2.error, ValueError, np.linalg.LinAlgError) as exc:
            results[method] = {'error': str(exc)}
            print('[WARN]', method, str(exc))
    ok = [(name, v) for name, v in results.items() if 'internal_consistency' in v]
    if not ok:
        raise SystemExit('[REJECT] no valid hand-eye solution')
    best, record = min(ok, key=lambda x: x[1]['internal_consistency']['heuristic_score'])
    output = {
        'status': 'CANDIDATE_NOT_VALIDATED',
        'warning': 'Internal fixed-board consistency is NOT an independent validation or motion safety clearance.',
        'base_frame': 'fr3_link0', 'gripper_frame': 'fr3_link8',
        'board': '11x8 inner corners, 0.025m square',
        'samples': [name for name, _, _ in data],
        'max_base_gripper_rotation_from_first_deg': max_rotation,
        'best_by_heuristic': best,
        'solutions': results,
    }
    path = args.directory / 'handeye_candidate_report_v2.json'
    if path.exists():
        raise SystemExit(f'[STOP] {path} already exists: refusing to overwrite')
    with path.open('x', encoding='utf-8') as f:
        json.dump(output, f, indent=2, ensure_ascii=False)
    print('[SAVED]', path)
    print('[NOT VALIDATED] Do not publish camera TF or start patient-contact motion from this candidate.')


if __name__ == '__main__':
    main()
