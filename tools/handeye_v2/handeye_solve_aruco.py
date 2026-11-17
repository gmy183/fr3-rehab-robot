#!/usr/bin/env python3
"""Offline eye-in-hand ArUco solver for FR3/D435i v2.4 candidate JSON samples.

No ROS imports, hardware access, or robot motion. Computes flange_T_camera.
Inputs: base_T_gripper (gripper->base) and camera_T_marker (marker->camera).
Target is immobile within ONE dataset. Never merge datasets across marker moves.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np
import cv2

METHODS = {
    'PARK': cv2.CALIB_HAND_EYE_PARK,
    'TSAI': cv2.CALIB_HAND_EYE_TSAI,
    'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
    'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
    'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
}

def angle_deg(R):
    v = (float(np.trace(R)) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(v, -1.0, 1.0))))

def validate_T(m, name):
    T = np.asarray(m, dtype=np.float64)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError(f'{name}: not finite 4x4')
    if not np.allclose(T[3, :], [0., 0., 0., 1.], atol=1e-6):
        raise ValueError(f'{name}: invalid bottom row')
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=2e-3) or abs(np.linalg.det(R) - 1) > 2e-3:
        raise ValueError(f'{name}: invalid rotation')
    return T

def load_data(folder):
    samples = []
    paths = sorted(folder.glob('sample_*.json'))
    if not paths:
        raise ValueError(f'No sample_*.json in {folder}')
    for path in paths:
        x = json.loads(path.read_text(encoding='utf-8'))
        needed = {
            'schema':'fr3_handeye_v2', 'status':'candidate',
            'calibration_target':'aruco', 'marker_dictionary':'DICT_6X6_50',
            'marker_id':0, 'base_frame':'fr3_link0', 'flange_frame':'fr3_link8',
            'camera_frame':'camera_color_optical_frame',
        }
        for k, expected in needed.items():
            if x.get(k) != expected:
                raise ValueError(f'{path.name}: {k} expected {expected!r}, got {x.get(k)!r}')
        if abs(float(x.get('marker_size_m', -1)) - .075) > 1e-6:
            raise ValueError(f'{path.name}: marker_size_m must be 0.075')
        B_G = validate_T(x['base_T_gripper'], f'{path.name} base_T_gripper')
        C_M = validate_T(x.get('camera_T_marker', x.get('camera_T_board')), f'{path.name} camera_T_marker')
        rmse = float(x['reprojection_rmse_px'])
        if not math.isfinite(rmse) or rmse > 2.0 or rmse < 0:
            raise ValueError(f'{path.name}: invalid ArUco RMSE {rmse}')
        if C_M[2, 3] <= 0:
            raise ValueError(f'{path.name}: marker behind camera')
        samples.append((path.name, B_G, C_M, rmse))
    return samples

def excitation(samples):
    rotations = []
    translations = []
    axes_vectors = []
    for i in range(len(samples)):
        for j in range(i+1,len(samples)):
            A,B=samples[i][1],samples[j][1]
            dr=A[:3,:3].T @ B[:3,:3]
            rvec,_=cv2.Rodrigues(dr)
            rot=float(np.linalg.norm(rvec))
            rotations.append(math.degrees(rot))
            translations.append(float(np.linalg.norm(A[:3,3]-B[:3,3])*1000))
            axes_vectors.append(rvec.reshape(3))
    sv=np.linalg.svd(np.array(axes_vectors),compute_uv=False) if axes_vectors else np.zeros(3)
    return {
        'max_relative_rotation_deg':float(max(rotations,default=0)),
        'median_relative_rotation_deg':float(np.median(rotations)) if rotations else 0.,
        'max_translation_spread_mm':float(max(translations,default=0)),
        'rotation_axis_singular_values':list(map(float,sv)),
        'rotation_axis_min_over_max':float(sv[-1]/sv[0]) if len(sv)>=3 and sv[0]>1e-12 else 0.,
    }

def residual(samples, X):
    boards = [G@X@C for _,G,C,_ in samples]
    # Medoid avoids averaging of angles/quaternions; reference is same measured board pose.
    costs=[]
    for ref in boards:
        d=[np.linalg.norm(ref[:3,3]-b[:3,3])*1000+2*angle_deg(ref[:3,:3].T@b[:3,:3]) for b in boards]
        costs.append(float(np.median(d)))
    ref=boards[int(np.argmin(costs))]
    translation=np.array([np.linalg.norm(ref[:3,3]-b[:3,3])*1000 for b in boards])
    rotation=np.array([angle_deg(ref[:3,:3].T@b[:3,:3]) for b in boards])
    return {
        'median_translation_mm':float(np.median(translation)),
        'p95_translation_mm':float(np.percentile(translation,95)),
        'median_rotation_deg':float(np.median(rotation)),
        'p95_rotation_deg':float(np.percentile(rotation,95)),
        'per_sample':[
            {'name':s[0],'translation_mm':float(t),'rotation_deg':float(r),'reprojection_rmse_px':s[3]}
            for s,t,r in zip(samples,translation,rotation)
        ],
    }

def solve(samples):
    Rg=[s[1][:3,:3] for s in samples]
    tg=[s[1][:3,3].reshape(3,1) for s in samples]
    Rt=[s[2][:3,:3] for s in samples]
    tt=[s[2][:3,3].reshape(3,1) for s in samples]
    outputs={}
    for name,m in METHODS.items():
        try:
            R,t=cv2.calibrateHandEye(Rg,tg,Rt,tt,method=m)
            X=np.eye(4)
            X[:3,:3]=np.asarray(R,dtype=float).reshape(3,3)
            X[:3,3]=np.asarray(t,dtype=float).reshape(3)
            validate_T(X,f'{name} result')
            rs=residual(samples,X)
            outputs[name]={'flange_T_camera':X.tolist(),'residual':rs}
        except (cv2.error, ValueError, FloatingPointError) as exc:
            outputs[name]={'error':str(exc)}
    good={name:x for name,x in outputs.items() if 'error' not in x}
    if not good:
        raise ValueError(f'All hand-eye methods failed: {outputs}')
    # Residual-based method ranking is descriptive, not an independent accuracy guarantee.
    best=min(good, key=lambda name:(good[name]['residual']['median_translation_mm'] +
                                  2*good[name]['residual']['median_rotation_deg']))
    return outputs,best

def main():
    parser=argparse.ArgumentParser(description='Offline FR3 D435i ArUco eye-in-hand solver')
    parser.add_argument('directory',type=Path,help='One capture run directory (marker fixed)')
    parser.add_argument('--min-samples',type=int,default=10)
    parser.add_argument('--output',type=Path,default=None)
    args=parser.parse_args()
    folder=args.directory.expanduser().resolve()
    if not folder.is_dir():
        parser.error(f'No directory: {folder}')
    samples=load_data(folder)
    exc=excitation(samples)
    print(f'[AUDIT] samples={len(samples)} | marker=DICT_6X6_50 ID=0 size=75mm')
    print(f'[EXCITATION] max_rotation={exc["max_relative_rotation_deg"]:.2f}deg, '
          f'median_rotation={exc["median_relative_rotation_deg"]:.2f}deg, '
          f'axes_min/max={exc["rotation_axis_min_over_max"]:.3f}')
    if len(samples)<args.min_samples:
        print(f'[NOT READY] Need at least {args.min_samples} in one fixed-target run; no matrix written.')
        return 2
    warnings=[]
    if exc['max_relative_rotation_deg']<8:
        warnings.append('Weak rotational excitation (<8deg max); result may be unstable')
    if exc['rotation_axis_min_over_max']<.03:
        warnings.append('Poor diversity of rotational axes; result may be degenerate')
    outputs,best=solve(samples)
    sol=outputs[best]
    r=sol['residual']
    if r['p95_translation_mm']>10 or r['p95_rotation_deg']>5:
        warnings.append('Large target-consistency residual; inspect sample quality')
    status='CANDIDATE_ONLY' if warnings else 'PRELIMINARY_VALIDATION_REQUIRED'
    result={
        'schema':'fr3_d435i_handeye_aruco_offline_v1',
        'source_directory':str(folder),'source_sample_count':len(samples),
        'frame_convention':'flange_T_camera = fr3_link8_T_camera_color_optical_frame',
        'matrix_definition':'base_T_marker_i = base_T_gripper_i @ flange_T_camera @ camera_T_marker_i',
        'method_selected_by_training_residual':best,
        'flange_T_camera':sol['flange_T_camera'],
        'translation_m':np.array(sol['flange_T_camera'])[:3,3].tolist(),
        'excitation':exc,'consistency':r,'all_methods':outputs,
        'quality_status':status,'warnings':warnings,
        'notice':'Preliminary internal consistency only; independent held-out validation required.'
    }
    path=args.output or folder/'handeye_aruco_candidate.json'
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    print(f'[SOLVED] method={best}, status={status}')
    print(f'[CONSISTENCY] median={r["median_translation_mm"]:.2f}mm/{r["median_rotation_deg"]:.2f}deg '
          f'p95={r["p95_translation_mm"]:.2f}mm/{r["p95_rotation_deg"]:.2f}deg')
    for msg in warnings:
        print('[WARN]',msg)
    print('[OUTPUT]',path)
    print('[IMPORTANT] Candidate calibration is NOT automatically deployed to TF or controllers.')
    return 0

if __name__=='__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print(f'[ERROR] {type(exc).__name__}: {exc}', file=sys.stderr)
        sys.exit(2)
