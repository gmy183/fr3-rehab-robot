"""Numerical helpers, independent from ROS and robot hardware."""
from __future__ import annotations

import math
import numpy as np


def transform_matrix(t_xyz, q_xyzw):
    q = np.asarray(q_xyzw, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('invalid quaternion')
    x, y, z, w = q / norm
    R = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=float)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t_xyz, dtype=float)
    return T


def relative_pose(T0, T1):
    A = np.asarray(T0, dtype=float)
    B = np.asarray(T1, dtype=float)
    rel = np.linalg.inv(A) @ B
    d = float(np.linalg.norm(rel[:3, 3]))
    c = float(np.clip((np.trace(rel[:3, :3]) - 1) / 2, -1, 1))
    r_deg = math.degrees(math.acos(c))
    return d, r_deg


def image_stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


def extract_board(image_bgr, K, dist, board=(11, 8), square_m=.025):
    """Returns candidate (T, rmse, corners) or None. Image must be BGR."""
    import cv2
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray, board,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
    )

    if found and corners is not None and len(corners) == board[0] * board[1]:
        criteria = (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
            30, 0.01
        )
        corners = cv2.cornerSubPix(
            gray, corners, (7, 7), (-1, -1), criteria
        )
    else:
        found, corners = cv2.findChessboardCornersSB(
            gray,
            board,
            flags=cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if not found or corners is None or len(corners) != board[0] * board[1]:
            return None

        corners = np.asarray(
            corners, dtype=np.float32
        ).reshape(-1, 1, 2)
    obj = np.zeros((board[0] * board[1], 3), np.float32)
    obj[:, :2] = np.mgrid[0:board[0], 0:board[1]].T.reshape(-1, 2) * square_m
    ok, rvec, tvec = cv2.solvePnP(obj, corners, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    residuals = projected.reshape(-1, 2) - corners.reshape(-1, 2)
    rmse = float(np.sqrt(np.mean(np.sum(residuals ** 2, axis=1))))
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.ravel()
    return T, rmse, corners
