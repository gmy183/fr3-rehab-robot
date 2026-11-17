"""Pure OpenCV helpers for FR3+D435i ArUco hand-eye calibration.

Target: OpenCV ArUco DICT_6X6_50, marker ID 0.
MARKER_SIZE_M is the full outside edge length of the printed marker, including
its black border. The generated A4 target uses a 75.0 mm outside edge.
"""
from __future__ import annotations

import math
import numpy as np
import cv2

MARKER_DICTIONARY_NAME = "DICT_6X6_50"
MARKER_ID = 0
MARKER_SIZE_M = 0.075


def transform_matrix(t_xyz, q_xyzw):
    q = np.asarray(q_xyzw, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid quaternion")
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


def _dictionary():
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "cv2.aruco is unavailable. Install an OpenCV build with contrib/aruco support."
        )
    aruco = cv2.aruco
    dictionary_id = getattr(aruco, MARKER_DICTIONARY_NAME, None)
    if dictionary_id is None:
        raise RuntimeError(f"OpenCV does not provide {MARKER_DICTIONARY_NAME}")
    return aruco.getPredefinedDictionary(dictionary_id)


def detect_marker_corners(image_bgr):
    """Return the refined four corners for the configured marker, or None.

    The expected target is DICT_6X6_50 / ID 0. Other IDs are deliberately
    rejected so an arbitrary square cannot be accepted as the calibration board.
    Returned corners have shape (4, 1, 2) and ArUco's canonical corner order.
    """
    if image_bgr is None or image_bgr.size == 0:
        return None
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    aruco = cv2.aruco
    dictionary = _dictionary()
    # Use OpenCV's default detector parameters. On the deployed Ubuntu/ROS
    # image (OpenCV 4.6.0), reading the wrapped `cornerRefinementMethod`
    # property via `hasattr(params, ...)` can segfault in the native binding.
    # Do not introspect or mutate that property; marker detection supports the
    # default parameters in both the newer and legacy ArUco APIs.
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary)
        corners_list, ids, _rejected = detector.detectMarkers(gray)
    else:  # OpenCV 4.6 / older ROS binary compatibility
        corners_list, ids, _rejected = aruco.detectMarkers(gray, dictionary)

    if ids is None or len(corners_list) == 0:
        return None
    ids = np.asarray(ids).reshape(-1)
    matches = [i for i, marker_id in enumerate(ids) if int(marker_id) == MARKER_ID]
    if not matches:
        return None

    # If a printed copy appears more than once, use the largest valid instance.
    index = max(
        matches,
        key=lambda i: abs(float(cv2.contourArea(
            np.asarray(corners_list[i], dtype=np.float32).reshape(4, 2)
        )))
    )
    corners = np.asarray(corners_list[index], dtype=np.float32).reshape(4, 1, 2)
    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        20,
        0.01,
    )
    try:
        corners = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), criteria)
    except cv2.error:
        # The detector's own subpixel refinement is still usable.
        pass
    return corners.astype(np.float32)


def extract_board(image_bgr, K, dist, marker_size_m=MARKER_SIZE_M):
    """Return (camera_T_marker, reprojection_RMSE_px, corners) or None.

    The pose frame is centred on the marker. marker_size_m is the full outer
    edge length, including the black border.
    """
    if not np.isfinite(float(marker_size_m)) or float(marker_size_m) <= 0:
        raise ValueError("marker_size_m must be finite and positive")
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    D = np.asarray(dist, dtype=np.float64).reshape(-1)
    if not (np.isfinite(K).all() and np.isfinite(D).all()
            and K[0, 0] > 0 and K[1, 1] > 0):
        raise ValueError("invalid camera intrinsics")

    corners = detect_marker_corners(image_bgr)
    if corners is None:
        return None

    half = float(marker_size_m) / 2.0
    # Required ordering for SOLVEPNP_IPPE_SQUARE and ArUco corners:
    # top-left, top-right, bottom-right, bottom-left; origin at marker centre.
    object_points = np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float32)
    image_points = corners.reshape(4, 2).astype(np.float32)

    ok, rvec, tvec = cv2.solvePnP(
        object_points, image_points, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(
            object_points, image_points, K, D, flags=cv2.SOLVEPNP_ITERATIVE
        )
    if not ok:
        return None

    projected, _ = cv2.projectPoints(object_points, rvec, tvec, K, D)
    residuals = projected.reshape(-1, 2) - image_points
    rmse = float(np.sqrt(np.mean(np.sum(residuals ** 2, axis=1))))
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(3)
    if not (np.isfinite(T).all() and math.isfinite(rmse)):
        return None
    if T[2, 3] <= 0:
        return None
    return T, rmse, corners
