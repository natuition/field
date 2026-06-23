import json
from pathlib import Path
from typing import Optional, Sequence, Tuple, Union

import cv2
import numpy as np


class CalibrationError(RuntimeError):
    """Raised when the calibration file is not usable."""


PathLike = Union[str, Path]


def read_json(path: PathLike):
    with Path(path).open("r", encoding="utf-8") as handle:
        return json.load(handle)


def raw_pixel_to_robot_mm(
    u: float,
    v: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    homography: np.ndarray,
) -> Tuple[float, float]:
    """
    Convertit un pixel brut caméra vers une position robot en mm.

    La correction de distorsion est appliquée avant l'homographie.
    """
    point_raw = np.array([[[u, v]]], dtype=np.float64)

    point_undistorted = cv2.undistortPoints(
        point_raw,
        camera_matrix,
        dist_coeffs,
        P=camera_matrix,
    )

    point_mm = cv2.perspectiveTransform(point_undistorted, homography)

    x_mm = float(point_mm[0, 0, 0])
    y_mm = float(point_mm[0, 0, 1])

    return x_mm, y_mm


def raw_pixel_to_robot_mm_from_json(
    u: float,
    v: float,
    calibration_robot_json_path: PathLike,
    offset_override_mm: Optional[Sequence[float]] = None,
) -> Tuple[float, float]:
    """
    Convertit un pixel brut caméra vers une position robot en mm
    à partir d'un fichier JSON de calibration robot.

    Le JSON doit contenir :
    - camera_matrix
    - dist_coeffs
    - homography_undistorted_px_to_robot_mm
    - origin_offset_mm, optionnel
    """
    payload = read_json(calibration_robot_json_path)

    camera_matrix = np.asarray(payload["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.asarray(payload["dist_coeffs"], dtype=np.float64)
    homography = np.asarray(
        payload["homography_undistorted_px_to_robot_mm"],
        dtype=np.float64,
    )

    x_mm, y_mm = raw_pixel_to_robot_mm(
        u,
        v,
        camera_matrix,
        dist_coeffs,
        homography,
    )

    if offset_override_mm is not None:
        offset_mm = offset_override_mm
    else:
        offset_mm = payload.get("origin_offset_mm", [0.0, 0.0])

    if len(offset_mm) != 2:
        raise CalibrationError(
            "origin_offset_mm doit contenir exactement 2 valeurs "
            "[offset_x_mm, offset_y_mm]"
        )

    x_mm += float(offset_mm[0])
    y_mm += float(offset_mm[1])

    return x_mm, y_mm


def load_homography(calibration_json: PathLike) -> Tuple[np.ndarray, int, int]:
    """
    Charge l'homographie pixels bruts -> mm relatifs au centre image.

    Le JSON doit contenir :
    - homography_raw_px_to_image_center_mm
    - image_width
    - image_height
    """
    payload = read_json(calibration_json)

    homography = np.asarray(
        payload["homography_raw_px_to_image_center_mm"],
        dtype=np.float64,
    )

    image_width = int(payload["image_width"])
    image_height = int(payload["image_height"])

    return homography, image_width, image_height


def raw_pixel_to_image_center_mm(
    u: float,
    v: float,
    homography: np.ndarray,
) -> Tuple[float, float]:
    """
    Convertit un pixel brut (u, v) vers une position au sol en mm.

    L'origine (0, 0) correspond au centre géométrique de l'image.
    """
    point_px = np.array([[[u, v]]], dtype=np.float64)

    point_mm = cv2.perspectiveTransform(point_px, homography)

    x_mm = float(point_mm[0, 0, 0])
    y_mm = float(point_mm[0, 0, 1])

    return x_mm, y_mm