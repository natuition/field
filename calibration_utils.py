from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

import cv2
import numpy as np


ARUCO_DICTIONARY_NAMES = {
    name: getattr(cv2.aruco, name)
    for name in dir(cv2.aruco)
    if name.startswith("DICT_")
}
IMAGE_EXTENSIONS = ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tif", "*.tiff")


@dataclass(slots=True)
class DetectionResult:
    image_path: Path
    image_size: tuple[int, int]
    marker_count: int
    charuco_count: int
    marker_corners: list[np.ndarray]
    marker_ids: np.ndarray | None
    charuco_corners: np.ndarray | None
    charuco_ids: np.ndarray | None
    debug_image: np.ndarray
    reason: str | None = None

    @property
    def is_valid(self) -> bool:
        return self.charuco_corners is not None and self.charuco_ids is not None and self.charuco_count > 0


class CalibrationError(RuntimeError):
    """Raised when the calibration dataset is not usable."""


def ensure_directory(path: str | Path) -> Path:
    directory = Path(path)
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def get_aruco_dictionary(dictionary_name: str) -> cv2.aruco.Dictionary:
    if dictionary_name not in ARUCO_DICTIONARY_NAMES:
        available = ", ".join(sorted(ARUCO_DICTIONARY_NAMES))
        raise CalibrationError(f"Dictionnaire ArUco inconnu: {dictionary_name}. Disponibles: {available}")
    return cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARY_NAMES[dictionary_name])


def create_charuco_board(
    squares_x: int,
    squares_y: int,
    square_length_mm: float,
    marker_length_mm: float,
    dictionary_name: str,
    legacy_pattern: bool = False,
) -> tuple[Any, cv2.aruco.Dictionary]:
    dictionary = get_aruco_dictionary(dictionary_name)
    try:
        board = cv2.aruco.CharucoBoard(
            (int(squares_x), int(squares_y)),
            float(square_length_mm),
            float(marker_length_mm),
            dictionary,
        )
    except AttributeError:
        board = cv2.aruco.CharucoBoard_create(
            int(squares_x),
            int(squares_y),
            float(square_length_mm),
            float(marker_length_mm),
            dictionary,
        )
    if legacy_pattern and hasattr(board, "setLegacyPattern"):
        board.setLegacyPattern(True)
    return board, dictionary


def board_corner_coordinates_mm(board: Any) -> np.ndarray:
    if hasattr(board, "getChessboardCorners"):
        corners = board.getChessboardCorners()
    else:
        corners = board.chessboardCorners
    return np.asarray(corners, dtype=np.float64)


def list_image_paths(folder: str | Path) -> list[Path]:
    root = Path(folder)
    image_paths: list[Path] = []
    for pattern in IMAGE_EXTENSIONS:
        image_paths.extend(root.glob(pattern))
    image_paths = sorted(path for path in image_paths if path.is_file())
    if not image_paths:
        raise CalibrationError(f"Aucune image trouvee dans {root}")
    return image_paths


def load_bgr_image(image_path: str | Path) -> np.ndarray:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise CalibrationError(f"Impossible de charger l'image {image_path}")
    return image


def image_size_wh(image: np.ndarray) -> tuple[int, int]:
    height, width = image.shape[:2]
    return width, height


def create_detector_parameters() -> Any:
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    return cv2.aruco.DetectorParameters_create()


def detect_markers(gray_image: np.ndarray, dictionary: cv2.aruco.Dictionary) -> tuple[list[np.ndarray], np.ndarray | None]:
    parameters = create_detector_parameters()
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        marker_corners, marker_ids, _ = detector.detectMarkers(gray_image)
    else:
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray_image, dictionary, parameters=parameters)
    return marker_corners, marker_ids


def annotate_detection(
    image: np.ndarray,
    marker_corners: list[np.ndarray],
    marker_ids: np.ndarray | None,
    charuco_corners: np.ndarray | None,
    charuco_ids: np.ndarray | None,
    label: str,
) -> np.ndarray:
    debug_image = image.copy()
    if marker_ids is not None and len(marker_ids) > 0:
        cv2.aruco.drawDetectedMarkers(debug_image, marker_corners, marker_ids)
    if charuco_ids is not None and charuco_corners is not None and len(charuco_ids) > 0:
        cv2.aruco.drawDetectedCornersCharuco(debug_image, charuco_corners, charuco_ids, (0, 255, 0))
    cv2.putText(debug_image, label, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
    return debug_image


def detect_charuco_corners(
    image_path: str | Path,
    board: Any,
    dictionary: cv2.aruco.Dictionary,
    min_charuco_corners: int,
) -> DetectionResult:
    image = load_bgr_image(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_ids = detect_markers(gray, dictionary)
    marker_count = 0 if marker_ids is None else int(len(marker_ids))

    if marker_count == 0:
        return DetectionResult(
            image_path=Path(image_path),
            image_size=image_size_wh(image),
            marker_count=0,
            charuco_count=0,
            marker_corners=marker_corners,
            marker_ids=marker_ids,
            charuco_corners=None,
            charuco_ids=None,
            debug_image=annotate_detection(image, marker_corners, marker_ids, None, None, "no markers"),
            reason="no_markers_detected",
        )

    _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners,
        marker_ids,
        gray,
        board,
    )
    charuco_count = 0 if charuco_ids is None else int(len(charuco_ids))
    reason = None if charuco_count >= min_charuco_corners else f"not_enough_charuco_corners<{min_charuco_corners}"
    label = "valid" if reason is None else reason
    return DetectionResult(
        image_path=Path(image_path),
        image_size=image_size_wh(image),
        marker_count=marker_count,
        charuco_count=charuco_count,
        marker_corners=marker_corners,
        marker_ids=marker_ids,
        charuco_corners=charuco_corners,
        charuco_ids=charuco_ids,
        debug_image=annotate_detection(image, marker_corners, marker_ids, charuco_corners, charuco_ids, label),
        reason=reason,
    )


def save_debug_image(debug_dir: str | Path, detection: DetectionResult) -> Path:
    ensure_directory(debug_dir)
    target = Path(debug_dir) / f"{detection.image_path.stem}_debug.jpg"
    cv2.imwrite(str(target), detection.debug_image)
    return target


def save_report_csv(report_path: str | Path, rows: Iterable[dict[str, Any]]) -> Path:
    rows = list(rows)
    if not rows:
        raise CalibrationError("Impossible d'ecrire un rapport vide")
    fieldnames = list(rows[0].keys())
    target = Path(report_path)
    ensure_directory(target.parent)
    with target.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return target


def numpy_to_native(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {key: numpy_to_native(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [numpy_to_native(item) for item in value]
    return value


def write_json(path: str | Path, payload: dict[str, Any]) -> Path:
    target = Path(path)
    ensure_directory(target.parent)
    with target.open("w", encoding="utf-8") as handle:
        json.dump(numpy_to_native(payload), handle, indent=2)
        handle.write("\n")
    return target


def read_json(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        return json.load(handle)


def validate_same_image_size(results: Sequence[DetectionResult]) -> tuple[int, int]:
    if not results:
        raise CalibrationError("Aucun resultat de detection fourni")
    sizes = {result.image_size for result in results}
    if len(sizes) != 1:
        raise CalibrationError(f"Les images n'ont pas toutes la meme resolution: {sorted(sizes)}")
    return next(iter(sizes))


def charuco_ids_to_world_points_mm(charuco_ids: np.ndarray, board: Any) -> np.ndarray:
    ids = np.asarray(charuco_ids, dtype=np.int32).reshape(-1)
    board_corners = board_corner_coordinates_mm(board)
    return np.asarray(board_corners[ids, :2], dtype=np.float64)


def transform_grid_to_robot(points_mm: np.ndarray, translation_mm: Sequence[float], rotation_deg: float) -> np.ndarray:
    points = np.asarray(points_mm, dtype=np.float64).reshape(-1, 2)
    if len(translation_mm) != 2:
        raise CalibrationError("La translation grille->robot doit contenir exactement 2 valeurs")
    tx, ty = float(translation_mm[0]), float(translation_mm[1])
    theta = math.radians(float(rotation_deg))
    rotation = np.array(
        [
            [math.cos(theta), -math.sin(theta)],
            [math.sin(theta), math.cos(theta)],
        ],
        dtype=np.float64,
    )
    rotated = points @ rotation.T
    translated = rotated + np.array([tx, ty], dtype=np.float64)
    return translated


def undistort_pixels_to_image_plane(
    points_px: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> np.ndarray:
    points = np.asarray(points_px, dtype=np.float64).reshape(-1, 1, 2)
    undistorted = cv2.undistortPoints(points, camera_matrix, dist_coeffs, P=camera_matrix)
    return np.asarray(undistorted, dtype=np.float64)


def raw_pixel_to_robot_mm(
    u: float,
    v: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    homography: np.ndarray,
) -> tuple[float, float]:
    point_raw = np.array([[[u, v]]], dtype=np.float64)
    point_undistorted = cv2.undistortPoints(point_raw, camera_matrix, dist_coeffs, P=camera_matrix)
    point_mm = cv2.perspectiveTransform(point_undistorted, homography)
    x_mm = float(point_mm[0, 0, 0])
    y_mm = float(point_mm[0, 0, 1])
    return x_mm, y_mm


def raw_pixel_to_robot_mm_from_json(
    u: float,
    v: float,
    calibration_robot_json_path: str | Path,
    offset_override_mm: Sequence[float] | None = None,
) -> tuple[float, float]:
    payload = read_json(calibration_robot_json_path)
    camera_matrix = np.asarray(payload["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.asarray(payload["dist_coeffs"], dtype=np.float64)
    homography = np.asarray(payload["homography_undistorted_px_to_robot_mm"], dtype=np.float64)
    x_mm, y_mm = raw_pixel_to_robot_mm(u, v, camera_matrix, dist_coeffs, homography)

    offset_mm = offset_override_mm if offset_override_mm is not None else payload.get("origin_offset_mm", [0.0, 0.0])
    if len(offset_mm) != 2:
        raise CalibrationError("origin_offset_mm doit contenir exactement 2 valeurs [offset_x_mm, offset_y_mm]")

    x_mm += float(offset_mm[0])
    y_mm += float(offset_mm[1])
    return x_mm, y_mm

def apply_homography(points_px: np.ndarray, homography: np.ndarray) -> np.ndarray:
    points = np.asarray(points_px, dtype=np.float64).reshape(-1, 1, 2)
    projected = cv2.perspectiveTransform(points, np.asarray(homography, dtype=np.float64))
    return projected.reshape(-1, 2)


def compute_mm_errors(predicted_mm: np.ndarray, expected_mm: np.ndarray) -> dict[str, float]:
    predicted = np.asarray(predicted_mm, dtype=np.float64).reshape(-1, 2)
    expected = np.asarray(expected_mm, dtype=np.float64).reshape(-1, 2)
    if predicted.shape != expected.shape:
        raise CalibrationError("Les tableaux de comparaison n'ont pas la meme forme")
    distances = np.linalg.norm(predicted - expected, axis=1)
    return {
        "mean_error_mm": float(np.mean(distances)) if len(distances) else 0.0,
        "max_error_mm": float(np.max(distances)) if len(distances) else 0.0,
        "num_points": int(len(distances)),
    }


def validate_homography(
    raw_charuco_corners_px: np.ndarray,
    charuco_ids: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    homography: np.ndarray,
    board: Any,
    grid_to_robot_translation_mm: Sequence[float] = (0.0, 0.0),
    grid_to_robot_rotation_deg: float = 0.0,
    grid_y_flip_max_mm: float | None = None,
) -> dict[str, float]:
    raw_points = np.asarray(raw_charuco_corners_px, dtype=np.float64).reshape(-1, 1, 2)
    world_grid_mm = charuco_ids_to_world_points_mm(charuco_ids, board)
    if grid_y_flip_max_mm is not None:
        world_grid_mm = np.column_stack((world_grid_mm[:, 0], float(grid_y_flip_max_mm) - world_grid_mm[:, 1]))
    world_robot_mm = transform_grid_to_robot(world_grid_mm, grid_to_robot_translation_mm, grid_to_robot_rotation_deg)

    predicted_mm = []
    for point in raw_points.reshape(-1, 2):
        predicted_mm.append(raw_pixel_to_robot_mm(point[0], point[1], camera_matrix, dist_coeffs, homography))
    predicted_mm = np.asarray(predicted_mm, dtype=np.float64)

    metrics = compute_mm_errors(predicted_mm, world_robot_mm)
    return metrics


def build_intrinsics_payload(
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rms_reprojection_error: float,
    image_size: tuple[int, int],
    charuco_config: dict[str, Any],
) -> dict[str, Any]:
    width, height = image_size
    return {
        "camera_matrix": np.asarray(camera_matrix, dtype=np.float64),
        "dist_coeffs": np.asarray(dist_coeffs, dtype=np.float64).reshape(1, -1),
        "rms_reprojection_error": float(rms_reprojection_error),
        "image_width": int(width),
        "image_height": int(height),
        "charuco": dict(charuco_config),
    }


def build_robot_payload(
    intrinsics_payload: dict[str, Any],
    homography: np.ndarray,
    homography_reprojection_error_mm: float,
    camera_height_mm: float,
    frame: dict[str, str],
    homography_charuco_config: dict[str, Any],
    origin_offset_mm: Sequence[float] = (0.0, 0.0),
) -> dict[str, Any]:
    if len(origin_offset_mm) != 2:
        raise CalibrationError("origin_offset_mm doit contenir exactement 2 valeurs [offset_x_mm, offset_y_mm]")
    return {
        "camera_matrix": np.asarray(intrinsics_payload["camera_matrix"], dtype=np.float64),
        "dist_coeffs": np.asarray(intrinsics_payload["dist_coeffs"], dtype=np.float64).reshape(1, -1),
        "homography_undistorted_px_to_robot_mm": np.asarray(homography, dtype=np.float64),
        "rms_reprojection_error": float(intrinsics_payload["rms_reprojection_error"]),
        "homography_reprojection_error_mm": float(homography_reprojection_error_mm),
        "units": "mm",
        "image_width": int(intrinsics_payload["image_width"]),
        "image_height": int(intrinsics_payload["image_height"]),
        "camera_height_mm": float(camera_height_mm),
        "origin_offset_mm": [float(origin_offset_mm[0]), float(origin_offset_mm[1])],
        "frame": dict(frame),
        "charuco": dict(homography_charuco_config),
        "intrinsics_charuco": dict(intrinsics_payload["charuco"]),
    }


def find_planar_homography(image_points_px: np.ndarray, world_points_mm: np.ndarray) -> np.ndarray:
    src = np.asarray(image_points_px, dtype=np.float64).reshape(-1, 1, 2)
    dst = np.asarray(world_points_mm, dtype=np.float64).reshape(-1, 1, 2)
    if len(src) < 4:
        raise CalibrationError("Au moins 4 correspondances sont necessaires pour l'homographie")
    homography, status = cv2.findHomography(src, dst, 0)
    if homography is None or status is None:
        raise CalibrationError("OpenCV n'a pas pu calculer l'homographie")
    return np.asarray(homography, dtype=np.float64)


def load_intrinsics(path: str | Path) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    payload = read_json(path)
    camera_matrix = np.asarray(payload["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.asarray(payload["dist_coeffs"], dtype=np.float64)
    return camera_matrix, dist_coeffs, payload


def format_matrix(matrix: np.ndarray) -> str:
    return np.array2string(np.asarray(matrix, dtype=np.float64), precision=6, suppress_small=False)


def load_homography(calibration_json: Path) -> tuple[np.ndarray, int, int]:
    """Charge l'homographie pixels bruts -> mm relatifs au centre image."""
    with calibration_json.open("r", encoding="utf-8") as file:
        payload = json.load(file)

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
) -> tuple[float, float]:
    """
    Convertit un pixel brut (u, v) vers une position au sol en mm.

    L'origine (0, 0) correspond au centre géométrique de l'image.
    """
    point_px = np.array([[[u, v]]], dtype=np.float64)

    point_mm = cv2.perspectiveTransform(point_px, homography)

    x_mm = float(point_mm[0, 0, 0])
    y_mm = float(point_mm[0, 0, 1])

    return x_mm, y_mm
