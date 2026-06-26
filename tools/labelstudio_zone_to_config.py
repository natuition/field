"""Extract tool_reach_zone from Label Studio JSON export and update config.

Expected Label Studio object type is rectanglelabels with:
 - value.x, value.y, value.width, value.height in percents
 - original_width, original_height in pixels
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple


DEFAULT_LABEL = "tool_reach_zone"
DEFAULT_CONFIG_KEY = "WORKING_ZONE_POLY_POINTS"


def _percent_to_px(value_percent: float, size_px: int) -> float:
    return (value_percent / 100.0) * float(size_px)


def _rotate_point(px: float, py: float, ox: float, oy: float, degrees: float) -> Tuple[float, float]:
    radians = math.radians(degrees)
    cos_t = math.cos(radians)
    sin_t = math.sin(radians)
    tx = px - ox
    ty = py - oy
    return ox + tx * cos_t - ty * sin_t, oy + tx * sin_t + ty * cos_t


def rectangle_to_polygon(result: Dict[str, Any]) -> List[List[int]]:
    value = result["value"]
    w_img = int(result["original_width"])
    h_img = int(result["original_height"])

    x = _percent_to_px(float(value["x"]), w_img)
    y = _percent_to_px(float(value["y"]), h_img)
    w = _percent_to_px(float(value["width"]), w_img)
    h = _percent_to_px(float(value["height"]), h_img)
    rotation = float(value.get("rotation", 0))

    corners = [
        (x, y),
        (x + w, y),
        (x + w, y + h),
        (x, y + h),
    ]

    if rotation != 0:
        corners = [_rotate_point(cx, cy, x, y, rotation) for cx, cy in corners]

    polygon: List[List[int]] = []
    for cx, cy in corners:
        int_x = max(0, min(w_img - 1, int(round(cx))))
        int_y = max(0, min(h_img - 1, int(round(cy))))
        polygon.append([int_x, int_y])
    return polygon


def iter_results(payload: Any) -> Iterable[Dict[str, Any]]:
    tasks = payload if isinstance(payload, list) else [payload]
    for task in tasks:
        for annotation in task.get("annotations", []):
            for result in annotation.get("result", []):
                yield result


def find_label_result(payload: Any, label: str) -> Dict[str, Any]:
    for result in iter_results(payload):
        if result.get("type") != "rectanglelabels":
            continue
        labels = result.get("value", {}).get("rectanglelabels", [])
        if label in labels:
            return result
    raise ValueError(f"Label '{label}' not found in rectanglelabels annotations")


def update_config_key(config_path: Path, key: str, value: List[List[int]]) -> None:
    lines = config_path.read_text(encoding="utf-8").splitlines()
    replacement = f"{key} = {value}"
    replaced = False
    out_lines: List[str] = []

    for line in lines:
        stripped = line.lstrip()
        if stripped.startswith("#"):
            out_lines.append(line)
            continue

        if line.split("=", 1)[0].strip() == key:
            out_lines.append(replacement)
            replaced = True
        else:
            out_lines.append(line)

    if not replaced:
        out_lines.append(replacement)

    config_path.write_text("\n".join(out_lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Extract tool_reach_zone from Label Studio JSON and update WORKING_ZONE_POLY_POINTS"
    )
    parser.add_argument("json_path", type=Path, help="Path to Label Studio JSON export")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/config.py"),
        help="Path to config.py (default: config/config.py)",
    )
    parser.add_argument(
        "--label",
        default=DEFAULT_LABEL,
        help=f"Rectangle label to extract (default: {DEFAULT_LABEL})",
    )
    parser.add_argument(
        "--key",
        default=DEFAULT_CONFIG_KEY,
        help=f"Config key to update (default: {DEFAULT_CONFIG_KEY})",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print extracted points without updating config",
    )
    args = parser.parse_args()

    payload = json.loads(args.json_path.read_text(encoding="utf-8"))
    result = find_label_result(payload, args.label)
    polygon_points = rectangle_to_polygon(result)

    print(f"Extracted {args.label}: {polygon_points}")
    if args.dry_run:
        return

    update_config_key(args.config, args.key, polygon_points)
    print(f"Updated {args.key} in {args.config}")


if __name__ == "__main__":
    main()
