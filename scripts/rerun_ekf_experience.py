#!/usr/bin/env python3
"""Replay ekf_visual_experience JSONL output in Rerun."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image
import rerun as rr


def set_frame_time(frame: int, timestamp: float) -> None:
    if hasattr(rr, "set_time_sequence"):
        rr.set_time_sequence("frame", frame)
        rr.set_time_seconds("time", timestamp)
    else:
        rr.set_time("frame", sequence=frame)
        rr.set_time("time", timestamp=timestamp)


def scalar(row: dict[str, Any], name: str, default: float = 0.0) -> float:
    value = row.get(name, default)
    if isinstance(value, bool):
        return float(value)
    if isinstance(value, (int, float)):
        return float(value)
    return default


def points3d(value: Any) -> list[list[float]]:
    if not isinstance(value, list):
        return []
    if len(value) == 3 and all(isinstance(x, (int, float)) for x in value):
        return [[float(value[0]), float(value[1]), float(value[2])]]
    points: list[list[float]] = []
    for item in value:
        if isinstance(item, list) and len(item) == 3:
            points.append([float(item[0]), float(item[1]), float(item[2])])
    return points


def log_optional_point(path: str, row: dict[str, Any], field: str, color: tuple[int, int, int]) -> None:
    pts = points3d(row.get(field))
    if pts and any(abs(v) > 1e-9 for v in pts[0]):
        rr.log(path, rr.Points3D(pts, colors=[color], radii=[0.06]))


def log_scalar(path: str, value: float) -> None:
    if hasattr(rr, "Scalar"):
        rr.log(path, rr.Scalar(value))
    else:
        rr.log(path, rr.Scalars(value))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input_dir",
        type=Path,
        nargs="?",
        default=Path("logs/ekf_visual_experience"),
        help="Directory containing frames.jsonl and images/",
    )
    parser.add_argument("--save", type=Path, help="Optional .rrd file to save instead of spawning viewer")
    args = parser.parse_args()

    frames_path = args.input_dir / "frames.jsonl"
    if not frames_path.exists():
      raise FileNotFoundError(f"Missing {frames_path}")

    rr.init("rm_vision_ekf_visual_experience", spawn=args.save is None)

    with frames_path.open("r", encoding="utf-8") as f:
        for line in f:
            row = json.loads(line)
            frame = int(row["frame"])
            set_frame_time(frame, float(row.get("t", 0.0)))

            image_path = row.get("image", "")
            if image_path:
                path = Path(image_path)
                if path.exists():
                    img = np.asarray(Image.open(path).convert("RGB"))
                    rr.log("image/reprojection", rr.Image(img))

            log_optional_point("world/armor_observation", row, "armor_xyz", (0, 255, 0))
            log_optional_point("world/ekf_target", row, "target_xyz", (255, 150, 0))
            log_optional_point("world/aim_point", row, "aim_xyz", (255, 0, 0))

            predicted = points3d(row.get("predicted_armors"))
            if predicted:
                rr.log(
                    "world/predicted_armors",
                    rr.Points3D(predicted, colors=[(50, 120, 255)] * len(predicted), radii=[0.04] * len(predicted)),
                )

            for name in (
                "gimbal_yaw",
                "cmd_yaw",
                "armor_yaw",
                "residual_yaw",
                "residual_pitch",
                "residual_distance",
                "nis",
                "nees",
                "w",
                "r",
                "shoot",
            ):
                log_scalar(f"scalar/{name}", scalar(row, name))

    if args.save is not None:
        rr.save(str(args.save))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
