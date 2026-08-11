from __future__ import annotations

import argparse
import json
import math
import mmap
import os
import socket
import struct
import time
from pathlib import Path
from typing import Any, Iterator

from flask import Flask, Response, jsonify, render_template, request


FRAME_HEADER = struct.Struct("<4sHHQQII")
FRAME_MAGIC = b"QYGF"
FRAME_VERSION = 1


def get_local_ip() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("10.255.255.255", 1))
        return str(sock.getsockname()[0])
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


def _fresh(path: str, stale_after: float) -> bool:
    try:
        age = time.time() - os.stat(path).st_mtime
        return 0.0 <= age <= stale_after
    except OSError:
        return False


def _read_consistent_frame_packet(path: str, mapped_size: int) -> tuple[bytes, int] | None:
    try:
        with open(path, "rb") as source:
            if os.fstat(source.fileno()).st_size < mapped_size:
                return None
            with mmap.mmap(source.fileno(), mapped_size, access=mmap.ACCESS_READ) as mapped:
                first_bytes = mapped[: FRAME_HEADER.size]
                first = FRAME_HEADER.unpack(first_bytes)
                magic, version, header_size, sequence, monotonic_ns, jpeg_size, capacity = first
                if (
                    magic != FRAME_MAGIC
                    or version != FRAME_VERSION
                    or header_size != FRAME_HEADER.size
                    or sequence == 0
                    or sequence % 2 != 0
                    or jpeg_size == 0
                    or jpeg_size > capacity
                    or capacity > mapped_size - header_size
                    or header_size + jpeg_size > mapped_size
                ):
                    return None
                payload = bytes(mapped[header_size : header_size + jpeg_size])
                second_bytes = mapped[: FRAME_HEADER.size]
                second = FRAME_HEADER.unpack(second_bytes)
                if first != second or second[3] % 2 != 0:
                    return None
                if not (payload.startswith(b"\xff\xd8") and payload.endswith(b"\xff\xd9")):
                    return None
                return payload, monotonic_ns
    except (OSError, ValueError, struct.error):
        return None


def read_consistent_frame(path: str, mapped_size: int) -> bytes | None:
    packet = _read_consistent_frame_packet(path, mapped_size)
    return packet[0] if packet is not None else None


def _frame_is_fresh(packet: tuple[bytes, int] | None, stale_after: float) -> bool:
    if packet is None:
        return False
    age_ns = time.monotonic_ns() - packet[1]
    return -1_000_000_000 <= age_ns <= int(stale_after * 1_000_000_000)


def _is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _validate_schema_version(value: dict[str, Any]) -> None:
    version = value.get("schema_version")
    if not isinstance(version, int) or isinstance(version, bool) or version != 1:
        raise ValueError("unsupported or missing schema_version")


def _validate_data(value: dict[str, Any]) -> None:
    _validate_schema_version(value)
    times = value.get("time")
    if not isinstance(times, list) or not all(_is_finite_number(item) for item in times):
        raise ValueError("time must be an array of finite numbers")
    for key, series in value.items():
        if key in {"schema_version", "time"} or not isinstance(series, list):
            continue
        if len(series) != len(times):
            raise ValueError(f"series {key} does not match time length")
        if not all(item is None or _is_finite_number(item) for item in series):
            raise ValueError(f"series {key} contains an invalid value")


def _validate_log(value: dict[str, Any]) -> None:
    _validate_schema_version(value)
    if not isinstance(value.get("source"), str):
        raise ValueError("source must be a string")

    sections: dict[str, dict[str, Any]] = {}
    for section_name in ("system", "detector", "tracker", "planner", "gimbal", "timing"):
        section = value.get(section_name)
        if not isinstance(section, dict):
            raise ValueError(f"{section_name} must be an object")
        sections[section_name] = section

    string_fields = {
        ("system", "mode"),
        ("tracker", "state"),
    }
    boolean_fields = {
        ("system", "producer_online"),
        ("tracker", "has_target"),
    }
    integer_fields = {
        ("system", "frame_id"),
        ("detector", "armor_count"),
        ("detector", "queue_depth"),
        ("planner", "observation_frame_id"),
    }
    numeric_fields = {
        ("system", "fps"),
        ("gimbal", "yaw_deg"),
        ("gimbal", "pitch_deg"),
        ("gimbal", "bullet_speed_mps"),
        ("timing", "frame_age_ms"),
        ("timing", "detector_wait_ms"),
        ("timing", "tracker_ms"),
        ("timing", "planner_ms"),
        ("timing", "publisher_ms"),
    }
    for section_name, field in string_fields:
        if not isinstance(sections[section_name].get(field), str):
            raise ValueError(f"{section_name}.{field} must be a string")
    for section_name, field in boolean_fields:
        if not isinstance(sections[section_name].get(field), bool):
            raise ValueError(f"{section_name}.{field} must be boolean")
    for section_name, field in integer_fields:
        item = sections[section_name].get(field)
        if not isinstance(item, int) or isinstance(item, bool) or item < 0:
            raise ValueError(f"{section_name}.{field} must be a non-negative integer")
    for section_name, field in numeric_fields:
        if not _is_finite_number(sections[section_name].get(field)):
            raise ValueError(f"{section_name}.{field} must be a finite number")

    planner = sections["planner"]
    planner_valid = planner.get("valid")
    if not isinstance(planner_valid, bool):
        raise ValueError("planner.valid must be boolean")
    nullable_planner_fields = (
        "age_ms", "control", "fire", "target_yaw_deg", "target_pitch_deg",
        "command_yaw_deg", "command_pitch_deg",
    )
    if planner_valid:
        for field in ("control", "fire"):
            if not isinstance(planner.get(field), bool):
                raise ValueError(f"planner.{field} must be boolean")
        for field in (
            "age_ms", "target_yaw_deg", "target_pitch_deg", "command_yaw_deg",
            "command_pitch_deg",
        ):
            if not _is_finite_number(planner.get(field)):
                raise ValueError(f"planner.{field} must be a finite number")
    elif any(planner.get(field) is not None for field in nullable_planner_fields):
        raise ValueError("invalid planner values must be null")

    if "target" not in value:
        raise ValueError("target is required")
    target = value["target"]
    if target is None:
        return
    if not isinstance(target, dict):
        raise ValueError("target must be an object or null")
    for field in ("name", "type"):
        if not isinstance(target.get(field), str):
            raise ValueError(f"target.{field} must be a string")
    for field in (
        "x_m", "y_m", "z_m", "vx_mps", "vy_mps", "vz_mps", "yaw_deg",
        "yaw_speed_radps", "radius_m",
    ):
        if not _is_finite_number(target.get(field)):
            raise ValueError(f"target.{field} must be a finite number")
    last_id = target.get("last_id")
    if not isinstance(last_id, int) or isinstance(last_id, bool):
        raise ValueError("target.last_id must be an integer")


def _read_json(path: str, stale_after: float, source: str) -> dict[str, Any]:
    if not _fresh(path, stale_after):
        raise OSError("source is missing or stale")
    with open(path, encoding="utf-8") as input_file:
        value = json.load(input_file)
    if not isinstance(value, dict):
        raise ValueError("payload root must be an object")
    if source == "data":
        _validate_data(value)
    elif source == "log":
        _validate_log(value)
    else:
        raise ValueError("unknown JSON source")
    return value


def _trim_series(value: dict[str, Any], max_points: int) -> dict[str, Any]:
    times = value.get("time")
    if not isinstance(times, list) or len(times) <= max_points:
        return value
    start = len(times) - max_points
    return {
        key: item[start:] if isinstance(item, list) and len(item) == len(times) else item
        for key, item in value.items()
    }


def _bounded_int(raw: str | None, default: int, lower: int, upper: int) -> int:
    try:
        parsed = int(raw) if raw is not None else default
    except ValueError:
        parsed = default
    return min(max(parsed, lower), upper)


def _json_response(payload: dict[str, Any], status: int = 200):
    response = jsonify(payload)
    response.status_code = status
    response.headers["Cache-Control"] = "no-store, max-age=0"
    return response


def create_app(config: dict[str, Any] | None = None) -> Flask:
    app = Flask(__name__)
    app.config.from_mapping(
        FRAME_PATH="/dev/shm/qyg_frame",
        DATA_PATH="/dev/shm/qyg_data.json",
        LOG_PATH="/dev/shm/qyg_log.json",
        FRAME_SIZE=4 * 1024 * 1024,
        STALE_AFTER_SECONDS=2.0,
        STREAM_FPS=60.0,
        PORT=9000,
    )
    if config:
        app.config.update(config)

    def source_error(source: str):
        return _json_response(
            {"error": f"{source} source is offline, stale, or invalid", "source": source}, 503
        )

    @app.get("/")
    def index():
        return render_template(
            "index.html",
            server_url=f"http://{get_local_ip()}:{app.config['PORT']}",
        )

    @app.get("/health")
    def health():
        stale_after = float(app.config["STALE_AFTER_SECONDS"])
        video = _frame_is_fresh(
            _read_consistent_frame_packet(
                app.config["FRAME_PATH"], int(app.config["FRAME_SIZE"])
            ),
            stale_after,
        )
        try:
            _read_json(app.config["DATA_PATH"], stale_after, "data")
            data = True
        except (OSError, ValueError, json.JSONDecodeError):
            data = False
        try:
            log_value = _read_json(app.config["LOG_PATH"], stale_after, "log")
            log = True
        except (OSError, ValueError, json.JSONDecodeError):
            log_value = {}
            log = False
        producer = bool(log and log_value.get("system", {}).get("producer_online", False))
        return _json_response(
            {"video": video, "data": data, "log": log, "producer": producer}
        )

    @app.get("/data")
    def data():
        try:
            value = _read_json(
                app.config["DATA_PATH"], float(app.config["STALE_AFTER_SECONDS"]), "data"
            )
            max_points = _bounded_int(request.args.get("max_points"), 200, 10, 600)
            return _json_response(_trim_series(value, max_points))
        except (OSError, ValueError, json.JSONDecodeError):
            return source_error("data")

    @app.get("/log")
    def log():
        try:
            return _json_response(
                _read_json(
                    app.config["LOG_PATH"], float(app.config["STALE_AFTER_SECONDS"]), "log"
                )
            )
        except (OSError, ValueError, json.JSONDecodeError):
            return source_error("log")

    @app.get("/video")
    def video():
        frame_path = app.config["FRAME_PATH"]
        frame_size = int(app.config["FRAME_SIZE"])
        stale_after = float(app.config["STALE_AFTER_SECONDS"])
        first = _read_consistent_frame_packet(frame_path, frame_size)
        if not _frame_is_fresh(first, stale_after):
            return source_error("video")

        def stream() -> Iterator[bytes]:
            packet = first
            interval = 1.0 / float(app.config["STREAM_FPS"])
            while _frame_is_fresh(packet, stale_after):
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + packet[0] + b"\r\n"
                time.sleep(interval)
                packet = _read_consistent_frame_packet(frame_path, frame_size)

        response = Response(stream(), mimetype="multipart/x-mixed-replace; boundary=frame")
        response.headers["Cache-Control"] = "no-store, max-age=0"
        response.headers["X-Accel-Buffering"] = "no"
        return response

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="庆园阁视觉调试平台")
    parser.add_argument("--port", type=int, default=9000)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    app = create_app({"PORT": args.port})
    print(f"庆园阁视觉调试平台: http://{get_local_ip()}:{args.port}")
    app.run(host="0.0.0.0", port=args.port, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
