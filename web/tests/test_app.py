import json
import os
import struct
import time

import pytest

from web.app import CURVE_SERIES, FRAME_HEADER, create_app, read_consistent_frame


@pytest.fixture()
def paths(tmp_path):
    return {
        "FRAME_PATH": str(tmp_path / "qyg_frame"),
        "DATA_PATH": str(tmp_path / "qyg_data.json"),
        "LOG_PATH": str(tmp_path / "qyg_log.json"),
        "FRAME_SIZE": 4096,
        "STALE_AFTER_SECONDS": 2.0,
        "TESTING": True,
    }


@pytest.fixture()
def client(paths):
    return create_app(paths).test_client()


def write_json(path, value):
    with open(path, "w", encoding="utf-8") as output:
        json.dump(value, output)


def write_frame(
    path,
    sequence=2,
    payload=b"\xff\xd8jpeg\xff\xd9",
    size=4096,
    timestamp_ns=None,
    capacity=None,
):
    if timestamp_ns is None:
        timestamp_ns = time.monotonic_ns()
    if capacity is None:
        capacity = size - FRAME_HEADER.size
    header = FRAME_HEADER.pack(
        b"QYGF", 1, FRAME_HEADER.size, sequence, timestamp_ns, len(payload), capacity
    )
    with open(path, "wb") as output:
        output.write(header)
        output.write(payload)
        output.truncate(size)


def valid_data(values=None):
    values = [] if values is None else values
    return {
        "schema_version": 1,
        "time": values,
        **{key: list(values) for key in CURVE_SERIES},
    }


def valid_log():
    return {
        "schema_version": 1,
        "source": "test",
        "system": {"mode": "IDLE", "producer_online": True, "fps": 0.0, "frame_id": 0},
        "detector": {"armor_count": 0, "queue_depth": 0},
        "tracker": {"state": "lost", "has_target": False},
        "target": None,
        "planner": {
            "valid": True,
            "observation_frame_id": 0,
            "age_ms": 0.0,
            "control": False,
            "fire": False,
            "target_yaw_deg": 0.0,
            "target_pitch_deg": 0.0,
            "command_yaw_deg": 0.0,
            "command_pitch_deg": 0.0,
        },
        "gimbal": {"yaw_deg": 0.0, "pitch_deg": 0.0, "bullet_speed_mps": 0.0},
        "timing": {
            "frame_age_ms": 0.0,
            "detector_wait_ms": 0.0,
            "tracker_ms": 0.0,
            "planner_ms": 0.0,
            "publisher_ms": 0.0,
        },
    }


def test_missing_sources_are_reported_offline(client):
    health = client.get("/health")
    assert health.status_code == 200
    assert health.get_json() == {
        "video": False,
        "data": False,
        "log": False,
        "producer": False,
    }
    assert client.get("/data").status_code == 503
    assert client.get("/log").status_code == 503
    assert client.get("/video").status_code == 503


def test_data_trims_equal_length_series_and_clamps_to_ten(paths, client):
    values = list(range(20))
    payload = valid_data(values)
    write_json(paths["DATA_PATH"], payload)

    response = client.get("/data?max_points=1")

    assert response.status_code == 200
    result = response.get_json()
    assert result["schema_version"] == 1
    assert result["time"] == values[-10:]
    assert result["fps"] == values[-10:]
    assert all(len(result[key]) == 10 for key in CURVE_SERIES)


def test_bad_or_stale_json_returns_503(paths, client):
    with open(paths["LOG_PATH"], "w", encoding="utf-8") as output:
        output.write("{")
    assert client.get("/log").status_code == 503

    write_json(paths["LOG_PATH"], {"schema_version": 1})
    stale = time.time() - 3.0
    os.utime(paths["LOG_PATH"], (stale, stale))
    assert client.get("/log").status_code == 503


@pytest.mark.parametrize(
    "payload",
    [
        {"schema_version": 1, "time": "not-an-array"},
        {"schema_version": 1, "time": [0, 1], "fps": [60]},
        {"schema_version": 1, "time": [0], "fps": ["fast"]},
        {**valid_data(), "fps": "fast"},
        {**valid_data(), "unknown": []},
        {"schema_version": True, "time": []},
    ],
)
def test_data_rejects_malformed_schema_shaped_payloads(paths, client, payload):
    write_json(paths["DATA_PATH"], payload)
    assert client.get("/data").status_code == 503
    assert client.get("/health").get_json()["data"] is False


def test_log_requires_boolean_producer_status(paths, client):
    payload = valid_log()
    payload["system"]["producer_online"] = "true"
    write_json(paths["LOG_PATH"], payload)
    assert client.get("/log").status_code == 503
    health = client.get("/health").get_json()
    assert health["log"] is False
    assert health["producer"] is False


@pytest.mark.parametrize(
    ("section", "field", "bad_value"),
    [
        ("system", "frame_id", 1.5),
        ("detector", "armor_count", -1),
        ("tracker", "has_target", 1),
        ("planner", "control", "false"),
        ("gimbal", "yaw_deg", "zero"),
        ("timing", "frame_age_ms", None),
    ],
)
def test_log_rejects_invalid_critical_field_types(paths, client, section, field, bad_value):
    payload = valid_log()
    payload[section][field] = bad_value
    write_json(paths["LOG_PATH"], payload)
    assert client.get("/log").status_code == 503


def test_json_source_with_future_mtime_is_rejected(paths, client):
    write_json(paths["DATA_PATH"], valid_data())
    future = time.time() + 10.0
    os.utime(paths["DATA_PATH"], (future, future))
    assert client.get("/data").status_code == 503


def test_health_tracks_each_source_and_producer(paths, client):
    write_frame(paths["FRAME_PATH"], size=paths["FRAME_SIZE"])
    write_json(paths["DATA_PATH"], valid_data())
    write_json(paths["LOG_PATH"], valid_log())

    assert client.get("/health").get_json() == {
        "video": True,
        "data": True,
        "log": True,
        "producer": True,
    }


def test_health_uses_frame_monotonic_timestamp_not_file_mtime(paths, client):
    write_frame(
        paths["FRAME_PATH"],
        size=paths["FRAME_SIZE"],
        timestamp_ns=time.monotonic_ns() - 3_000_000_000,
    )
    assert client.get("/health").get_json()["video"] is False


def test_reader_rejects_in_progress_and_accepts_stable_jpeg(paths):
    write_frame(paths["FRAME_PATH"], sequence=3, size=paths["FRAME_SIZE"])
    assert read_consistent_frame(paths["FRAME_PATH"], paths["FRAME_SIZE"]) is None

    write_frame(paths["FRAME_PATH"], sequence=4, size=paths["FRAME_SIZE"])
    assert read_consistent_frame(paths["FRAME_PATH"], paths["FRAME_SIZE"]) == b"\xff\xd8jpeg\xff\xd9"

    write_frame(
        paths["FRAME_PATH"], sequence=6, size=paths["FRAME_SIZE"], capacity=paths["FRAME_SIZE"] * 2
    )
    assert read_consistent_frame(paths["FRAME_PATH"], paths["FRAME_SIZE"]) is None


def test_video_returns_mjpeg_chunk(paths, client):
    write_frame(paths["FRAME_PATH"], sequence=2, size=paths["FRAME_SIZE"])
    response = client.get("/video", buffered=False)
    assert response.status_code == 200
    assert next(response.response).startswith(b"--frame\r\nContent-Type: image/jpeg")


def test_index_is_qyg_branded_and_offline_capable(client):
    response = client.get("/")
    body = response.get_data(as_text=True)
    assert response.status_code == 200
    assert "庆园阁视觉调试平台" in body
    assert all(label in body for label in ("VIDEO", "DATA", "LOG", "PRODUCER"))
    assert "chart.umd.min.js" in body
    assert "qyg-team-logo-header.webp" in body
    assert "qyg-team-favicon.png" in body
    assert "cdn.jsdelivr" not in body
    assert client.get("/static/vendor/chart.umd.min.js").status_code == 200
    assert client.get("/static/img/qyg-team-logo-original.jpg").status_code == 200
    assert client.get("/static/img/qyg-team-logo-header.webp").status_code == 200
    assert client.get("/static/img/qyg-team-favicon.png").status_code == 200
