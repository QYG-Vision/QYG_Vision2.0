#!/usr/bin/env python3
"""Analyze QYG_sentry_debug CSV logs for pitch-loop oscillation.

The script reads the CsvLogger output produced by QYG_sentry_debug and reports
per-run/per-offset metrics. It intentionally uses column names instead of VOFA
channel numbers so the analysis survives channel insertions.
"""

from __future__ import annotations

import argparse
import shutil
import csv
import glob
import math
import statistics
from pathlib import Path


DEFAULT_LATEST_COUNT = 8
DEFAULT_ARCHIVE_FOLDER = Path("logs/pitch_loop_trials")

REQUIRED_COLUMNS = [
    "time_ms",
    "gimbal_pose_time_offset_ms",
    "target_found",
    "armor_found",
    "rx_pitch_deg",
    "tx_pitch_deg",
    "planner_pitch_deg",
    "planner_test_gain",
    "meas_ypd_pitch_deg",
    "pitch_loop_rx_pp_deg",
    "pitch_loop_meas_pp_deg",
    "ekf_abs_vz",
    "ekf_abs_w",
    "ekf_speed_norm",
]


def latest_csv() -> Path:
    files = [Path(p) for p in glob.glob("logs/pnp_plot_*.csv")]
    if not files:
        raise SystemExit("No logs/pnp_plot_*.csv found. Run QYG_sentry_debug first.")
    return max(files, key=lambda p: p.stat().st_mtime)


def latest_csvs(count: int) -> list[Path]:
    files = [Path(p) for p in glob.glob("logs/pnp_plot_*.csv")]
    if not files:
        raise SystemExit("No logs/pnp_plot_*.csv found. Run QYG_sentry_debug first.")
    return sorted(files, key=lambda p: p.stat().st_mtime, reverse=True)[:count]


def archive_csvs(paths: list[Path], folder: Path) -> None:
    folder.mkdir(parents=True, exist_ok=True)
    for path in paths:
        target = folder / path.name
        if path.resolve() == target.resolve():
            continue
        shutil.copy2(path, target)


def parse_float(value: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def read_rows(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        missing = [col for col in REQUIRED_COLUMNS if col not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"CSV missing columns: {', '.join(missing)}")
        rows = []
        for row in reader:
            rows.append({key: parse_float(value) for key, value in row.items()})
    if not rows:
        raise SystemExit(f"CSV has no data rows: {path}")
    return rows


def split_segments(rows: list[dict[str, float]], gap_ms: float) -> list[list[dict[str, float]]]:
    segments: list[list[dict[str, float]]] = []
    current: list[dict[str, float]] = []
    last_t = None
    last_offset = None
    last_gain = None

    for row in rows:
        t = row["time_ms"]
        offset = round(row["gimbal_pose_time_offset_ms"])
        gain = round(row["planner_test_gain"], 3)
        starts_new = (
            not current
            or last_t is None
            or t - last_t > gap_ms
            or last_offset is None
            or offset != last_offset
            or last_gain is None
            or abs(gain - last_gain) > 1e-3
        )
        if starts_new:
            if current:
                segments.append(current)
            current = [row]
        else:
            current.append(row)
        last_t = t
        last_offset = offset
        last_gain = gain

    if current:
        segments.append(current)
    return segments


def peak_to_peak(values: list[float]) -> float:
    if not values:
        return 0.0
    return max(values) - min(values)


def rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def mean(values: list[float]) -> float:
    if not values:
        return 0.0
    return statistics.fmean(values)


def percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    values = sorted(values)
    idx = min(len(values) - 1, max(0, round((len(values) - 1) * q)))
    return values[idx]


def analyze_segment(segment: list[dict[str, float]], skip_ms: float) -> dict[str, float | str]:
    start_t = segment[0]["time_ms"]
    end_t = segment[-1]["time_ms"]
    used = [row for row in segment if row["time_ms"] - start_t >= skip_ms]
    if not used:
        used = segment

    tx = [row["tx_pitch_deg"] for row in used]
    rx = [row["rx_pitch_deg"] for row in used]
    planner = [row["planner_pitch_deg"] for row in used]
    meas = [row["meas_ypd_pitch_deg"] for row in used]
    tx_rx_err = [a - b for a, b in zip(tx, rx)]

    target_rate = mean([row["target_found"] for row in used])
    armor_rate = mean([row["armor_found"] for row in used])
    rx_pp = peak_to_peak(rx)
    meas_pp = peak_to_peak(meas)
    tx_pp = peak_to_peak(tx)
    planner_pp = peak_to_peak(planner)
    loop_rx_pp = percentile([row["pitch_loop_rx_pp_deg"] for row in used], 0.95)
    loop_meas_pp = percentile([row["pitch_loop_meas_pp_deg"] for row in used], 0.95)
    ekf_vz_p95 = percentile([row["ekf_abs_vz"] for row in used], 0.95)
    ekf_w_p95 = percentile([row["ekf_abs_w"] for row in used], 0.95)
    ekf_speed_p95 = percentile([row["ekf_speed_norm"] for row in used], 0.95)

    suspicion = "mixed"
    if target_rate < 0.9 or armor_rate < 0.9:
        suspicion = "detection_dropout"
    elif tx_pp >= 1.0 and rx_pp >= 1.0:
        suspicion = "planner_or_tx_step_driving_loop"
    elif tx_pp < 0.3 and rx_pp >= 1.0:
        suspicion = "gimbal_control_response"
    elif rx_pp < 0.3 and meas_pp >= 1.0:
        suspicion = "measurement_or_pose_time_alignment"
    elif ekf_vz_p95 < 0.05 and (rx_pp >= 1.0 or meas_pp >= 1.0):
        suspicion = "closed_loop_pitch_oscillation_not_ekf_vz"

    tx_rx_err_rms = rms(tx_rx_err)
    command_pp = max(tx_pp, 1.0)
    raw_score = loop_meas_pp + loop_rx_pp + tx_rx_err_rms

    # Dimensionless score for comparing runs whose planner/tx command amplitudes differ.
    # Lower is better. The constants are reference "acceptable small" values, not hard limits.
    norm_meas = loop_meas_pp / command_pp
    norm_rx = loop_rx_pp / command_pp
    norm_err = tx_rx_err_rms / command_pp
    norm_vz = ekf_vz_p95 / 0.2
    norm_w = ekf_w_p95 / 0.5
    dropout_penalty = max(0.0, 0.98 - target_rate) * 10.0 + max(0.0, 0.98 - armor_rate) * 5.0
    fair_score = (
        0.35 * norm_meas
        + 0.25 * norm_rx
        + 0.25 * norm_err
        + 0.10 * norm_vz
        + 0.05 * norm_w
        + dropout_penalty
    )

    comparable = target_rate >= 0.95 and armor_rate >= 0.95 and command_pp >= 1.0

    return {
        "csv": "",
        "offset_ms": round(segment[0]["gimbal_pose_time_offset_ms"]),
        "gain": round(mean([row["planner_test_gain"] for row in used]), 3),
        "duration_s": (end_t - start_t) / 1000.0,
        "used_s": (used[-1]["time_ms"] - used[0]["time_ms"]) / 1000.0 if len(used) > 1 else 0.0,
        "target_rate": target_rate,
        "armor_rate": armor_rate,
        "planner_pp": planner_pp,
        "tx_pp": tx_pp,
        "rx_pp": rx_pp,
        "meas_pp": meas_pp,
        "loop_rx_pp_p95": loop_rx_pp,
        "loop_meas_pp_p95": loop_meas_pp,
        "tx_rx_err_rms": tx_rx_err_rms,
        "ekf_abs_vz_p95": ekf_vz_p95,
        "ekf_abs_w_p95": ekf_w_p95,
        "ekf_speed_p95": ekf_speed_p95,
        "command_pp": command_pp,
        "raw_score": raw_score,
        "fair_score": fair_score,
        "comparable": comparable,
        "suspicion": suspicion,
    }


def print_table(results: list[dict[str, float | str]]) -> None:
    headers = [
        "offset",
        "gain",
        "used_s",
        "tgt%",
        "armor%",
        "plan_pp",
        "tx_pp",
        "rx_pp",
        "meas_pp",
        "rxpp95",
        "measpp95",
        "err_rms",
        "vz95",
        "raw",
        "fair",
        "fair?",
        "suspect",
    ]
    print(" ".join(f"{h:>10}" for h in headers))
    for r in sorted(results, key=lambda x: (float(x["fair_score"]), float(x["raw_score"]), float(x["offset_ms"]))):
        values = [
            f'{r["offset_ms"]:>10.0f}',
            f'{r["gain"]:>10.2f}',
            f'{r["used_s"]:>10.1f}',
            f'{100 * float(r["target_rate"]):>10.0f}',
            f'{100 * float(r["armor_rate"]):>10.0f}',
            f'{r["planner_pp"]:>10.2f}',
            f'{r["tx_pp"]:>10.2f}',
            f'{r["rx_pp"]:>10.2f}',
            f'{r["meas_pp"]:>10.2f}',
            f'{r["loop_rx_pp_p95"]:>10.2f}',
            f'{r["loop_meas_pp_p95"]:>10.2f}',
            f'{r["tx_rx_err_rms"]:>10.2f}',
            f'{r["ekf_abs_vz_p95"]:>10.3f}',
            f'{r["raw_score"]:>10.2f}',
            f'{r["fair_score"]:>10.3f}',
            f'{("yes" if r["comparable"] else "no"):>10}',
            f'{str(r["suspicion"]):>10}',
        ]
        print(" ".join(values))


def print_offset_summary(results: list[dict[str, float | str]]) -> dict[str, float | str]:
    by_param: dict[tuple[float, float], list[dict[str, float | str]]] = {}
    for result in results:
        if not bool(result["comparable"]):
            continue
        key = (float(result["offset_ms"]), float(result["gain"]))
        by_param.setdefault(key, []).append(result)

    if not by_param:
        best = min(results, key=lambda r: (float(r["fair_score"]), float(r["raw_score"])))
        print()
        print("offset_summary: no fully comparable run found; recommendation is low confidence.")
        return best

    summary = []
    for (offset, gain), offset_results in by_param.items():
        fair_values = [float(r["fair_score"]) for r in offset_results]
        raw_values = [float(r["raw_score"]) for r in offset_results]
        rx_values = [float(r["loop_rx_pp_p95"]) for r in offset_results]
        meas_values = [float(r["loop_meas_pp_p95"]) for r in offset_results]
        summary.append(
            {
                "offset_ms": offset,
                "gain": gain,
                "count": len(offset_results),
                "fair_median": statistics.median(fair_values),
                "raw_median": statistics.median(raw_values),
                "rxpp95_median": statistics.median(rx_values),
                "measpp95_median": statistics.median(meas_values),
                "best_run": min(offset_results, key=lambda r: (float(r["fair_score"]), float(r["raw_score"]))),
            }
        )

    summary.sort(
        key=lambda r: (
            float(r["fair_median"]),
            float(r["raw_median"]),
            float(r["offset_ms"]),
            float(r["gain"]),
        )
    )
    print()
    print("parameter_summary_comparable_only:")
    print(
        f'{"offset":>10} {"gain":>8} {"runs":>6} {"fair_med":>10} {"raw_med":>10} '
        f'{"rxpp95":>10} {"measpp95":>10}'
    )
    for row in summary:
        print(
            f'{row["offset_ms"]:>10.0f} {row["gain"]:>8.2f} {row["count"]:>6} '
            f'{row["fair_median"]:>10.3f} '
            f'{row["raw_median"]:>10.2f} {row["rxpp95_median"]:>10.2f} '
            f'{row["measpp95_median"]:>10.2f}'
        )

    return summary[0]["best_run"]


def analyze_csv(path: Path, skip_ms: float, min_used_s: float, gap_ms: float) -> list[dict[str, float | str]]:
    try:
        rows = read_rows(path)
    except ValueError as exc:
        print(f"skip_incompatible_csv: {path} ({exc})")
        return []
    segments = split_segments(rows, gap_ms)
    results = []
    for segment in segments:
        result = analyze_segment(segment, skip_ms)
        if float(result["used_s"]) >= min_used_s:
            result["csv"] = str(path)
            results.append(result)
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="*", type=Path, help="CSV path(s). Defaults to latest logs/pnp_plot_*.csv")
    parser.add_argument(
        "--latest",
        type=int,
        default=DEFAULT_LATEST_COUNT,
        help=f"Analyze latest N logs when no CSV is given. Default: {DEFAULT_LATEST_COUNT}",
    )
    parser.add_argument(
        "--archive",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=f"Copy analyzed CSVs into {DEFAULT_ARCHIVE_FOLDER} for safekeeping. Default: on",
    )
    parser.add_argument("--skip-s", type=float, default=2.0, help="Seconds to skip at each segment start")
    parser.add_argument("--min-used-s", type=float, default=3.0, help="Minimum analyzed duration per segment")
    parser.add_argument("--gap-ms", type=float, default=500.0, help="New segment if time gap is larger than this")
    args = parser.parse_args()

    paths = args.csv or latest_csvs(args.latest)
    if args.archive:
        archive_csvs(paths, DEFAULT_ARCHIVE_FOLDER)

    results = []
    for path in paths:
        results.extend(analyze_csv(path, args.skip_s * 1000.0, args.min_used_s, args.gap_ms))

    print("CSV:")
    for path in paths:
        print(f"  {path}")
    if args.archive:
        print(f"archived_to: {DEFAULT_ARCHIVE_FOLDER}")
    print(f"segments analyzed: {len(results)}")
    if not results:
        print("No segment long enough. Lower --min-used-s or collect a longer run.")
        return
    print_table(results)

    best = print_offset_summary(results)
    print()
    print("RECOMMENDATION")
    print(f'  offset: {best["offset_ms"]:.0f} ms')
    print(f'  gain: {best["gain"]:.2f}')
    print(f'  confidence: {"normal" if best["comparable"] else "low"}')
    print(f'  fair_score: {best["fair_score"]:.3f}')
    print(f'  raw_score: {best["raw_score"]:.2f}')
    print(f'  suspect: {best["suspicion"]}')
    print(f'  source_csv: {best["csv"]}')


if __name__ == "__main__":
    main()
