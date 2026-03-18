#!/usr/bin/env python3

import argparse
import re
from pathlib import Path
from datetime import datetime

import numpy as np
import pandas as pd


DEFAULT_WAIT_TIME = 5.0
DEFAULT_TORQUE_SCALE = 0.102834


PAIR_NAME_RE = re.compile(r"torque_([+-]?\d+)_velocity_([+-]?\d+)\.csv$")


def parse_timestamp(value):
    return datetime.strptime(value, "%H:%M:%S.%f")


def parse_pair_from_name(path):
    m = PAIR_NAME_RE.search(path.name)
    if not m:
        return None, None
    return int(m.group(1)), int(m.group(2))


def collect_csv_paths(inputs, base_dir):
    paths = []

    if not inputs:
        candidates = sorted(
            [p for p in base_dir.glob("experiment_*") if p.is_dir()],
            key=lambda p: p.name,
        )
        if not candidates:
            return []
        latest = candidates[-1]
        return sorted([p for p in latest.glob("*.csv") if p.is_file()])

    for item in inputs:
        p = Path(item).expanduser().resolve()
        if p.is_dir():
            paths.extend([x for x in p.glob("*.csv") if x.is_file()])
        elif p.is_file() and p.suffix.lower() == ".csv":
            paths.append(p)

    # deduplicate and stable sort
    uniq = sorted({p.resolve() for p in paths})
    return uniq


def analyze_one_csv(path, wait_time, torque_scale):
    df = pd.read_csv(path)
    if df.empty:
        return None

    required = [
        "timestamp",
        "sensor_torque",
        "act_target_torque",
        "load_target_velocity",
        "act_torque",
    ]
    for col in required:
        if col not in df.columns:
            raise ValueError(f"{path.name}: missing column '{col}'")

    if "thermal_paused" in df.columns:
        df = df[df["thermal_paused"] == 0]
    if df.empty:
        return None

    df = df.copy()
    df["time_obj"] = pd.to_datetime(df["timestamp"], format="%H:%M:%S.%f", errors="coerce")
    df = df.dropna(subset=["time_obj"])
    if df.empty:
        return None

    t0 = df.iloc[0]["time_obj"]
    stable = df[(df["time_obj"] - t0).dt.total_seconds() > wait_time]
    if not stable.empty:
        segment = stable
    else:
        segment = df

    real_torque = segment["sensor_torque"] / torque_scale
    input_power = segment["act_torque"] * 2.58 / 1000.0 * 22.0
    output_power = real_torque
    efficiency = input_power / output_power
    efficiency = efficiency.replace([np.inf, -np.inf], np.nan).dropna()
    if efficiency.empty:
        return None

    act_torque, load_velocity = parse_pair_from_name(path)
    if act_torque is None:
        act_torque = int(round(segment["act_target_torque"].median()))
    if load_velocity is None:
        load_velocity = int(round(segment["load_target_velocity"].median()))

    return {
        "file_name": path.name,
        "act_target_torque": act_torque,
        "load_target_velocity": load_velocity,
        "sample_count": int(len(efficiency)),
        "efficiency_mean": float(efficiency.mean()),
        "efficiency_std": float(efficiency.std()),
        "efficiency_min": float(efficiency.min()),
        "efficiency_max": float(efficiency.max()),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Analyze auto_exp CSV files (single folder or multiple CSVs)."
    )
    parser.add_argument(
        "inputs",
        nargs="*",
        help="CSV files or directories. If omitted, latest experiment_* directory in ./data is used.",
    )
    parser.add_argument("--wait-time", type=float, default=DEFAULT_WAIT_TIME)
    parser.add_argument("--torque-scale", type=float, default=DEFAULT_TORQUE_SCALE)
    parser.add_argument("--output", default="auto_efficiency_result.csv")
    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    csv_paths = collect_csv_paths(args.inputs, script_dir)
    if not csv_paths:
        print("No CSV files found.")
        return 1

    rows = []
    skipped = []
    for p in csv_paths:
        try:
            r = analyze_one_csv(p, args.wait_time, args.torque_scale)
            if r is None:
                skipped.append((p.name, "empty/invalid after filtering"))
            else:
                rows.append(r)
        except Exception as e:
            skipped.append((p.name, str(e)))

    if not rows:
        print("No valid analysis rows produced.")
        if skipped:
            print("Skipped files:")
            for name, reason in skipped:
                print(f"- {name}: {reason}")
        return 2

    out_df = pd.DataFrame(rows)
    out_df = out_df.sort_values(["act_target_torque", "load_target_velocity"]).reset_index(drop=True)

    out_path = Path(args.output).expanduser()
    if not out_path.is_absolute():
        out_path = Path.cwd() / out_path
    out_df.to_csv(out_path, index=False)

    print(f"Analysis complete: {out_path}")
    print(f"Analyzed files: {len(rows)}")
    if skipped:
        print(f"Skipped files: {len(skipped)}")
        for name, reason in skipped:
            print(f"- {name}: {reason}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
