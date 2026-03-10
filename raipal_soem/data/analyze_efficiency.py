import pandas as pd
import numpy as np
from datetime import datetime

# ==============================
# 설정
# ==============================

INPUT_CSV = "data.csv"
OUTPUT_CSV = "efficiency_result.csv"

wait_time = 2.0   # 안정화 대기 시간 (sec)

TORQUE_SCALE = 0.102834

# ==============================
# 시간 파싱
# ==============================

def parse_time(t):
    return datetime.strptime(t, "%H:%M:%S.%f")


# ==============================
# CSV 읽기
# ==============================

df = pd.read_csv(INPUT_CSV)

df["time_obj"] = df["timestamp"].apply(parse_time)

events = df.index[df["target_flag"] == 1].tolist()

results = []

# ==============================
# Step별 분석
# ==============================

for i, start_idx in enumerate(events):

    start_time = df.loc[start_idx, "time_obj"]

    if i < len(events) - 1:
        end_idx = events[i+1]
    else:
        end_idx = len(df)

    segment = df.iloc[start_idx:end_idx].copy()

    # 안정화 이후 데이터만 사용
    segment = segment[
        (segment["time_obj"] - start_time).dt.total_seconds() > wait_time
    ]

    if len(segment) == 0:
        continue

    target_torque = df.loc[start_idx, "act_target_torque"]
    target_vel = df.loc[start_idx, "act_target_velocity"]

    # ==============================
    # 단위 변환
    # ==============================

    # sensor torque 보정
    real_torque = segment["sensor_torque"] / TORQUE_SCALE

    # sensor rpm -> rad/s
    sensor_w = segment["sensor_rpm"] * 2*np.pi / 60

    # actuator velocity -> rad/s
    act_w = segment["act_vel"] * 2*np.pi / 65536

    act_tor = segment["act_torque"]

    # ==============================
    # Power 계산
    # ==============================

    output_power = real_torque * sensor_w
    input_power = act_tor * 2.58 / 1000 * act_w

    efficiency = output_power / input_power

    efficiency = efficiency.replace([np.inf, -np.inf], np.nan).dropna()

    # 비정상 값 제거
    efficiency = efficiency[(efficiency > 0) & (efficiency < 2)]

    if len(efficiency) == 0:
        continue

    eff_mean = efficiency.mean()
    eff_std = efficiency.std()

    stable_timestamp = segment.iloc[0]["timestamp"]

    results.append({
        "timestamp": stable_timestamp,
        "target_torque": target_torque,
        "target_velocity": target_vel,
        "efficiency_mean": eff_mean,
        "efficiency_std": eff_std
    })


# ==============================
# 결과 저장
# ==============================

result_df = pd.DataFrame(results)

result_df.to_csv(OUTPUT_CSV, index=False)

print("\nAnalysis complete\n")
print(result_df)