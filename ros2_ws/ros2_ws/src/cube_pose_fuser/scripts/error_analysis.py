#!/usr/bin/env python3
import argparse
import csv
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message  # ROS2 official API :contentReference[oaicite:3]{index=3}
from rosidl_runtime_py.utilities import get_message  # common practice for bag parsing :contentReference[oaicite:4]{index=4}


@dataclass
class PoseSample:
    t_ns: int
    p: np.ndarray        # (3,)
    q: np.ndarray        # (4,) xyzw


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


def rot_angle_deg_from_quat(q_est_xyzw: np.ndarray, q_true_xyzw: np.ndarray) -> float:
    # rotation error angle = 2*acos(|dot(q1,q2)|)
    q1 = quat_normalize(q_est_xyzw)
    q2 = quat_normalize(q_true_xyzw)
    d = float(abs(np.dot(q1, q2)))
    d = max(-1.0, min(1.0, d))
    ang = 2.0 * math.acos(d)
    return float(np.degrees(ang))


def read_pose_topic_from_bag(
    bag_dir: str,
    topic_name: str
) -> List[PoseSample]:
    """
    Read PoseStamped-like messages: msg.pose.position.{x,y,z}, msg.pose.orientation.{x,y,z,w}
    """
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()  # official tutorial uses SequentialReader :contentReference[oaicite:5]{index=5}
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    typemap: Dict[str, str] = {t.name: t.type for t in topic_types}

    if topic_name not in typemap:
        raise RuntimeError(f"Topic '{topic_name}' not found in bag. Available: {sorted(typemap.keys())}")

    msg_type = get_message(typemap[topic_name])

    samples: List[PoseSample] = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, msg_type)

        # PoseStamped: msg.pose.position + msg.pose.orientation
        # geometry_msgs/Pose includes Point position + Quaternion orientation :contentReference[oaicite:6]{index=6}
        p = msg.pose.position
        o = msg.pose.orientation
        pos = np.array([p.x, p.y, p.z], dtype=float)
        quat = quat_normalize(np.array([o.x, o.y, o.z, o.w], dtype=float))

        samples.append(PoseSample(t_ns=int(t_ns), p=pos, q=quat))

    samples.sort(key=lambda s: s.t_ns)
    return samples


def align_by_nearest_timestamp(
    est: List[PoseSample],
    truth: List[PoseSample],
    max_dt_ms: float
) -> Tuple[List[PoseSample], List[PoseSample]]:
    if not est or not truth:
        return [], []

    truth_ts = np.array([s.t_ns for s in truth], dtype=np.int64)
    max_dt_ns = int(max_dt_ms * 1e6)

    est_aligned = []
    truth_aligned = []

    for e in est:
        idx = int(np.searchsorted(truth_ts, e.t_ns))
        cand = []
        if 0 <= idx < len(truth):
            cand.append(truth[idx])
        if 0 <= idx - 1 < len(truth):
            cand.append(truth[idx - 1])

        if not cand:
            continue

        best = min(cand, key=lambda s: abs(s.t_ns - e.t_ns))
        if abs(best.t_ns - e.t_ns) <= max_dt_ns:
            est_aligned.append(e)
            truth_aligned.append(best)

    return est_aligned, truth_aligned


def distance_bins_default() -> List[float]:
    # 你可以按试验距离改：0.2~2m 常用分段
    return [0.0, 0.5, 1.0, 1.5, 2.0, 999.0]


def write_csv(path: str, rows: List[dict], fieldnames: List[str]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="rosbag2 directory (contains metadata.yaml)")
    ap.add_argument("--est_topic", default="/cube_pose_in_car")
    ap.add_argument("--truth_topic", default="/mocap/cube_pose")
    ap.add_argument("--max_dt_ms", type=float, default=30.0, help="max time gap for alignment (ms)")
    ap.add_argument("--out_dir", default="error_report")
    ap.add_argument("--topk", type=int, default=10)
    args = ap.parse_args()

    bag_dir = os.path.abspath(os.path.expanduser(args.bag))
    out_dir = os.path.abspath(os.path.expanduser(args.out_dir))
    os.makedirs(out_dir, exist_ok=True)

    est = read_pose_topic_from_bag(bag_dir, args.est_topic)
    truth = read_pose_topic_from_bag(bag_dir, args.truth_topic)

    est_a, truth_a = align_by_nearest_timestamp(est, truth, args.max_dt_ms)

    if not est_a:
        raise RuntimeError("No aligned samples. Try increasing --max_dt_ms or check topics.")

    # errors
    trans_err = np.array([np.linalg.norm(e.p - t.p) for e, t in zip(est_a, truth_a)], dtype=float)
    trans_rmse = float(np.sqrt(np.mean(trans_err ** 2)))

    rot_err_deg = np.array([rot_angle_deg_from_quat(e.q, t.q) for e, t in zip(est_a, truth_a)], dtype=float)
    rot_mean = float(np.mean(rot_err_deg))
    rot_rmse = float(np.sqrt(np.mean(rot_err_deg ** 2)))

    # distance = ||truth position||
    dists = np.array([np.linalg.norm(t.p) for t in truth_a], dtype=float)
    bins = distance_bins_default()

    # per-bin stats
    bin_stats = []
    for i in range(len(bins) - 1):
        lo, hi = bins[i], bins[i + 1]
        mask = (dists >= lo) & (dists < hi)
        if not np.any(mask):
            continue
        bin_stats.append({
            "range_m": f"[{lo:.2f},{hi:.2f})",
            "n": int(np.sum(mask)),
            "trans_rmse_m": float(np.sqrt(np.mean(trans_err[mask] ** 2))),
            "trans_mean_m": float(np.mean(trans_err[mask])),
            "rot_mean_deg": float(np.mean(rot_err_deg[mask])),
            "rot_rmse_deg": float(np.sqrt(np.mean(rot_err_deg[mask] ** 2))),
        })

    # topK worst translation error
    idx_sorted = np.argsort(-trans_err)
    worst = []
    for k in range(min(args.topk, len(idx_sorted))):
        i = int(idx_sorted[k])
        worst.append({
            "rank": k + 1,
            "t_ns": int(est_a[i].t_ns),
            "dt_ms": float(abs(est_a[i].t_ns - truth_a[i].t_ns) / 1e6),
            "dist_truth_m": float(dists[i]),
            "trans_err_m": float(trans_err[i]),
            "rot_err_deg": float(rot_err_deg[i]),
            "est_x": float(est_a[i].p[0]), "est_y": float(est_a[i].p[1]), "est_z": float(est_a[i].p[2]),
            "truth_x": float(truth_a[i].p[0]), "truth_y": float(truth_a[i].p[1]), "truth_z": float(truth_a[i].p[2]),
        })

    # write CSV (aligned samples)
    sample_rows = []
    for e, t, te, re, d in zip(est_a, truth_a, trans_err, rot_err_deg, dists):
        sample_rows.append({
            "t_ns": int(e.t_ns),
            "dt_ms": float(abs(e.t_ns - t.t_ns) / 1e6),
            "dist_truth_m": float(d),
            "trans_err_m": float(te),
            "rot_err_deg": float(re),
            "est_x": float(e.p[0]), "est_y": float(e.p[1]), "est_z": float(e.p[2]),
            "truth_x": float(t.p[0]), "truth_y": float(t.p[1]), "truth_z": float(t.p[2]),
        })

    write_csv(
        os.path.join(out_dir, "aligned_samples.csv"),
        sample_rows,
        fieldnames=list(sample_rows[0].keys())
    )
    write_csv(
        os.path.join(out_dir, "worst_cases.csv"),
        worst,
        fieldnames=list(worst[0].keys()) if worst else ["rank"]
    )

    # write markdown report
    report_md = os.path.join(out_dir, "error_report.md")
    with open(report_md, "w") as f:
        f.write("# Pose Error Report\n\n")
        f.write(f"- bag: `{bag_dir}`\n")
        f.write(f"- est_topic: `{args.est_topic}`\n")
        f.write(f"- truth_topic: `{args.truth_topic}`\n")
        f.write(f"- aligned_samples: {len(est_a)} (max_dt_ms={args.max_dt_ms})\n\n")

        f.write("## Overall\n\n")
        f.write(f"- Translation RMSE: **{trans_rmse:.4f} m**\n")
        f.write(f"- Rotation Mean Error: **{rot_mean:.2f} deg**\n")
        f.write(f"- Rotation RMSE: **{rot_rmse:.2f} deg**\n\n")

        f.write("## By Distance Bins (truth distance)\n\n")
        f.write("| Range (m) | N | Trans RMSE (m) | Trans Mean (m) | Rot Mean (deg) | Rot RMSE (deg) |\n")
        f.write("|---:|---:|---:|---:|---:|---:|\n")
        for s in bin_stats:
            f.write(f"| {s['range_m']} | {s['n']} | {s['trans_rmse_m']:.4f} | {s['trans_mean_m']:.4f} | {s['rot_mean_deg']:.2f} | {s['rot_rmse_deg']:.2f} |\n")
        f.write("\n")

        f.write("## Worst Cases (TopK by translation error)\n\n")
        f.write("| Rank | dist_truth(m) | trans_err(m) | rot_err(deg) | dt(ms) | t(ns) |\n")
        f.write("|---:|---:|---:|---:|---:|---:|\n")
        for w in worst:
            f.write(f"| {w['rank']} | {w['dist_truth_m']:.3f} | {w['trans_err_m']:.4f} | {w['rot_err_deg']:.2f} | {w['dt_ms']:.1f} | {w['t_ns']} |\n")

    print(f"平移RMSE: {trans_rmse:.4f} m")
    print(f"旋转角误差均值: {rot_mean:.2f} °  |  RMSE: {rot_rmse:.2f} °")
    print(f"输出报告目录: {out_dir}")
    print(f"- {report_md}")
    print(f"- {os.path.join(out_dir, 'aligned_samples.csv')}")
    print(f"- {os.path.join(out_dir, 'worst_cases.csv')}")


if __name__ == "__main__":
    main()

