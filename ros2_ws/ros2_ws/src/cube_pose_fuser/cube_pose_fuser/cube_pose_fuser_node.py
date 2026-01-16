#!/usr/bin/env python3
import csv
import math
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from aruco_msgs.msg import MarkerArray  # aruco_msgs/msg/MarkerArray


def quat_normalize(q_xyzw: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q_xyzw)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q_xyzw / n


def quat_to_rot(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = q_xyzw
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ], dtype=float)


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    tr = float(np.trace(R))
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S

    return quat_normalize(np.array([x, y, z, w], dtype=float))


def tf_mul(R1: np.ndarray, t1: np.ndarray, R2: np.ndarray, t2: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    R = R1 @ R2
    t = t1 + R1 @ t2
    return R, t


def tf_inv(R: np.ndarray, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    R_inv = R.T
    t_inv = -R_inv @ t
    return R_inv, t_inv


def average_quaternions_markley(quats_xyzw: List[np.ndarray], weights: List[float]) -> np.ndarray:
    A = np.zeros((4, 4), dtype=float)
    for q_xyzw, w in zip(quats_xyzw, weights):
        q = quat_normalize(q_xyzw)
        # Markley 用 wxyz
        qwxyz = np.array([q[3], q[0], q[1], q[2]], dtype=float)
        A += w * np.outer(qwxyz, qwxyz)

    eigvals, eigvecs = np.linalg.eigh(A)
    q_avg_wxyz = eigvecs[:, int(np.argmax(eigvals))]
    if q_avg_wxyz[0] < 0:
        q_avg_wxyz = -q_avg_wxyz
    return quat_normalize(np.array([q_avg_wxyz[1], q_avg_wxyz[2], q_avg_wxyz[3], q_avg_wxyz[0]], dtype=float))


class CubePoseFuser(Node):
    def __init__(self):
        super().__init__('cube_pose_fuser')

        # 用 ~ 默认到用户家目录，随后 resolve() 变绝对路径
        default_csv = str((Path.home() / 'markercoords.csv').resolve())

        self.declare_parameter('markers_topic', '/arucoros2/markerposes')
        self.declare_parameter('output_topic', '/cube_pose_in_car')
        self.declare_parameter('output_frame', '')  # 空：沿用输入 header.frame_id
        self.declare_parameter('markercoords_csv', default_csv)
        self.declare_parameter('weight_by_confidence', True)
        self.declare_parameter('outlier_reject', True)
        self.declare_parameter('max_trans_deviation_m', 0.15)  # 离群剔除阈值（可按需要调）

        self.markers_topic = self.get_parameter('markers_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.weight_by_conf = bool(self.get_parameter('weight_by_confidence').value)
        self.outlier_reject = bool(self.get_parameter('outlier_reject').value)
        self.max_trans_dev = float(self.get_parameter('max_trans_deviation_m').value)

        # 强制绝对路径：支持 ~，最后 resolve() 成 /home/xxx/...
        csv_raw = str(self.get_parameter('markercoords_csv').value)
        self.csv_path = str(Path(csv_raw).expanduser().resolve())

        # marker_id -> (R_cube_marker, t_cube_marker)
        self.marker_map: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
        self._load_marker_csv_abs(self.csv_path)

        self.pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.sub = self.create_subscription(MarkerArray, self.markers_topic, self.cb_markers, 10)

        self.get_logger().info(f"Subscribed: {self.markers_topic} (aruco_msgs/MarkerArray)")
        self.get_logger().info(f"Publishing: {self.output_topic}")
        self.get_logger().info(f"markercoords_csv (ABS): {self.csv_path}  loaded={len(self.marker_map)}")

    def _load_marker_csv_abs(self, abs_path: str) -> None:
        path = Path(abs_path)
        if not path.is_file():
            self.get_logger().error(f"CSV not found: {abs_path}")
            return

        try:
            with path.open('r', newline='') as f:
                # 兼容 TAB/逗号分隔
                sample = f.read(2048)
                f.seek(0)
                dialect = csv.Sniffer().sniff(sample, delimiters=[',', '\t', ';', ' '])

                reader = csv.DictReader(f, dialect=dialect)
                fields = set(reader.fieldnames or [])

                # 兼容 marker_id / id
                id_key = 'marker_id' if 'marker_id' in fields else ('id' if 'id' in fields else None)
                if id_key is None:
                    raise ValueError(f"CSV header must contain 'marker_id' (or 'id'). got={reader.fieldnames}")

                for k in ('x', 'y', 'z'):
                    if k not in fields:
                        raise ValueError(f"CSV header missing '{k}'. got={reader.fieldnames}")

                has_quat = {'qx', 'qy', 'qz', 'qw'}.issubset(fields)

                for row in reader:
                    mid = int(float(row[id_key]))  # 有些表会带 .0
                    t = np.array([float(row['x']), float(row['y']), float(row['z'])], dtype=float)

                    if has_quat:
                        q = quat_normalize(np.array([
                            float(row['qx']), float(row['qy']), float(row['qz']), float(row['qw'])
                        ], dtype=float))
                        R = quat_to_rot(q)
                    else:
                        # 你的 CSV 目前没有姿态：默认 R=I（如需更准请补 qx qy qz qw）
                        R = np.eye(3, dtype=float)

                    self.marker_map[mid] = (R, t)

        except Exception as e:
            self.get_logger().error(f"Failed to load CSV '{abs_path}': {e}")

    def cb_markers(self, msg: MarkerArray) -> None:
        if not msg.markers or not self.marker_map:
            return

        cand_t: List[np.ndarray] = []
        cand_q: List[np.ndarray] = []
        weights: List[float] = []

        for m in msg.markers:
            mid = int(m.id)
            if mid not in self.marker_map:
                continue

            # MarkerArray -> Marker.pose is PoseWithCovariance
            p = m.pose.pose.position
            o = m.pose.pose.orientation
            t_cam_marker = np.array([p.x, p.y, p.z], dtype=float)
            q_cam_marker = quat_normalize(np.array([o.x, o.y, o.z, o.w], dtype=float))
            R_cam_marker = quat_to_rot(q_cam_marker)

            R_cube_marker, t_cube_marker = self.marker_map[mid]

            # T_cam_cube = T_cam_marker * inv(T_cube_marker)
            R_marker_cube, t_marker_cube = tf_inv(R_cube_marker, t_cube_marker)
            R_cam_cube, t_cam_cube = tf_mul(R_cam_marker, t_cam_marker, R_marker_cube, t_marker_cube)
            q_cam_cube = rot_to_quat(R_cam_cube)

            w = float(m.confidence) if self.weight_by_conf else 1.0
            if not math.isfinite(w) or w <= 0.0:
                w = 1.0

            cand_t.append(t_cam_cube)
            cand_q.append(q_cam_cube)
            weights.append(w)

        if not cand_t:
            return

        # 简单离群剔除：先算一个粗均值，再剔除偏离太大的
        if self.outlier_reject and len(cand_t) >= 3:
            t0 = np.mean(np.stack(cand_t, axis=0), axis=0)
            keep = []
            for i, t in enumerate(cand_t):
                if np.linalg.norm(t - t0) <= self.max_trans_dev:
                    keep.append(i)
            if len(keep) >= 2:
                cand_t = [cand_t[i] for i in keep]
                cand_q = [cand_q[i] for i in keep]
                weights = [weights[i] for i in keep]

        wsum = float(np.sum(weights)) if float(np.sum(weights)) > 1e-12 else float(len(weights))
        if wsum <= 1e-12:
            weights = [1.0] * len(weights)
            wsum = float(len(weights))

        t_avg = np.sum([w * t for w, t in zip(weights, cand_t)], axis=0) / wsum
        q_avg = average_quaternions_markley(cand_q, weights)

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.output_frame if self.output_frame else msg.header.frame_id
        out.pose.position.x = float(t_avg[0])
        out.pose.position.y = float(t_avg[1])
        out.pose.position.z = float(t_avg[2])
        out.pose.orientation.x = float(q_avg[0])
        out.pose.orientation.y = float(q_avg[1])
        out.pose.orientation.z = float(q_avg[2])
        out.pose.orientation.w = float(q_avg[3])

        self.pub.publish(out)


def main():
    rclpy.init()
    node = CubePoseFuser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

