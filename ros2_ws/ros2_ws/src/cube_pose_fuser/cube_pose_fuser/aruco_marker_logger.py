#!/usr/bin/env python3
import csv
import time
from pathlib import Path
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from aruco_msgs.msg import MarkerArray


class ArucoMarkerLogger(Node):
    def __init__(self):
        super().__init__('aruco_marker_logger')

        # ---- parameters ----
        self.declare_parameter('marker_topic', '/marker_publisher/markers')
        self.declare_parameter(
            'output_file',
            '/home/nvidia/ros2_ws/src/cube_pose_fuser/aruco_markers_data/aruco_markers.csv'
        )
        self.declare_parameter('append', True)
        self.declare_parameter('write_covariance', False)

        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.output_file = Path(self.get_parameter('output_file').get_parameter_value().string_value)
        self.append = self.get_parameter('append').get_parameter_value().bool_value
        self.write_cov = self.get_parameter('write_covariance').get_parameter_value().bool_value

        # ensure dir exists
        self.output_file.parent.mkdir(parents=True, exist_ok=True)

        # 判断是否需要写表头（新文件/空文件/或append=False）
        existed = self.output_file.exists()
        empty = (not existed) or (self.output_file.stat().st_size == 0)

        # open file
        mode = 'a' if self.append else 'w'
        # csv 官方建议：如果是文件对象，最好 open(..., newline='') :contentReference[oaicite:1]{index=1}
        self._fp = self.output_file.open(mode, newline='')
        self._writer = csv.writer(self._fp)

        # header
        self._columns = [
            'msg_stamp_sec', 'msg_stamp_nanosec', 'msg_stamp_ns', 'msg_stamp_iso_utc',
            'recv_wall_time_ns',
            'frame_id',
            'marker_id',
            'px', 'py', 'pz',
            'qx', 'qy', 'qz', 'qw',
            'confidence',
        ]
        if self.write_cov:
            self._columns += [f'cov_{i}' for i in range(36)]

        if (not self.append) or empty:
            self._writer.writerow(self._columns)
            self._fp.flush()

        # subscription
        self._sub = self.create_subscription(
            MarkerArray,
            self.marker_topic,
            self._cb,
            qos_profile_sensor_data
        )

        self.get_logger().info(f'Subscribing: {self.marker_topic}')
        self.get_logger().info(f'Logging to:   {self.output_file} (append={self.append})')

    def _cb(self, msg: MarkerArray):
        # 每个 Marker 写一行
        for m in msg.markers:
            # aruco_msgs/Marker: header、id、PoseWithCovariance pose、confidence :contentReference[oaicite:2]{index=2}
            stamp = m.header.stamp
            sec = int(stamp.sec)
            nsec = int(stamp.nanosec)
            stamp_ns = sec * 1_000_000_000 + nsec
            stamp_iso = datetime.fromtimestamp(sec + nsec * 1e-9, tz=timezone.utc).isoformat()

            recv_wall_ns = time.time_ns()  # 本机回调时间（方便算延迟/抖动）

            p = m.pose.pose.position
            q = m.pose.pose.orientation

            row = [
                sec, nsec, stamp_ns, stamp_iso,
                recv_wall_ns,
                m.header.frame_id,
                int(m.id),
                float(p.x), float(p.y), float(p.z),
                float(q.x), float(q.y), float(q.z), float(q.w),
                float(m.confidence),
            ]

            if self.write_cov:
                cov = list(m.pose.covariance)
                cov = (cov + [0.0] * 36)[:36]
                row += [float(x) for x in cov]

            self._writer.writerow(row)

        self._fp.flush()

    def destroy_node(self):
        try:
            self._fp.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

