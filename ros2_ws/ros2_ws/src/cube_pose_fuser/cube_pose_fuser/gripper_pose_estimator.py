#!/usr/bin/env python3
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from aruco_msgs.msg import MarkerArray
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs


class GripperPoseEstimator(Node):
    def __init__(self):
        super().__init__('gripper_pose_estimator')
        
        # 参数声明
        self.declare_parameter('markers_topic', '/arucoros2/markerposes')
        self.declare_parameter('output_topic', '/gripper_pose')
        self.declare_parameter('gripper_tag_ids', [0, 1, 2, 3])  # 4个tag的ID
        self.declare_parameter('gripper_tag_distance', 0.05)  # tag到中心的距离(米)
        self.declare_parameter('min_tags_required', 3)  # 最少需要几个tag
        self.declare_parameter('camera_fx', 600.0)  # 相机内参(从camera_info获取更好)
        self.declare_parameter('camera_fy', 600.0)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)
        
        # 获取参数
        self.markers_topic = self.get_parameter('markers_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.gripper_tag_ids = self.get_parameter('gripper_tag_ids').value
        self.tag_distance = float(self.get_parameter('gripper_tag_distance').value)
        self.min_tags = int(self.get_parameter('min_tags_required').value)
        
        # 相机内参
        self.fx = float(self.get_parameter('camera_fx').value)
        self.fy = float(self.get_parameter('camera_fy').value)
        self.cx = float(self.get_parameter('camera_cx').value)
        self.cy = float(self.get_parameter('camera_cy').value)
        
        # 构建抓取器tag的3D坐标（抓取器坐标系）
        # 假设：tag[0]在+X, tag[1]在+Y, tag[2]在-X, tag[3]在-Y
        d = self.tag_distance
        self.object_points = np.array([
            [0, -d, 0],   # tag_ids[0]
            [+d, 0, 0],   # tag_ids[1]
            [0, +d, 0],   # tag_ids[2]
            [-d, 0, 0],   # tag_ids[3]
        ], dtype=np.float32)
        
        # 相机内参矩阵
        self.camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.zeros(5, dtype=np.float32)  # 假设已校正，畸变为0
        
        # 发布者和订阅者
        self.pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.sub = self.create_subscription(MarkerArray, self.markers_topic, self.cb_markers, 10)
        
        self.get_logger().info(f"抓取器位姿估计节点已启动")
        self.get_logger().info(f"抓取器Tag IDs: {self.gripper_tag_ids}")
        self.get_logger().info(f"Tag距离中心: {self.tag_distance}m")
        self.get_logger().info(f"最少需要Tag数: {self.min_tags}")
    
    def cb_markers(self, msg: MarkerArray):
        if not msg.markers:
            return
        
        # 筛选出抓取器的tag
        detected_tags = {}
        for m in msg.markers:
            if int(m.id) in self.gripper_tag_ids:
                detected_tags[int(m.id)] = m
        
        num_detected = len(detected_tags)
        
        # 检查数量
        if num_detected < self.min_tags:
            # self.get_logger().warn(f"检测到{num_detected}个抓取器tag，少于最小要求{self.min_tags}")
            return
        
        # 构建PnP输入
        object_pts = []
        image_pts = []
        
        for i, tag_id in enumerate(self.gripper_tag_ids):
            if tag_id in detected_tags:
                m = detected_tags[tag_id]
                # 3D点（抓取器坐标系）
                object_pts.append(self.object_points[i])
                # 2D点（相机坐标系的3D位置，这里直接用xyz）
                image_pts.append([m.pose.pose.position.x, 
                                 m.pose.pose.position.y, 
                                 m.pose.pose.position.z])
        
        if len(object_pts) < self.min_tags:
            return
        
        object_pts = np.array(object_pts, dtype=np.float32)
        image_pts = np.array(image_pts, dtype=np.float32)
        
        # 使用solvePnP计算抓取器位姿
        # 注意：这里的image_pts实际上是3D点，需要特殊处理
        # 更好的方法是直接用3D-3D配准（ICP或Kabsch算法）
        
        # 使用Kabsch算法（更适合3D-3D配准）
        rvec, tvec = self.solve_pose_3d_3d(object_pts, image_pts)
        
        if rvec is None:
            return
        
        # 转换为四元数
        rot_mat, _ = cv2.Rodrigues(rvec)
        quat = self.rotation_matrix_to_quaternion(rot_mat)
        
        # 发布PoseStamped
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.pose.position.x = float(tvec[0])
        out.pose.position.y = float(tvec[1])
        out.pose.position.z = float(tvec[2])
        out.pose.orientation.x = float(quat[0])
        out.pose.orientation.y = float(quat[1])
        out.pose.orientation.z = float(quat[2])
        out.pose.orientation.w = float(quat[3])
        
        self.pub.publish(out)
        
        self.get_logger().info(f"抓取器位姿: pos=[{tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}], "
                              f"使用{len(object_pts)}个tag")
    
    def solve_pose_3d_3d(self, src_pts, dst_pts):
        """
        3D-3D点云配准（Kabsch算法）
        src_pts: 抓取器坐标系的点
        dst_pts: 相机坐标系的点
        返回: rvec, tvec (抓取器在相机坐标系的位姿)
        """
        if len(src_pts) < 3:
            return None, None
        
        # 计算质心
        centroid_src = np.mean(src_pts, axis=0)
        centroid_dst = np.mean(dst_pts, axis=0)
        
        # 去中心化
        src_centered = src_pts - centroid_src
        dst_centered = dst_pts - centroid_dst
        
        # 计算协方差矩阵
        H = src_centered.T @ dst_centered
        
        # SVD分解
        U, S, Vt = np.linalg.svd(H)
        
        # 计算旋转矩阵
        R = Vt.T @ U.T
        
        # 处理反射情况
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # 确保Z轴向上
        z_axis = R[:, 2]
        if z_axis[2] < 0:  # 如果Z轴向下，翻转
            # 绕X轴旋转180度
            flip = np.array([
                [1, 0, 0],
                [0, -1, 0],
                [0, 0, -1]
            ], dtype=np.float32)
            R = R @ flip
        
        # 计算平移向量
        t = centroid_dst - R @ centroid_src
        
        # 转换为rvec
        rvec, _ = cv2.Rodrigues(R)
        tvec = t.reshape(3, 1)
        
        return rvec, tvec
    
    def rotation_matrix_to_quaternion(self, R):
        """旋转矩阵转四元数 (xyzw格式)"""
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        
        return np.array([x, y, z, w], dtype=np.float32)


def main():
    rclpy.init()
    node = GripperPoseEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()