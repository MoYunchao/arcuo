#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import yaml
import os

class CameraInfoOverride(Node):
    def __init__(self):
        super().__init__('camera_info_override')
        
        # 声明参数
        self.declare_parameter('camera_name', 'infra1')
        self.declare_parameter('camera_info_url', '') 
        self.declare_parameter('image_topic', '/camera/camera/infra1/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/infra1/camera_info_calib')

        # 获取参数值
        self.camera_name = self.get_parameter('camera_name').value
        url = self.get_parameter('camera_info_url').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value

        # --- 核心修改：手动解析文件路径 ---
        if not url:
            self.get_logger().error("camera_info_url is empty!")
            raise RuntimeError("camera_info_url is empty")
        
        # 去掉 file:// 前缀，获取真实路径
        file_path = url.replace('file://', '').replace('file:', '')
        
        self.get_logger().info(f"Attempting to load yaml from: {file_path}")

        # --- 核心修改：直接加载 YAML ---
        self.camera_info_msg = None
        try:
            with open(file_path, 'r') as f:
                calib_data = yaml.safe_load(f)
                self.camera_info_msg = self.parse_yaml_to_camerainfo(calib_data)
                self.get_logger().info("Successfully loaded camera info manually!")
        except Exception as e:
            self.get_logger().error(f"Failed to open/parse YAML file: {e}")
            raise RuntimeError(f"Could not load camera info: {e}")

        # 创建发布者和订阅者
        self.pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)

        self.get_logger().info(f"Sub image: {self.image_topic}")
        self.get_logger().info(f"Pub camera_info: {self.camera_info_topic}")

    def parse_yaml_to_camerainfo(self, yaml_data):
        """将 YAML 字典转换为 CameraInfo 消息"""
        ci = CameraInfo()
        # 基础参数
        ci.width = yaml_data.get('image_width', 0)
        ci.height = yaml_data.get('image_height', 0)
        ci.distortion_model = yaml_data.get('distortion_model', 'plumb_bob')
        
        # 矩阵参数 (YAML 中通常是 dict: {rows, cols, data})
        # D: 畸变系数 (直接是列表或 dict)
        d_data = yaml_data.get('distortion_coefficients', {}).get('data', [])
        ci.d = [float(x) for x in d_data]

        # K: 内参矩阵 (3x3)
        k_data = yaml_data.get('camera_matrix', {}).get('data', [])
        ci.k = [float(x) for x in k_data]

        # R: 旋转/校正矩阵 (3x3)
        r_data = yaml_data.get('rectification_matrix', {}).get('data', [])
        ci.r = [float(x) for x in r_data]

        # P: 投影矩阵 (3x4)
        p_data = yaml_data.get('projection_matrix', {}).get('data', [])
        ci.p = [float(x) for x in p_data]

        return ci

    def on_image(self, msg: Image):
        if self.camera_info_msg is not None:
            # 关键：同步时间戳和 Frame ID
            self.camera_info_msg.header = msg.header
            self.pub.publish(self.camera_info_msg)

def main():
    rclpy.init()
    try:
        node = CameraInfoOverride()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node stopped cleanly: {e}")
    finally:
        # node.destroy_node() # 有时会导致异常，可省略
        rclpy.shutdown()

if __name__ == '__main__':
    main()
