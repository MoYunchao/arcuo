#!/usr/bin/env python3
"""
绳子控制键盘节点
功能：通过键盘按键发送控制信号
按键：1=收紧, 2=放松, 3=固定, q=退出
"""

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


class RopeKeyboardControl(Node):
    def __init__(self):
        super().__init__('rope_keyboard_control')
        
        # 创建发布者，发布到 /rope_command topic
        self.pub = self.create_publisher(UInt8, '/rope_command', 10)
        
        # 打印使用说明
        self.print_instructions()
        
        # 命令映射：按键 -> (名称, 数值)
        self.commands = {
            '1': ('收紧绳子', 1),
            '2': ('放松绳子', 2),
            '3': ('固定绳子', 3),
        }
    
    def print_instructions(self):
        """打印使用说明"""
        print("
" + "=" * 60)
        print("           绳子控制键盘节点已启动")
        print("=" * 60)
        print("\n按键说明：")
        print("  [1] - 收紧绳子")
        print("  [2] - 放松绳子")
        print("  [3] - 固定绳子")
        print("  [q] - 退出程序")
        print("\n" + "=" * 60)
        print("等待按键输入...
")
    
    def send_command(self, name, value):
        """发送控制命令"""
        msg = UInt8()
        msg.data = value
        self.pub.publish(msg)
        
        # 打印发送信息
        timestamp = self.get_clock().now().to_msg()
        print(f"[{timestamp.sec}.{timestamp.nanosec:09d}] >>> 发送命令: {name} (值={value})")
    
    def run(self):
        """主循环：监听键盘输入"""
        # 保存原始终端设置
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            # 设置终端为原始模式（不需要按回车，直接响应按键）
            tty.setraw(fd)
            
            print("准备就绪，请按键...
")
            
            while rclpy.ok():
                # 读取单个字符
                key = sys.stdin.read(1)
                
                # 处理按键
                if key == 'q' or key == 'Q':
                    print("\n退出键盘控制节点")
                    break
                    
                elif key in self.commands:
                    # 有效命令
                    name, value = self.commands[key]
                    self.send_command(name, value)
                    
                elif key == '\x03':  # Ctrl+C
                    print("\n检测到 Ctrl+C，退出")
                    break
                    
                elif key == '
' or key == '
':
                    # 忽略回车
                    pass
                    
                else:
                    # 无效按键
                    print(f"无效按键: '{key}' (ASCII: {ord(key)}), 请按 1/2/3 或 q")
                    
        except Exception as e:
            print(f"\n错误: {e}")
            
        finally:
            # 恢复终端设置
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print("终端设置已恢复")


def main():
    """主函数"""
    rclpy.init()
    node = RopeKeyboardControl()
    
    try:
        node.run()
    except KeyboardInterrupt:
        print("
程序被中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()