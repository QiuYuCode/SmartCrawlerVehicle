#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
订阅 IMU 数据并显示 RPY 角度
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
import math


class GetImuRPY(Node):
    """订阅 IMU 话题并打印 RPY"""
    
    def __init__(self):
        super().__init__('get_imu_rpy')
        
        # 声明参数
        self.declare_parameter('imu_topic', '/handsfree/imu')
        
        # 获取参数
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10
        )
    
    def imu_callback(self, msg: Imu):
        """IMU 数据回调"""
        # 四元数转欧拉角
        orientation = msg.orientation
        (roll, pitch, yaw) = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])
        
        # 弧度转角度
        roll_deg = roll * 180.0 / math.pi
        pitch_deg = pitch * 180.0 / math.pi
        yaw_deg = yaw * 180.0 / math.pi
        
        self.get_logger().info(f'Roll = {roll_deg:.2f}, Pitch = {pitch_deg:.2f}, Yaw = {yaw_deg:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = GetImuRPY()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

