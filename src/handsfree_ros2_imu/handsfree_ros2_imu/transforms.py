#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
ROS2 坐标转换工具函数
使用 scipy.spatial.transform.Rotation 实现
"""
from scipy.spatial.transform import Rotation


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> list:
    """
    欧拉角转四元数 (RPY -> Quaternion)
    
    使用 'xyz' 内旋顺序 (等同于 ROS2 的 static frame RPY)
    
    Args:
        roll: 绕 X 轴旋转角度 (弧度)
        pitch: 绕 Y 轴旋转角度 (弧度)
        yaw: 绕 Z 轴旋转角度 (弧度)
    
    Returns:
        [x, y, z, w] 四元数
    """
    r = Rotation.from_euler('xyz', [roll, pitch, yaw])
    q = r.as_quat()  # 返回 [x, y, z, w] 顺序
    return q.tolist()


def euler_from_quaternion(quaternion: list) -> tuple:
    """
    四元数转欧拉角 (Quaternion -> RPY)
    
    Args:
        quaternion: [x, y, z, w] 四元数
    
    Returns:
        (roll, pitch, yaw) 欧拉角 (弧度)
    """
    r = Rotation.from_quat(quaternion)  # scipy 使用 [x, y, z, w] 顺序
    euler = r.as_euler('xyz')
    return tuple(euler)

