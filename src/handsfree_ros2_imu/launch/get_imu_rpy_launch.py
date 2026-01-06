#!/usr/bin/env python3
"""
Get IMU RPY Launch File
订阅 IMU 数据并打印 RPY 角度
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明启动参数
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/handsfree/imu',
        description='IMU 话题名称'
    )
    
    # RPY 显示节点
    get_imu_rpy_node = Node(
        package='handsfree_ros2_imu',
        executable='get_imu_rpy',
        name='get_imu_rpy',
        output='screen',
        parameters=[{
            'imu_topic': LaunchConfiguration('imu_topic'),
        }],
    )
    
    return LaunchDescription([
        imu_topic_arg,
        get_imu_rpy_node,
    ])

