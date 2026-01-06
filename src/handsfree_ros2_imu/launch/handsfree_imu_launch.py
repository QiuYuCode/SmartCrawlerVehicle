#!/usr/bin/env python3
"""
HandsFree IMU ROS2 Launch File
支持 a9, b6, b9 三种 IMU 型号
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # 声明启动参数
    imu_type_arg = DeclareLaunchArgument(
        'imu_type',
        default_value='a9',
        description='IMU 型号 [a9, b6, b9]'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/HFRobotIMU',
        description='IMU 串口设备路径'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='921600',
        description='串口波特率'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='base_link',
        description='IMU frame_id'
    )
    
    gra_normalization_arg = DeclareLaunchArgument(
        'gra_normalization',
        default_value='true',
        description='A9 重力加速度归一化处理 (仅 A9 有效)'
    )
    
    # A9 IMU 节点
    hfi_a9_node = Node(
        package='handsfree_ros2_imu',
        executable='hfi_a9_node',
        name='hfi_a9_imu',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'gra_normalization': LaunchConfiguration('gra_normalization'),
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('imu_type'), "' == 'a9'"])
        )
    )
    
    # B6 IMU 节点
    hfi_b6_node = Node(
        package='handsfree_ros2_imu',
        executable='hfi_b6_node',
        name='hfi_b6_imu',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('imu_type'), "' == 'b6'"])
        )
    )
    
    # B9 IMU 节点
    hfi_b9_node = Node(
        package='handsfree_ros2_imu',
        executable='hfi_b9_node',
        name='hfi_b9_imu',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('imu_type'), "' == 'b9'"])
        )
    )
    
    return LaunchDescription([
        imu_type_arg,
        port_arg,
        baudrate_arg,
        frame_id_arg,
        gra_normalization_arg,
        hfi_a9_node,
        hfi_b6_node,
        hfi_b9_node,
    ])

