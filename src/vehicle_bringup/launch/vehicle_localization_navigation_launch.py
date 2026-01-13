#!/usr/bin/env python3
"""
完整的机器人定位和导航启动文件
包含：
1. 机器人状态发布 (URDF)
2. Bunker 底盘
3. RPLidar A3 激光雷达
4. HandsFree IMU
5. AMCL 定位
6. Navigation2 导航栈
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 获取包路径
    pkg_vehicle_bringup = get_package_share_directory('vehicle_bringup')
    pkg_bunker = get_package_share_directory('bunker_base')
    pkg_rplidar = get_package_share_directory('rplidar_ros')
    pkg_imu = get_package_share_directory('handsfree_ros2_imu')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 配置文件路径
    urdf_file = os.path.join(pkg_vehicle_bringup, 'description', 'vehicle.urdf')
    nav2_params_file = os.path.join(pkg_vehicle_bringup, 'config', 'nav2_params.yaml')
    default_map_file = os.path.join(pkg_vehicle_bringup, 'maps', 'map.yaml')
    rviz_config_file = os.path.join(pkg_vehicle_bringup, 'config', 'nav2_default_view.rviz')

    # 读取 URDF
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # 声明启动参数
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file to load'
    )

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='true',
        description='Use composed bringup if True'
    )

    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RVIZ'
    )

    # 获取启动配置
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_rviz = LaunchConfiguration('use_rviz')

    # 创建参数文件（动态设置 use_sim_time）
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    # ========== 机器人和传感器节点 ==========

    # 机器人状态发布者
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='both'
    )

    # Bunker 底盘
    bunker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bunker, 'launch', 'bunker_base.launch.py')
        )
    )

    # RPLidar A3
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rplidar, 'launch', 'rplidar_a3_launch.py')
        ),
        launch_arguments={'frame_id': 'laser'}.items()
    )

    # HandsFree IMU
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_imu, 'launch', 'handsfree_imu_launch.py')
        ),
        launch_arguments={
            'imu_type': 'a9',
            'frame_id': 'base_imu_link'
        }.items()
    )

    # ========== Navigation2 节点 ==========

    # AMCL 定位
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': configured_params,
            'use_composition': use_composition,
        }.items()
    )

    # Navigation2 导航栈
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': configured_params,
            'use_composition': use_composition,
        }.items()
    )

    # RViz2 可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        # 声明启动参数
        declare_map_arg,
        declare_params_file_arg,
        declare_use_sim_time_arg,
        declare_autostart_arg,
        declare_use_composition_arg,
        declare_use_rviz_arg,

        # 启动机器人和传感器
        robot_state_publisher_node,
        bunker_launch,
        rplidar_launch,
        imu_launch,

        # 启动定位和导航
        localization_launch,
        navigation_launch,
        rviz_node,
    ])
