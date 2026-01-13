#!/usr/bin/env python3
"""
Navigation2 启动文件 - 用于 Bunker 履带车自主导航
前提：需要已有建图完成的地图文件
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
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 配置文件路径
    nav2_params_file = os.path.join(pkg_vehicle_bringup, 'config', 'nav2_params.yaml')
    default_map_file = os.path.join(pkg_vehicle_bringup, 'maps', 'map.yaml')
    rviz_config_file = os.path.join(pkg_vehicle_bringup, 'config', 'nav2_default_view.rviz')

    # 声明启动参数
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file to load'
    )

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
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

    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    # 获取启动配置
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_rviz = LaunchConfiguration('use_rviz')
    namespace = LaunchConfiguration('namespace')

    # 创建参数文件（动态设置 use_sim_time）
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    # 启动 Localization (AMCL)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': configured_params,
            'use_composition': use_composition,
        }.items()
    )

    # 启动 Navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': configured_params,
            'use_composition': use_composition,
        }.items()
    )

    # 启动 RViz2
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
        declare_namespace_arg,

        # 启动节点
        localization_launch,
        navigation_launch,
        rviz_node,
    ])
