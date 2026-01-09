import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取各个包的路径
    pkg_vehicle_bringup = get_package_share_directory('vehicle_bringup')
    
    # --- 修改点：这里将 bunker_ros2 改为 bunker_base ---
    pkg_bunker = get_package_share_directory('bunker_base') 
    
    pkg_rplidar = get_package_share_directory('rplidar_ros')
    pkg_imu = get_package_share_directory('handsfree_ros2_imu')
    pkg_slam = get_package_share_directory('slam_toolbox')

    # 2. 配置文件路径
    urdf_file = os.path.join(pkg_vehicle_bringup, 'description', 'vehicle.urdf')
    slam_config_file = os.path.join(pkg_vehicle_bringup, 'config', 'mapper_params_online_async.yaml')

    # 3. 读取 URDF 内容
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # --- 节点定义 ---

    # A. 机器人状态发布者
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='both'
    )

    # B. 启动底盘 (Bunker)
    # 注意：通常 bunker_base 包里的启动文件是 bunker_base.launch.py
    # 如果再次报错找不到文件，请去 install/bunker_base/share/bunker_base/launch/ 下确认文件名
    bunker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bunker, 'launch', 'bunker_base.launch.py') 
        )
    )

    # C. 启动雷达 (RPLidar)
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rplidar, 'launch', 'rplidar_a3_launch.py')
        ),
        launch_arguments={'frame_id': 'laser'}.items()
    )

    # D. 启动 IMU (HandsFree)
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_imu, 'launch', 'handsfree_imu_launch.py')
        ),
        launch_arguments={
            'imu_type': 'a9',
            'frame_id': 'base_imu_link'
        }.items()
    )

    # E. 启动 SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': slam_config_file}.items()
    )

    return LaunchDescription([
        robot_state_publisher_node,
        bunker_launch,
        rplidar_launch,
        imu_launch,
        slam_launch,
    ])