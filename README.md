# Smart Crawler Vehicle

基于 AgileX Bunker mini 移动机器人平台的 ROS2 智能履带车系统，集成了多传感器融合和自主导航功能。

## 目录

- [系统概述](#系统概述)
- [硬件平台](#硬件平台)
- [软件架构](#软件架构)
- [快速开始](#快速开始)
- [功能模块](#功能模块)
- [常见问题](#常见问题)

## 系统概述

本项目是一个完整的移动机器人系统，支持：

- 自主导航和 SLAM 建图
- 多传感器融合（激光雷达、IMU、相机）
- 自定义执行器控制（升降机构）
- CAN 总线底盘控制
- ROS2 标准接口

## 硬件平台

### 主要组件

| 组件 | 型号 | 接口类型 | 功能 |
|------|------|----------|------|
| 移动底盘 | AgileX Bunker mini | CAN 总线 | 差速驱动，运动控制 |
| 激光雷达 | SLAMTEC RPLidar A3 | USB | 2D 激光扫描，建图定位 |
| IMU | HandsFree A9 | 串口 | 姿态检测，里程计融合 |
| 相机 | Intel RealSense | USB 3.0 | 深度视觉，环境感知 |
| 升降执行器 | RSA86E 步进电机 | RS485/Modbus RTU | 垂直运动控制 |

### 连接说明

- **CAN 总线**：USB-CAN 适配器连接至 Bunker 底盘
- **串口设备**：
  - IMU：串口连接（自动检测）
  - 升降电机：`/dev/ttyS0`（Modbus RTU，波特率 9600）
- **USB 设备**：RPLidar、RealSense 相机

## 软件架构

### 包结构

```
SmartCrawlerVehicle/
├── src/
│   ├── ugv_sdk/                    # AgileX UGV 底层 SDK（子模块）
│   ├── bunker_ros2/                # Bunker mini 底盘 ROS2 驱动（子模块）
│   ├── rplidar_ros/                # RPLidar 激光雷达驱动（子模块）
│   ├── realsense-ros/              # RealSense 相机驱动（子模块）
│   ├── vehicle_bringup/            # 系统启动和配置包
│   ├── lift_motor/                 # 升降电机控制包
│   └── handsfree_ros2_imu/         # HandsFree IMU 驱动包
└── README.md                       # 本文档
```

### 关键话题

| 话题名称 | 消息类型 | 方向 | 说明 |
|----------|----------|------|------|
| `/cmd_vel` | geometry_msgs/Twist | 输入 | 底盘速度控制命令 |
| `/scan` | sensor_msgs/LaserScan | 输出 | 激光雷达扫描数据 |
| `/imu` | sensor_msgs/Imu | 输出 | IMU 姿态数据 |
| `/mag` | sensor_msgs/MagneticField | 输出 | 磁力计数据 |
| `/lift_motor/cmd` | std_msgs/String | 输入 | 升降电机命令 |

### TF 树结构

```
map
 └─ odom
     └─ base_link
         ├─ laser
         ├─ base_imu_link
         └─ camera_link
```

## 快速开始

**机器人已经安装配置了代码，不需要执行下面的步骤，下面的步骤是给新环境小车用的！**

### 1. 环境配置


```bash
# 安装 ROS2 依赖（首次安装）
sudo apt-get update
sudo apt-get install build-essential git cmake libasio-dev \
  ros-humble-desktop ros-humble-slam-toolbox

# 克隆仓库（包含子模块）
git clone --recurse-submodules <repository_url>
cd SmartCrawlerVehicle

# 如果已克隆但未初始化子模块
git submodule update --init --recursive
```

### 2. 编译工作空间

```bash
# 编译所有包
colcon build --symlink-install

# 加载环境
source install/setup.bash
```

### 3. 启动系统

```bash
# 启动完整系统（包含 SLAM）
ros2 launch vehicle_bringup vehicle_slam_launch.py

# 在新终端中测试移动，通过键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

## 功能模块

### 1. Bunker Mini 底盘控制

#### 启动底盘节点

```bash
# 单独启动底盘驱动
ros2 launch bunker_base bunker_base.launch.py
```

#### 键盘控制

```bash
# 在新终端中测试移动，通过键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### 查看底盘状态

```bash
# 监听里程计数据
ros2 topic echo /odom

# 监听 CAN 原始数据
candump can0

# 查看底盘 TF 变换
ros2 run tf2_ros tf2_echo odom base_link
```

### 2. RPLidar 激光雷达

#### 启动激光雷达

```bash
# 启动 RPLidar A3
ros2 launch rplidar_ros rplidar_a3_launch.py

# 启动并可视化
ros2 launch rplidar_ros view_rplidar_a3_launch.py

# 自定义串口设备
ros2 launch rplidar_ros rplidar_a3_launch.py serial_port:=/dev/raplidar
```

#### 查看扫描数据

```bash
# 查看原始扫描数据
ros2 topic echo /scan

# 查看扫描频率
ros2 topic hz /scan

# 在 RViz2 中可视化
rviz2 -d src/rplidar_ros/rviz/rplidar.rviz
```

#### 激光雷达参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/rplidar` | 串口设备路径 |
| `serial_baudrate` | `256000` | 波特率 |
| `frame_id` | `laser` | TF 坐标系名称 |
| `scan_mode` | `Standard` | 扫描模式（A3 支持多种模式） |

### 3. HandsFree IMU

#### 启动 IMU 节点

```bash
# 启动默认配置
ros2 run handsfree_ros2_imu imu_node

# 指定串口和波特率
ros2 run handsfree_ros2_imu imu_node \
  --ros-args \
  -p serial_port:=/dev/HFRobotIMU \
  -p serial_baud:=115200
```

#### 查看 IMU 数据

```bash
# 查看 IMU 数据
ros2 topic echo /imu

# 查看磁力计数据
ros2 topic echo /mag

# 查看数据频率
ros2 topic hz /imu

# 可视化姿态
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_imu_link
```

#### IMU 参数配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/HFRobotIMU` | 串口设备 |
| `serial_baud` | `115200` | 波特率 |
| `imu_frame_id` | `base_imu_link` | IMU 坐标系 |
| `imu_type` | `A9` | IMU 型号（A9/B6/B9） |

### 4. 升降电机控制

#### 启动电机控制节点

```bash
# 使用默认配置启动
ros2 run lift_motor motor_control

# 指定串口和参数
ros2 run lift_motor motor_control \
  --ros-args \
  -p port:=/dev/ttyS0 \
  -p default_speed:=60
```

#### 电机控制命令

```bash
# 回到原点（零位）
ros2 topic pub /lift_motor/cmd std_msgs/String "{data: 'origin'}" --once

# 绝对位置移动（相对于零点，移动 5000 个脉冲）
ros2 topic pub /lift_motor/cmd std_msgs/String "{data: 'abs 5000'}" --once

# 相对位置移动（向上移动 5000 个脉冲）
ros2 topic pub /lift_motor/cmd std_msgs/String "{data: 'move 5000'}" --once

# 相对位置移动（向下移动 5000 个脉冲）
ros2 topic pub /lift_motor/cmd std_msgs/String "{data: 'move -5000'}" --once

# 查询当前电机驱动器状态
ros2 topic pub /lift_motor/cmd std_msgs/String "{data: 'status'}" --once
ros2 topic echo /lift_motor/status

# 使能/失能电机
ros2 topic pub /lift_motor/cmd std_msgs/String "{data: 'enable'}" --once
ros2 topic pub /lift_motor/cmd std_msgs/String "{data: 'disable'}" --once

# 紧急停止
ros2 topic pub /lift_motor/cmd std_msgs/String "{data: 'stop'}" --once
```

#### 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port` | `/dev/ttyS0` | 串口设备路径 |
| `baudrate` | `9600` | Modbus 波特率 |
| `slave_id` | `1` | Modbus 从机地址 |
| `default_speed` | `60` | 默认速度（RPM） |


### 5. RealSense 相机

#### 启动相机节点

```bash
# 启动默认配置
ros2 launch realsense2_camera rs_launch.py

# 启动并启用深度对齐到彩色图像
ros2 launch realsense2_camera rs_launch.py align_depth:=true

# 自定义分辨率和帧率
ros2 launch realsense2_camera rs_launch.py \
  depth_width:=640 \
  depth_height:=480 \
  depth_fps:=30
```

#### 查看相机数据

```bash
# 查看彩色图像
ros2 topic echo /camera/color/image_raw

# 查看深度图像
ros2 topic echo /camera/depth/image_rect_raw

# 查看相机信息
ros2 topic echo /camera/color/camera_info

# 在 RViz2 中可视化
rviz2
# 添加 Image 显示，话题选择 /camera/color/image_raw 或 /camera/depth/image_rect_raw
```

#### 点云处理

```bash
# 查看点云数据
ros2 topic echo /camera/depth/color/points

# 保存点云
ros2 run pcl_ros pointcloud_to_pcd input:=/camera/depth/color/points
```

### 6. SLAM 建图与导航

#### 启动 SLAM 建图

```bash
# 启动完整 SLAM 系统
ros2 launch vehicle_bringup vehicle_slam_launch.py

# 在 RViz2 中可视化
rviz2 -d src/vehicle_bringup/rviz/vehicle_slam.rviz  # 如果有配置文件
```

#### 保存地图

遥控小车，确保地图完成创建后，保存地图到指定路径
![建图后保存地图](./docs/建图后保存地图.png)

#### SLAM 参数调整

配置文件：`src/vehicle_bringup/config/mapper_params_online_async.yaml`

关键参数：
- `resolution`：地图分辨率（默认 0.05m）
- `map_update_interval`：地图更新间隔（默认 5.0s）
- `throttle_scans`：跳过扫描数（降低计算负载）

### 7. 键盘遥控

```bash
# 安装 teleop_twist_keyboard
sudo apt-get install ros-humble-teleop-twist-keyboard

# 启动键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 按键说明：
# u i o     前进并左转/前进/前进并右转
# j k l     原地左转/停止/原地右转
# m , .     后退并左转/后退/后退并右转
# q/z       增加/减少速度
```

## 常见问题

### Q1: CAN 总线连接失败

**症状**：`candump can0` 无输出，或提示 "Device or resource busy"

**解决方案**：
```bash
# 关闭 CAN 接口
sudo ip link set can0 down

# 检查接口状态
ip -details link show can0
```

### Q2: 底盘不响应运动命令

**可能原因**：
1. 硬件急停按钮未释放
2. 遥控器模式未切换到命令控制模式
3. CAN 总线未正确配置

**检查步骤**：
```bash
# 1. 检查 CAN 数据流
candump can0

# 2. 检查话题连接
ros2 topic info /cmd_vel

# 3. 手动发送测试命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 0.1, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 0.0}" --rate 10
```

### Q3: RPLidar 无法启动

**症状**：找不到设备或权限被拒绝

**解决方案**：

优先检查设备的硬件连接

```bash
# 查找 RPLidar 设备
ls -l /dev/rplidar

# 添加用户到 dialout 组
sudo usermod -aG dialout $USER

# 注销并重新登录，或重启系统
```

### Q4: 升降电机不移动

**可能原因**：
1. 串口设备路径错误
2. 电机未使能

**检查步骤**：
```bash
# 1. 确认串口设备
ls -l /dev/ttyS*

# 2. 查看节点日志
ros2 run lift_motor motor_control --ros-args --log-level DEBUG

# 3. 使能电机
ros2 topic pub /lift_motor/cmd std_msgs/String "data: 'enable'" --once

# 4. 测试小范围移动
ros2 topic pub /lift_motor/cmd std_msgs/String "data: 'move 5000'" --once
```

### Q5: IMU 数据不稳定

**解决方案**：
1. 检查串口波特率是否匹配
2. 确保 IMU 安装牢固，远离电磁干扰
3. 校准 IMU（如果设备支持）

```bash
# 检查数据频率
ros2 topic hz /imu

# 查看原始数据
ros2 topic echo /imu
```

### Q6: SLAM 建图效果差

**优化建议**：
1. 降低移动速度，缓慢移动
2. 在特征丰富的环境中建图
3. 调整 SLAM 参数：
   - 降低 `resolution`（提高精度）
   - 增加 `map_update_interval`（减轻 CPU 负担）
   - 检查激光雷达安装是否水平稳定

### Q7: 编译错误

```bash
# 清理编译缓存
rm -rf build/ install/ log/

# 检查依赖
rosdepc install --from-paths src --ignore-src -r -y

# 重新编译
colcon build --symlink-install --event-handlers console_direct+
```

### Q8: 远程连接

在自己的电脑上安装 NoMachine 或者 todesk

## 开发指南

### 编译特定包

```bash
# 只编译单个包
colcon build --packages-select vehicle_bringup

# 编译包及其依赖
colcon build --packages-up-to vehicle_bringup

# 跳过某些包
colcon build --packages-skip realsense2_camera
```

### 调试技巧

```bash
# 查看节点列表
ros2 node list

# 查看话题列表
ros2 topic list

# 查看服务列表
ros2 service list

# 查看 TF 树
ros2 run tf2_tools view_frames

# 录制数据包
ros2 bag record -a  # 录制所有话题
ros2 bag record /scan /imu /odom  # 录制指定话题

# 回放数据包
ros2 bag play <bag_file>
```

### 修改 URDF

机器人描述文件位于 `src/vehicle_bringup/description/vehicle.urdf`，修改后需要：

```bash
# 重新编译
colcon build --packages-select vehicle_bringup

# 检查 URDF 语法
check_urdf src/vehicle_bringup/description/vehicle.urdf

# 查看 TF 树
ros2 run tf2_tools view_frames
```


## 许可证

请参考各子模块的许可证信息。

## 联系方式

如有问题，请提交 Issue 或联系维护者。

---

**安全提醒**：测试运动命令时务必准备好无线遥控器，确保能随时接管控制！
