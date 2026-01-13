# Navigation2 配置说明

## 文件概述

本目录包含 Navigation2 导航系统的配置文件：

- **nav2_params.yaml** - Navigation2 主配置文件，包含所有导航节点的参数
- **mapper_params_online_async.yaml** - SLAM Toolbox 建图配置
- **nav2_default_view.rviz** - RViz 可视化配置（待创建）

## 配置文件结构

### nav2_params.yaml

该文件包含以下主要部分：

#### 1. AMCL 定位 (Adaptive Monte Carlo Localization)
- 使用粒子滤波进行机器人定位
- 配置了适合 Bunker 履带车的运动模型参数
- 优化了激光雷达参数以适配 RPLidar A3

#### 2. 控制器服务器 (Controller Server)
- 使用 DWB (Dynamic Window Approach) 局部规划器
- 配置了差速驱动机器人的速度和加速度限制
- 针对履带式平台优化了轨迹评价函数

#### 3. 代价地图 (Costmaps)
- **局部代价地图**: 5m x 5m 滚动窗口，用于局部避障
- **全局代价地图**: 使用完整地图，用于全局路径规划
- 两者都配置了体素层(VoxelLayer)和膨胀层(InflationLayer)

#### 4. 规划器服务器 (Planner Server)
- 使用 NavFn 全局路径规划器
- 基于 Dijkstra 算法

#### 5. 行为服务器 (Behavior Server)
- 提供恢复行为：旋转、后退、等待等
- 用于处理机器人被困的情况

#### 6. 其他组件
- **路径平滑器**: 平滑全局路径
- **速度平滑器**: 平滑速度命令
- **碰撞监测器**: 实时碰撞检测和避让
- **航点跟随器**: 支持多点导航

## 关键参数调优指南

### 机器人尺寸参数

当前配置基于 Bunker 履带车：
```yaml
# 机器人尺寸：0.6m x 0.4m
robot_radius: 0.35  # 保守估计的机器人半径
inflation_radius: 0.55  # 障碍物膨胀半径
```

**调整建议**：如果机器人频繁误判碰撞或无法通过狭窄通道，可调整这两个参数。

### 速度和加速度限制

```yaml
max_vel_x: 0.5      # 最大前进速度 (m/s)
max_vel_theta: 1.0  # 最大旋转速度 (rad/s)
acc_lim_x: 0.5      # X 方向加速度限制 (m/s²)
acc_lim_theta: 1.0  # 旋转加速度限制 (rad/s²)
```

**调整建议**：
- 提高速度可加快导航，但需要确保机器人能安全制动
- 降低加速度可使运动更平滑，但响应会变慢
- 这些值应与 Bunker 底盘的物理限制匹配

### 目标容差

```yaml
xy_goal_tolerance: 0.15    # 位置容差 (m)
yaw_goal_tolerance: 0.2    # 朝向容差 (rad)
```

**调整建议**：
- 增大容差可使机器人更容易到达目标
- 减小容差可提高定位精度，但可能导致振荡

### DWB 轨迹评价权重

```yaml
critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
BaseObstacle.scale: 0.02   # 障碍物避让权重
PathAlign.scale: 32.0      # 路径跟踪权重
GoalAlign.scale: 24.0      # 目标对齐权重
```

**调整建议**：
- 增大 `BaseObstacle.scale` 可使机器人更保守地避障
- 增大 `PathAlign.scale` 可使机器人更紧密地跟随路径
- 增大 `GoalAlign.scale` 可使机器人更早开始调整朝向

### 代价地图更新频率

```yaml
# 局部代价地图
update_frequency: 5.0   # 更新频率 (Hz)
publish_frequency: 2.0  # 发布频率 (Hz)

# 全局代价地图
update_frequency: 1.0
publish_frequency: 1.0
```

**调整建议**：
- 在性能受限的平台（如树莓派）上，可降低这些频率
- 在高速运动场景下，可提高局部代价地图的更新频率

## 使用场景

### 1. 标准自主导航

使用带地图的定位和导航：
```bash
ros2 launch vehicle_bringup vehicle_localization_navigation_launch.py
```

### 2. 仅导航（已有定位）

如果已经有其他定位源（如 SLAM），只启动导航栈：
```bash
ros2 launch vehicle_bringup vehicle_navigation_launch.py
```

### 3. 自定义地图

使用特定地图文件：
```bash
ros2 launch vehicle_bringup vehicle_localization_navigation_launch.py \
    map:=/path/to/your/map.yaml
```

### 4. 调试模式

禁用自动启动，手动启动各个组件：
```bash
ros2 launch vehicle_bringup vehicle_navigation_launch.py autostart:=false
```

然后手动配置和启动生命周期节点。

## 常见问题排查

### 1. 机器人不移动
- 检查 CAN 总线是否正确配置
- 确认 Bunker 底盘急停按钮已释放
- 检查 `/cmd_vel` 话题是否有消息发布

### 2. 定位不准确
- 确保地图质量良好
- 调整 AMCL 粒子数量（`min_particles`, `max_particles`）
- 检查激光雷达数据是否正常

### 3. 路径规划失败
- 检查机器人半径设置是否合理
- 验证地图是否完整
- 调整规划器的 `tolerance` 参数

### 4. 机器人振荡
- 降低控制器频率
- 增大目标容差
- 调整 DWB 评价权重

### 5. 性能问题
- 降低代价地图更新频率
- 减小局部代价地图尺寸
- 减少 DWB 采样数量（`vx_samples`, `vtheta_samples`）
- 禁用组合模式：`use_composition:=false`

## 进一步资源

- [Nav2 官方文档](https://docs.nav2.org/)
- [Nav2 配置指南](https://docs.nav2.org/configuration/index.html)
- [Nav2 调优指南](https://docs.nav2.org/tuning/index.html)
- [DWB 控制器参数](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html)
