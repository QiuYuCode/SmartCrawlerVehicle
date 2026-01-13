# 地图文件目录

## 说明

此目录用于存放通过 SLAM 建图生成的地图文件。

## 地图文件格式

标准的 ROS2 地图由两个文件组成：
- `<map_name>.yaml` - 地图配置文件
- `<map_name>.pgm` - 地图图像文件（灰度占用网格）

## 如何生成地图

1. 首先运行 SLAM 建图模式：
```bash
ros2 launch vehicle_bringup vehicle_slam_launch.py
```

2. 使用键盘或手柄遥控机器人移动，扫描环境

3. 在建图完成后，保存地图到此目录：
```bash
ros2 run nav2_map_server map_saver_cli -f /home/krtrobot/WorkSpace/SmartCrawlerVehicle/src/vehicle_bringup/maps/my_map
```

## 使用地图进行导航

使用保存的地图启动导航：
```bash
ros2 launch vehicle_bringup vehicle_localization_navigation_launch.py map:=/home/krtrobot/WorkSpace/SmartCrawlerVehicle/src/vehicle_bringup/maps/my_map.yaml
```

或者使用默认地图名称 `map.yaml`（如果存在）：
```bash
ros2 launch vehicle_bringup vehicle_localization_navigation_launch.py
```

## 地图 YAML 文件示例

```yaml
image: map.pgm
resolution: 0.050000
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

## 参数说明

- **image**: 地图图像文件名
- **resolution**: 地图分辨率（米/像素）
- **origin**: 地图左下角在世界坐标系中的位置 [x, y, yaw]
- **negate**: 是否反转颜色（0=否，1=是）
- **occupied_thresh**: 占用阈值（大于此值视为障碍物）
- **free_thresh**: 空闲阈值（小于此值视为可通行）
