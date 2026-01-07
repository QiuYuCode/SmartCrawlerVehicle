#!/bin/bash
# 安装 HandsFree IMU udev 规则脚本

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_FILE="${SCRIPT_DIR}/../rules/99-handsfree-imu.rules"

echo "正在安装 HandsFree IMU udev 规则..."

if [ ! -f "$RULES_FILE" ]; then
    echo "错误: 找不到规则文件 $RULES_FILE"
    exit 1
fi

sudo cp "$RULES_FILE" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "udev 规则安装完成!"
echo "请重新插拔 IMU 设备，或重启系统"
echo "设备将自动映射为 /dev/HFRobotIMU"

