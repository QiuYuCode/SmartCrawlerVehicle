#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray
import serial
import time
import struct

# --- Modbus RTU 协议常量 (基于 RSA86E 手册) ---
SLAVE_ID = 0x01           # 默认从站地址
FUNC_WRITE_SINGLE = 0x06  # 写单个寄存器功能码

# 寄存器地址
REG_MAX_SPEED = 0x0033    # 最大速度 (单位: rev/min, 正负决定方向)
REG_START_CMD = 0x0037    # 启动命令 (0x01: 速度模式)
REG_STOP_CMD  = 0x0038    # 停止命令 (0: 正常停止, 1: 急停)
REG_ENABLE    = 0x0039    # 使能控制 (1: 使能/锁轴, 0: 释放)

class LiftMotorNode(Node):
    def __init__(self):
        super().__init__('lift_motor_node')

        # 1. 声明参数
        self.declare_parameter('port', '/dev/ttyS0')    # 串口号
        self.declare_parameter('baudrate', 9600)          # 波特率
        self.declare_parameter('speed', 300)              # 默认运动速度 (rpm)
        self.declare_parameter('slave_id', 1)             # 从站 ID

        self.port_name = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.target_speed = self.get_parameter('speed').get_parameter_value().integer_value
        self.slave_id = self.get_parameter('slave_id').get_parameter_value().integer_value

        # 2. 初始化串口
        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.get_logger().info(f'串口已打开: {self.port_name} ({self.baudrate})')
        except Exception as e:
            self.get_logger().error(f'无法打开串口: {e}')
            return

        # 3. 创建订阅者
        self.cmd_sub = self.create_subscription(
            String,
            'lift_motor/cmd',
            self.command_callback,
            10
        )
        self.get_logger().info('升降电机节点已就绪，等待指令 (up, down, stop)...')

    def calculate_crc(self, data):
        """计算 Modbus CRC16 校验码"""
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for i in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

    def send_modbus_write(self, register_addr, value):
        """发送写单个寄存器指令 (Function 06)"""
        if not self.ser or not self.ser.isOpen():
            return

        # 构建帧: [地址, 功能码, 寄存器高, 寄存器低, 值高, 值低]
        # value 需要处理为 16 位有符号整数 (例如负速度)
        val_bytes = struct.pack('>h', value) # Big-endian short
        
        frame = bytearray()
        frame.append(self.slave_id)
        frame.append(FUNC_WRITE_SINGLE)
        frame.append((register_addr >> 8) & 0xFF)
        frame.append(register_addr & 0xFF)
        frame.extend(val_bytes)

        # 计算 CRC
        crc = self.calculate_crc(frame)
        frame.append(crc & 0xFF)        # CRC 低位
        frame.append((crc >> 8) & 0xFF) # CRC 高位

        try:
            self.ser.write(frame)
            self.get_logger().info(f'发送指令: {frame.hex()}')
            # 简单的延时，防止指令发送过快驱动器处理不过来
            time.sleep(0.02) 
        except Exception as e:
            self.get_logger().error(f'串口发送错误: {e}')

    def set_speed_mode(self, speed_rpm):
        """执行速度模式控制流程: 设置速度 -> 使能 -> 启动"""
        # 1. 设置最大速度 (寄存器 0x0033)
        # 正数正转，负数反转
        self.send_modbus_write(REG_MAX_SPEED, speed_rpm)
        
        # 2. 使能电机 (寄存器 0x0039 = 1)
        self.send_modbus_write(REG_ENABLE, 1)

        # 3. 触发速度模式启动 (寄存器 0x0037 = 1)
        self.send_modbus_write(REG_START_CMD, 1)

    def stop_motor(self):
        """停止电机"""
        # 发送正常停止命令 (寄存器 0x0038 = 0)
        # 也可以发送急停 (寄存器 0x0038 = 1)
        self.send_modbus_write(REG_STOP_CMD, 0)

    def command_callback(self, msg):
        cmd = msg.data.lower()
        self.get_logger().info(f'收到指令: {cmd}')

        if cmd == 'up':
            # 上升：正向速度
            self.set_speed_mode(self.target_speed)
        elif cmd == 'down':
            # 下降：负向速度
            self.set_speed_mode(-self.target_speed)
        elif cmd == 'stop':
            self.stop_motor()
        else:
            self.get_logger().warn(f'未知指令: {cmd}')

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.isOpen():
            self.stop_motor() # 尝试在退出前停止
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LiftMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()