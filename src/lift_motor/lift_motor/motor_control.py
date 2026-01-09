#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import struct

# --- Modbus RTU 协议常量 ---
SLAVE_ID = 0x01           # 默认从站地址
FUNC_READ_HOLDING = 0x03  # 读保持寄存器
FUNC_WRITE_SINGLE = 0x06  # 写单个寄存器

# 寄存器地址定义 (基于 RSA86E 手册)
REG_STATUS       = 0x0004    # 驱动器状态
REG_CUR_POS_L    = 0x0007    # 当前位置低16位
REG_CUR_POS_H    = 0x0008    # 当前位置高16位
REG_INPUT_STATE  = 0x0009    # 输入端口状态
REG_LIMIT_CFG    = 0x0018    # 超程停车功能
REG_MAX_SPEED    = 0x0033    # 最大速度
REG_PULSE_LOW    = 0x0034    # 目标位置/脉冲数 低16位
REG_PULSE_HIGH   = 0x0035    # 目标位置/脉冲数 高16位
REG_START_CMD    = 0x0037    # 启动命令 (1:速度, 2:相对, 4:绝对)
REG_STOP_CMD     = 0x0038    # 停止命令
REG_ENABLE       = 0x0039    # 使能控制
REG_CLR_ALARM    = 0x0019    # 报警清除/位置清零(需确认手册)

class LiftMotorNode(Node):
    def __init__(self):
        super().__init__('lift_motor_node')

        # --- 参数配置 ---
        self.declare_parameter('port', '/dev/ttyS0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('default_speed', 60)
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('disable_limits', True)
        
        # 关键参数：脉冲当量 (1mm 对应多少脉冲)
        # 计算公式：pulses_per_mm = (电机细分 / 丝杆导程)
        # 例如：细分1600(每转脉冲数) / 导程5mm = 320 脉冲/mm
        # 请根据您的硬件实际情况修改此默认值！
        self.declare_parameter('pulses_per_mm', 320.0) 
        
        # 关键参数：最大行程 (单位: mm)
        self.declare_parameter('max_travel_mm', 980.0)

        # 获取参数
        self.port_name = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.default_speed = self.get_parameter('default_speed').get_parameter_value().integer_value
        self.slave_id = self.get_parameter('slave_id').get_parameter_value().integer_value
        disable_limits = self.get_parameter('disable_limits').get_parameter_value().bool_value
        
        self.pulses_per_mm = self.get_parameter('pulses_per_mm').get_parameter_value().double_value
        self.max_travel_mm = self.get_parameter('max_travel_mm').get_parameter_value().double_value

        # 初始化串口
        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.5,
                write_timeout=0.5
            )
            self.get_logger().info(f'串口已打开: {self.port_name}')
            self.get_logger().info(f'参数设置: 1mm={self.pulses_per_mm}脉冲, 最大行程={self.max_travel_mm}mm')
        except Exception as e:
            self.get_logger().error(f'无法打开串口: {e}')
            return

        # 启动时关闭限位保护(如需)
        if disable_limits:
            time.sleep(0.1)
            self.set_limit_protection(False)

        # 订阅指令
        self.cmd_sub = self.create_subscription(
            String,
            'lift_motor/cmd',
            self.command_callback,
            10
        )

    # ... (CRC计算和Modbus发送函数保持不变) ...
    def calculate_crc(self, data):
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
        if not self.ser or not self.ser.isOpen(): return
        val_bytes = struct.pack('>H', value & 0xFFFF)
        frame = bytearray([self.slave_id, FUNC_WRITE_SINGLE, (register_addr >> 8) & 0xFF, register_addr & 0xFF])
        frame.extend(val_bytes)
        crc = self.calculate_crc(frame)
        frame.extend([crc & 0xFF, (crc >> 8) & 0xFF])
        try:
            self.ser.write(frame)
            time.sleep(0.02)
        except Exception as e:
            self.get_logger().error(f'写入失败: {e}')

    def read_modbus_register(self, register_addr):
        if not self.ser or not self.ser.isOpen(): return None
        frame = bytearray([self.slave_id, FUNC_READ_HOLDING, (register_addr >> 8) & 0xFF, register_addr & 0xFF, 0x00, 0x01])
        crc = self.calculate_crc(frame)
        frame.extend([crc & 0xFF, (crc >> 8) & 0xFF])
        try:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            time.sleep(0.05)
            response = self.ser.read(7)
            if len(response) == 7 and response[0] == self.slave_id:
                return (response[3] << 8) | response[4]
            return None
        except Exception as e:
            self.get_logger().error(f'读取异常: {e}')
            return None

    # ... (基础功能函数) ...
    def set_limit_protection(self, enable: bool):
        val = 6 if enable else 0
        self.send_modbus_write(REG_LIMIT_CFG, val)

    def set_enable(self, enable: bool):
        self.send_modbus_write(REG_ENABLE, 1 if enable else 0)
        self.get_logger().info(f'电机状态: {"使能" if enable else "释放"}')

    def stop_motor(self):
        self.send_modbus_write(REG_STOP_CMD, 0)
        self.get_logger().info('执行停止')

    # --- 核心：MM 控制逻辑 ---

    def mm_to_pulses(self, mm):
        """将毫米转换为脉冲数"""
        return int(mm * self.pulses_per_mm)

    def move_absolute_mm(self, target_mm, speed_rpm):
        """绝对位置控制 (mm)"""
        # 1. 软件限位检查
        if not (0 <= target_mm <= self.max_travel_mm):
            self.get_logger().error(f'目标位置 {target_mm}mm 超出行程范围 (0-{self.max_travel_mm}mm)!')
            return

        self.get_logger().info(f'>>> 准备移动到绝对位置: {target_mm} mm')

        # 2. 转换为脉冲
        target_pulses = self.mm_to_pulses(target_mm)
        
        # 3. 设置速度 (绝对值)
        self.send_modbus_write(REG_MAX_SPEED, abs(int(speed_rpm)))

        # 4. 写入目标脉冲 (32位拆分)
        val = target_pulses & 0xFFFFFFFF
        self.send_modbus_write(REG_PULSE_LOW, val & 0xFFFF)
        self.send_modbus_write(REG_PULSE_HIGH, (val >> 16) & 0xFFFF)

        # 5. 使能并触发绝对模式 (0x04)
        self.send_modbus_write(REG_ENABLE, 1)
        self.send_modbus_write(REG_START_CMD, 4) # 4 = 绝对位置模式

    def move_relative_mm(self, dist_mm, speed_rpm):
        """相对位置控制 (mm)"""
        self.get_logger().info(f'>>> 准备相对移动: {dist_mm} mm')

        # 转换为脉冲 (保留正负号)
        pulse_delta = self.mm_to_pulses(dist_mm)

        # 设置速度 (正值)
        self.send_modbus_write(REG_MAX_SPEED, abs(int(speed_rpm)))

        # 写入脉冲数
        val = pulse_delta & 0xFFFFFFFF
        self.send_modbus_write(REG_PULSE_LOW, val & 0xFFFF)
        self.send_modbus_write(REG_PULSE_HIGH, (val >> 16) & 0xFFFF)

        # 使能并触发相对模式 (0x02)
        self.send_modbus_write(REG_ENABLE, 1)
        self.send_modbus_write(REG_START_CMD, 2) # 2 = 相对位置模式

    def check_status_mm(self):
        """查询状态并显示当前毫米位置"""
        status = self.read_modbus_register(REG_STATUS)
        time.sleep(0.02)
        # 读取当前位置 (32位)
        pos_l = self.read_modbus_register(REG_CUR_POS_L) # 0x0007
        time.sleep(0.02)
        pos_h = self.read_modbus_register(REG_CUR_POS_H) # 0x0008
        
        current_mm = 0.0
        if pos_l is not None and pos_h is not None:
            # 拼接 32 位有符号整数
            current_pulses = (pos_h << 16) | pos_l
            if current_pulses & 0x80000000: # 处理负数
                current_pulses -= 0x100000000
            
            # 换算回 mm
            if self.pulses_per_mm > 0:
                current_mm = current_pulses / self.pulses_per_mm
            
            self.get_logger().info(f'当前位置: {current_mm:.2f} mm ({current_pulses} 脉冲)')
        
        if status is not None:
            is_moving = (status >> 1) & 0x01
            self.get_logger().info(f'运动状态: {"运行中" if is_moving else "静止"}')

    def command_callback(self, msg):
        """
        指令格式:
        move abs 500   -> 移动到 500mm
        move rel 10    -> 向上 10mm
        move rel -10   -> 向下 10mm
        status         -> 查看当前 mm 位置
        """
        cmd_str = msg.data.lower().strip()
        parts = cmd_str.split()
        
        if not parts: return
        action = parts[0]

        try:
            if action == 'move' and len(parts) >= 3:
                mode = parts[1] # abs 或 rel
                val = float(parts[2]) # 毫米数值

                if mode == 'abs':
                    self.move_absolute_mm(val, self.default_speed)
                elif mode == 'rel':
                    self.move_relative_mm(val, self.default_speed)
                else:
                    self.get_logger().warn(f'未知模式: {mode} (请用 abs 或 rel)')

            elif action == 'status':
                self.check_status_mm()
            elif action == 'stop':
                self.stop_motor()
            elif action == 'enable':
                self.set_enable(True)
            elif action == 'disable':
                self.set_enable(False)
            else:
                self.get_logger().warn(f'未知指令: {cmd_str}')
        except ValueError:
            self.get_logger().error('数值格式错误')

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.isOpen():
            self.stop_motor()
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