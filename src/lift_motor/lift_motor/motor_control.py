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
REG_WORK_MODE          = 0x0003    # 驱动器工作模式
REG_DRIVE_STATUS       = 0x0004    # 驱动器状态 (Bit4-5:0-无)
REG_INPUT_STATE        = 0x0009    # 输入端口状态 (查看限位开关)
REG_LIMIT_CFG          = 0x0018    # 超程停车功能 (Bit1:硬限位, Bit2:软限位)
REG_MAX_SPEED          = 0x0033    # 最大速度
REG_PULSE_LOW          = 0x0034    # 总脉冲数低16位
REG_PULSE_HIGH         = 0x0035    # 总脉冲数高16位
REG_START_CMD          = 0x0037    # 启动命令
REG_STOP_CMD           = 0x0038    # 停止命令
REG_ENABLE             = 0x0039    # 使能控制
REG_CLR_ALARM          = 0x0019    # 报警清除/位置清零

# 错误代码定义
REG_CUR_POS_L = 0x0007    # 当前绝对位置低16位
REG_CUR_POS_H = 0x0008    # 当前绝对位置高16位

class LiftMotorNode(Node):
    def __init__(self):
        super().__init__('lift_motor_node')

        # 参数配置
        self.declare_parameter('port', '/dev/ttyS0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('default_speed', 60)
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('disable_limits', False)

        self.port_name = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.default_speed = self.get_parameter('default_speed').get_parameter_value().integer_value
        self.slave_id = self.get_parameter('slave_id').get_parameter_value().integer_value
        disable_limits = self.get_parameter('disable_limits').get_parameter_value().bool_value

        # 初始化串口
        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.03
            )
            self.get_logger().info(f'串口已打开: {self.port_name}')
        except Exception as e:
            self.get_logger().error(f'无法打开串口: {e}')
            return

        # 启动时处理限位设置
        if disable_limits:
            time.sleep(0.03) # 等待串口稳定
            self.set_limit_protection(False)

        # 订阅指令
        self.cmd_sub = self.create_subscription(
            String,
            'lift_motor/cmd',
            self.command_callback,
            10
        )
        self.get_logger().info('指令列表: up, down, stop, origin, enable, disable')
        self.get_logger().info('移动指令: move 10000 (正数向上, 负数向下)')

    def calculate_crc(self, data):
        """计算 CRC16"""
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
        """发送写寄存器指令 (06)"""
        if not self.ser or not self.ser.isOpen(): return
        
        val_bytes = struct.pack('>H', value & 0xFFFF)
        frame = bytearray([self.slave_id, FUNC_WRITE_SINGLE, (register_addr >> 8) & 0xFF, register_addr & 0xFF])
        frame.extend(val_bytes)
        
        crc = self.calculate_crc(frame)
        frame.extend([crc & 0xFF, (crc >> 8) & 0xFF])

        try:
            self.ser.write(frame)
            time.sleep(0.03)
        except Exception as e:
            self.get_logger().error(f'发送失败: {e}')

    def read_modbus_register(self, register_addr):
        """读取寄存器 (03)，用于调试"""
        if not self.ser or not self.ser.isOpen(): return None

        frame = bytearray([self.slave_id, FUNC_READ_HOLDING, (register_addr >> 8) & 0xFF, register_addr & 0xFF, 0x00, 0x01])
        crc = self.calculate_crc(frame)
        frame.extend([crc & 0xFF, (crc >> 8) & 0xFF])

        try:
            self.ser.flushInput()
            self.ser.write(frame)
            time.sleep(0.03)
            response = self.ser.read(7) # 地址(1)+功能(1)+字节数(1)+数据(2)+CRC(2)
            if len(response) == 7:
                # 简单解析
                val = (response[3] << 8) | response[4]
                return val
            else:
                self.get_logger().warn('读取超时或响应长度错误')
                return None
        except Exception as e:
            self.get_logger().error(f'读取失败: {e}')
            return None

    def set_limit_protection(self, enable: bool):
        """开启或关闭限位保护 (寄存器 0x0018)"""
        # 默认值 6 (二进制110) 表示开启硬限位(Bit1)和软限位(Bit2)
        # 设为 0 表示全部关闭
        val = 6 if enable else 0
        self.send_modbus_write(REG_LIMIT_CFG, val)
        state = "开启" if enable else "关闭"
        self.get_logger().info(f'>>> 限位保护已{state} (Reg 0x0018={val})')

    def set_enable(self, enable: bool):
        """使能或释放电机 (寄存器 0x0039)"""
        val = 1 if enable else 0
        self.send_modbus_write(REG_ENABLE, val)
        self.get_logger().info(f'电机状态: {"使能(锁轴)" if enable else "释放(脱机)"}')

    def set_speed_mode(self, speed_rpm):
        """最大速度设置，默认最大值为 60 RPM r/min
        :param speed_rpm: 速度 (单位: RPM)
        """
        self.send_modbus_write(REG_MAX_SPEED, int(speed_rpm))
        self.send_modbus_write(REG_ENABLE, 1)
        self.send_modbus_write(REG_START_CMD, 1)

    def move_relative(self, pulses, speed_rpm):
        """相对运动模式

        :param pulses: 脉冲数
        :param speed_rpm: 速度 (单位: RPM)
        """
        self.send_modbus_write(REG_MAX_SPEED, abs(int(speed_rpm)))
        val = int(pulses) & 0xFFFFFFFF
        self.send_modbus_write(REG_PULSE_LOW, val & 0xFFFF)
        self.send_modbus_write(REG_PULSE_HIGH, (val >> 16) & 0xFFFF)
        self.send_modbus_write(REG_ENABLE, 1)
        self.send_modbus_write(REG_START_CMD, 2)
        self.get_logger().info(f'执行相对运动: {pulses} 脉冲')

    def move_absolute(self, position_pulses, speed_rpm):
        """绝对运动模式 (移动到指定坐标)

        :param position_pulses: 目标绝对位置 (脉冲数)
        :param speed_rpm: 速度 (单位: RPM)
        """
        # 1. 设置速度 (0x0033)
        self.send_modbus_write(REG_MAX_SPEED, abs(int(speed_rpm)))
        
        # 2. 设置目标绝对位置 (0x0034, 0x0035)
        # Python 的 & 0xFFFFFFFF 会自动处理负数补码，无需手动计算
        val = int(position_pulses) & 0xFFFFFFFF
        self.send_modbus_write(REG_PULSE_LOW, val & 0xFFFF)
        self.send_modbus_write(REG_PULSE_HIGH, (val >> 16) & 0xFFFF)
        
        # 3. 使能电机 (0x0039)
        self.send_modbus_write(REG_ENABLE, 1)
        
        # 4. 发送启动命令 (0x0037)
        # 值为 4 (二进制 100) 代表绝对位置模式触发
        self.send_modbus_write(REG_START_CMD, 4)
        
        self.get_logger().info(f'执行绝对运动 -> 目标位置: {position_pulses}')
    
    def stop_motor(self):
        """停止电机动作"""
        self.send_modbus_write(REG_STOP_CMD, 0)
        self.get_logger().info('电机停止')

    def back_to_origin(self):
        """回原点模式触发"""
        self.send_modbus_write(REG_START_CMD, 8)
        self.get_logger().info(f"回原点模式启动")
    
    def check_status(self):
        """读取状态和当前位置"""
        status = self.read_modbus_register(REG_DRIVE_STATUS)
        pos_l = self.read_modbus_register(REG_CUR_POS_L) # 0x0007
        pos_h = self.read_modbus_register(REG_CUR_POS_H) # 0x0008
        
        current_pos = 0
        if pos_l is not None and pos_h is not None:
            # 拼接32位有符号整数
            current_pos = (pos_h << 16) | pos_l
            if current_pos & 0x80000000:
                current_pos -= 0x100000000
                
        self.get_logger().info(f'-----------------------')
        self.get_logger().info(f'驱动器状态寄存器: {status if status else "None"}')
        self.get_logger().info(f'当前状态: {current_pos}')
        self.get_logger().info(f'-----------------------')
    
    def clear_position(self):
        """尝试强制清零当前位置"""
        # 写入 0x0019 (报警清除/位置清零)
        self.send_modbus_write(REG_CLR_ALARM, 1) 
        self.get_logger().info('已发送位置清零/报警清除指令 (Reg 0x0019 = 1)')
    
    def command_callback(self, msg):
        """ros 处理指令回调"""
        cmd_str = msg.data.lower().strip()
        parts = cmd_str.split()
        cmd = parts[0]

        if cmd == 'enable':
            self.set_enable(True)
        elif cmd == 'disable':
            self.set_enable(False)
        elif cmd == 'up':
            self.set_speed_mode(self.default_speed)
        elif cmd == 'down':
            self.set_speed_mode(-self.default_speed)
        elif cmd == 'stop':
            self.stop_motor()
        elif cmd == 'origin':
            self.back_to_origin()
        elif cmd == 'status':
            self.check_status()
        elif cmd == 'clear':
            self.clear_position()
        elif cmd == 'move' and len(parts) > 1:
            # 相对运动: move 1000 (当前位置 + 1000)
            try:
                self.move_relative(int(parts[1]), self.default_speed)
            except ValueError:
                self.get_logger().error('相对运动参数错误')
        elif cmd == 'abs' and len(parts) > 1:
            # [新增] 绝对运动: abs 5000 (移动到坐标 5000)
            try:
                self.move_absolute(int(parts[1]), self.default_speed)
            except ValueError:
                self.get_logger().error('绝对运动参数错误')
        else:
            self.get_logger().warn(f'未知指令: {cmd_str}')

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