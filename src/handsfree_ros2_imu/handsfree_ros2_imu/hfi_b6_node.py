#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
HandsFree IMU B6 ROS2 Node (无磁力计版本)
"""
import serial
import struct
import math
import serial.tools.list_ports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from handsfree_ros2_imu.transforms import quaternion_from_euler


def find_ttyUSB():
    """查找 ttyUSB* 设备"""
    print('imu 默认串口为 /dev/ttyUSB0, 若识别多个串口设备, 请在 launch 文件中修改 imu 对应的串口')
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print(f'当前电脑所连接的 USB 串口设备共 {len(posts)} 个: {posts}')


def checkSum(list_data, check_data):
    """校验"""
    return sum(list_data) & 0xff == check_data


def hex_to_short(raw_data):
    """16 进制转短整型"""
    return list(struct.unpack("hhhh", bytearray(raw_data)))


class HFIB6Node(Node):
    """HandsFree IMU B6 ROS2 节点 (无磁力计)"""
    
    def __init__(self):
        super().__init__('hfi_b6_imu')
        
        # 声明参数
        self.declare_parameter('port', '/dev/HFRobotIMU')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('frame_id', 'base_link')
        
        # 获取参数
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # 创建发布者
        self.imu_pub = self.create_publisher(Imu, 'handsfree/imu', 10)
        
        # 初始化数据缓冲
        self.key = 0
        self.buff = {}
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]
        self.pub_flag = [True, True, True]
        
        # 打开串口
        find_ttyUSB()
        try:
            self.hf_imu = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.5)
            if self.hf_imu.isOpen():
                self.get_logger().info('\033[32m串口打开成功...\033[0m')
            else:
                self.hf_imu.open()
                self.get_logger().info('\033[32m打开串口成功...\033[0m')
        except Exception as e:
            self.get_logger().error(f'\033[31m串口打开失败: {e}\033[0m')
            raise SystemExit(1)
        
        # 创建定时器 (200Hz)
        self.timer = self.create_timer(1.0 / 200.0, self.timer_callback)
    
    def timer_callback(self):
        """定时器回调，读取串口数据"""
        try:
            buff_count = self.hf_imu.inWaiting()
        except Exception as e:
            self.get_logger().error(f'exception: {e}')
            self.get_logger().error('imu 失去连接，接触不良，或断线')
            raise SystemExit(1)
        
        if buff_count > 0:
            buff_data = self.hf_imu.read(buff_count)
            for i in range(buff_count):
                self.handle_serial_data(buff_data[i])
    
    def handle_serial_data(self, raw_data):
        """处理串口数据"""
        self.buff[self.key] = raw_data
        self.key += 1
        
        if self.buff[0] != 0x55:
            self.key = 0
            return
        
        if self.key < 11:
            return
        
        data_buff = list(self.buff.values())
        
        if self.buff[1] == 0x51 and self.pub_flag[0]:
            if checkSum(data_buff[0:10], data_buff[10]):
                self.acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 
                                      for i in range(3)]
            else:
                self.get_logger().warn('0x51 校验失败')
            self.pub_flag[0] = False
        
        elif self.buff[1] == 0x52 and self.pub_flag[1]:
            if checkSum(data_buff[0:10], data_buff[10]):
                self.angular_velocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 
                                          for i in range(3)]
            else:
                self.get_logger().warn('0x52 校验失败')
            self.pub_flag[1] = False
        
        elif self.buff[1] == 0x53 and self.pub_flag[2]:
            if checkSum(data_buff[0:10], data_buff[10]):
                self.angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 
                                      for i in range(3)]
            else:
                self.get_logger().warn('0x53 校验失败')
            self.pub_flag[2] = False
        
        else:
            self.get_logger().debug(f'该数据处理类没有提供该 {self.buff[1]} 的解析或数据错误')
            self.buff = {}
            self.key = 0
            return
        
        self.buff = {}
        self.key = 0
        
        if any(self.pub_flag):
            return
        
        self.pub_flag = [True, True, True]
        self.publish_imu_data()
    
    def publish_imu_data(self):
        """发布 IMU 数据"""
        stamp = self.get_clock().now().to_msg()
        
        # IMU 消息
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id
        
        # 欧拉角转四元数
        angle_radian = [self.angle_degree[i] * math.pi / 180.0 for i in range(3)]
        qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])
        
        imu_msg.orientation.x = qua[0]
        imu_msg.orientation.y = qua[1]
        imu_msg.orientation.z = qua[2]
        imu_msg.orientation.w = qua[3]
        
        imu_msg.angular_velocity.x = self.angular_velocity[0]
        imu_msg.angular_velocity.y = self.angular_velocity[1]
        imu_msg.angular_velocity.z = self.angular_velocity[2]
        
        imu_msg.linear_acceleration.x = self.acceleration[0]
        imu_msg.linear_acceleration.y = self.acceleration[1]
        imu_msg.linear_acceleration.z = self.acceleration[2]
        
        self.imu_pub.publish(imu_msg)
    
    def destroy_node(self):
        """销毁节点时关闭串口"""
        if hasattr(self, 'hf_imu') and self.hf_imu.isOpen():
            self.hf_imu.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HFIB6Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

