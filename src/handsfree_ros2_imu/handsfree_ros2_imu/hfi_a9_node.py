# -*- coding:utf-8 -*-
"""
HandsFree IMU A9 ROS2 Node
"""
import serial
import struct
import math
import serial.tools.list_ports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from handsfree_ros2_imu.transforms import quaternion_from_euler


def find_ttyUSB():
    """查找 ttyUSB* 设备"""
    print('imu 默认串口为 /dev/ttyUSB0, 若识别多个串口设备, 请在 launch 文件中修改 imu 对应的串口')
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print(f'当前电脑所连接的 USB 串口设备共 {len(posts)} 个: {posts}')


def checkSum(list_data, check_data):
    """CRC 校验"""
    data = bytearray(list_data)
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return hex(((crc & 0xff) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])


def hex_to_ieee(raw_data):
    """16 进制转 ieee 浮点数"""
    ieee_data = []
    raw_data.reverse()
    for i in range(0, len(raw_data), 4):
        data2str = (hex(raw_data[i] | 0xff00)[4:6] + 
                    hex(raw_data[i + 1] | 0xff00)[4:6] + 
                    hex(raw_data[i + 2] | 0xff00)[4:6] + 
                    hex(raw_data[i + 3] | 0xff00)[4:6])
        ieee_data.append(struct.unpack('>f', bytes.fromhex(data2str))[0])
    ieee_data.reverse()
    return ieee_data


class HFIA9Node(Node):
    """HandsFree IMU A9 ROS2 节点"""
    
    def __init__(self):
        super().__init__('hfi_a9_imu')
        
        # 声明参数
        self.declare_parameter('port', '/dev/HFRobotIMU')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('gra_normalization', True)
        self.declare_parameter('frame_id', 'base_link')
        
        # 获取参数
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.gra_normalization = self.get_parameter('gra_normalization').get_parameter_value().bool_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # 创建发布者
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/mag', 10)
        
        # 初始化数据缓冲
        self.key = 0
        self.buff = {}
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]
        self.pub_flag = [True, True]
        self.data_right_count = 0
        
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
        
        # 创建定时器 (300Hz)
        self.timer = self.create_timer(1.0 / 300.0, self.timer_callback)
    
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
        if self.data_right_count > 200000:
            self.get_logger().error('该设备传输数据错误，退出')
            raise SystemExit(1)
        
        self.buff[self.key] = raw_data
        self.key += 1
        
        if self.buff[0] != 0xaa:
            self.data_right_count += 1
            self.key = 0
            return
        
        if self.key < 3:
            return
        
        if self.buff[1] != 0x55:
            self.key = 0
            return
        
        if self.key < self.buff[2] + 5:
            return
        
        self.data_right_count = 0
        data_buff = list(self.buff.values())
        
        if self.buff[2] == 0x2c and self.pub_flag[0]:
            if checkSum(data_buff[2:47], data_buff[47:49]):
                data = hex_to_ieee(data_buff[7:47])
                self.angular_velocity = data[1:4]
                self.acceleration = data[4:7]
                self.magnetometer = data[7:10]
            else:
                self.get_logger().warn('校验失败')
            self.pub_flag[0] = False
        elif self.buff[2] == 0x14 and self.pub_flag[1]:
            if checkSum(data_buff[2:23], data_buff[23:25]):
                data = hex_to_ieee(data_buff[7:23])
                self.angle_degree = data[1:4]
            else:
                self.get_logger().warn('校验失败')
            self.pub_flag[1] = False
        else:
            self.get_logger().debug(f'该数据处理类没有提供该 {self.buff[2]} 的解析或数据错误')
            self.buff = {}
            self.key = 0
            return
        
        self.buff = {}
        self.key = 0
        self.pub_flag[0] = self.pub_flag[1] = True
        
        # 发布 IMU 消息
        self.publish_imu_data()
    
    def publish_imu_data(self):
        """发布 IMU 和磁力计数据"""
        stamp = self.get_clock().now().to_msg()
        
        # IMU 消息
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id
        
        # 磁力计消息
        mag_msg = MagneticField()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = self.frame_id
        
        # 欧拉角转四元数
        angle_radian = [self.angle_degree[i] * math.pi / 180.0 for i in range(3)]
        qua = quaternion_from_euler(angle_radian[0], -angle_radian[1], -angle_radian[2])
        
        imu_msg.orientation.x = qua[0]
        imu_msg.orientation.y = qua[1]
        imu_msg.orientation.z = qua[2]
        imu_msg.orientation.w = qua[3]
        
        imu_msg.angular_velocity.x = self.angular_velocity[0]
        imu_msg.angular_velocity.y = self.angular_velocity[1]
        imu_msg.angular_velocity.z = self.angular_velocity[2]
        
        acc_k = math.sqrt(sum(a**2 for a in self.acceleration))
        if acc_k == 0:
            acc_k = 1
        
        if self.gra_normalization:
            imu_msg.linear_acceleration.x = self.acceleration[0] * -9.8 / acc_k
            imu_msg.linear_acceleration.y = self.acceleration[1] * -9.8 / acc_k
            imu_msg.linear_acceleration.z = self.acceleration[2] * -9.8 / acc_k
        else:
            imu_msg.linear_acceleration.x = self.acceleration[0] * -9.8
            imu_msg.linear_acceleration.y = self.acceleration[1] * -9.8
            imu_msg.linear_acceleration.z = self.acceleration[2] * -9.8
        
        mag_msg.magnetic_field.x = self.magnetometer[0]
        mag_msg.magnetic_field.y = self.magnetometer[1]
        mag_msg.magnetic_field.z = self.magnetometer[2]
        
        self.imu_pub.publish(imu_msg)
        self.mag_pub.publish(mag_msg)
    
    def destroy_node(self):
        """销毁节点时关闭串口"""
        if hasattr(self, 'hf_imu') and self.hf_imu.isOpen():
            self.hf_imu.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HFIA9Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

