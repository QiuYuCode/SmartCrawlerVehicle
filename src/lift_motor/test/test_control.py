#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import serial
import time
import struct
import fcntl
import array

# 配置部分
PORT = '/dev/ttyS0'
BAUDRATE = 9600
SLAVE_ID = 1

# 构造读取状态指令: [ID, 03, 00, 04, 00, 01, CRC_L, CRC_H]
# 01 03 00 04 00 01 -> CRC=0x0BC9 -> Low=C9, High=0B
CMD_READ_STATUS = bytes([0x01, 0x03, 0x00, 0x04, 0x00, 0x01, 0xC9, 0x0B])

def test_mode_1_direct():
    """模式1: 直接读取 (适用于带自动收发转换器的硬件)"""
    print("-" * 30)
    print("模式 1: 直接读取 (无 RTS 控制)...")
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1.0)
        ser.reset_input_buffer()
        ser.write(CMD_READ_STATUS)
        # 不进行任何RTS操作，直接读
        data = ser.read(7) 
        if data:
            print(f"✅ 成功收到数据: {data.hex()}")
        else:
            print("❌ 超时，无数据")
        ser.close()
    except Exception as e:
        print(f"❌ 错误: {e}")

def test_mode_2_manual_rts():
    """模式2: 手动 RTS 翻转 (适用于无源 232转485)"""
    print("-" * 30)
    print("模式 2: 手动 RTS 控制 (Python层)...")
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1.0, rtscts=False)
        
        # 发送前拉高 RTS (发送模式)
        ser.rts = True
        time.sleep(0.005) 
        ser.write(CMD_READ_STATUS)
        
        # 关键：等待发送完毕。ttyS0 必须要 flush
        ser.flush() 
        # time.sleep(0.001) # 微调：如果flush后还有延迟，这里由于是物理口，可能不需要
        
        # 发送后拉低 RTS (接收模式)
        ser.rts = False
        
        data = ser.read(7)
        if data:
            print(f"✅ 成功收到数据: {data.hex()}")
        else:
            print("❌ 超时，无数据")
        ser.close()
    except Exception as e:
        print(f"❌ 错误: {e}")

def test_mode_3_kernel_rs485():
    """模式3: Linux 内核级 RS485 模式 (推荐用于工控机原生485口)"""
    print("-" * 30)
    print("模式 3: Linux 内核 RS485 模式...")
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1.0)
        
        # 尝试通过 ioctl 开启内核 RS485 支持
        # struct serial_rs485 (flags, delay_rts_before_send, delay_rts_after_send, padding...)
        # SER_RS485_ENABLED = 1, SER_RS485_RTS_ON_SEND = 2, SER_RS485_RTS_AFTER_SEND = 4
        # 许多工控机 ttyS0 支持这个
        try:
            import fcntl
            import struct
            TIOCSRS485 = 0x542F
            SER_RS485_ENABLED = 1
            SER_RS485_RTS_ON_SEND = 2
            SER_RS485_RTS_AFTER_SEND = 4
            
            # 配置：使能485，发送时RTS拉高，发送后拉低
            flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND
            # 有些驱动可能需要反向逻辑，如果不行尝试: flags = SER_RS485_ENABLED | SER_RS485_RTS_AFTER_SEND
            
            serial_rs485 = struct.pack('IIIIIQ', flags, 0, 0, 0, 0, 0)
            fcntl.ioctl(ser.fd, TIOCSRS485, serial_rs485)
            print("   (内核 RS485 模式设置成功)")
        except Exception as e:
            print(f"   (内核 RS485 设置失败或不支持: {e})")
            print("   (跳过此模式测试)")
            ser.close()
            return

        ser.reset_input_buffer()
        ser.write(CMD_READ_STATUS)
        data = ser.read(7)
        if data:
            print(f"✅ 成功收到数据: {data.hex()}")
        else:
            print("❌ 超时，无数据")
        ser.close()
    except Exception as e:
        print(f"❌ 错误: {e}")

if __name__ == "__main__":
    print(f"开始测试串口 {PORT} ...请确保电机已上电")
    test_mode_1_direct()
    time.sleep(0.5)
    test_mode_2_manual_rts()
    time.sleep(0.5)
    test_mode_3_kernel_rs485()