import serial
import time

# 初始化串口（操作的是工控机的232串口，对应COM口根据实际情况改，比如COM3、/dev/ttyS1）
# 参数必须和步进驱动器的485通信参数一致
ser = serial.Serial(
    port='/dev/ttyS0',          # 工控机的232串口编号（Windows），Linux下是/dev/ttyS*
    baudrate=9600,       # 匹配步进驱动器的波特率
    bytesize=serial.EIGHTBITS,  # 8位数据位
    parity=serial.PARITY_NONE,  # 无校验
    stopbits=serial.STOPBITS_ONE, # 1位停止位
    timeout=0.1,         # 接收超时时间
    rtscts=False         # 手动控制RTS引脚（用于485收发方向）
)

def send_485_command(command_bytes):
    """
    向485总线发送指令（控制步进驱动器）
    :param command_bytes: 步进驱动器的控制指令（字节类型，需按驱动器协议来）
    """
    # 1. 控制RTS引脚为高电平（打开485发送方向，不同转换器可能是低电平，需测试）
    ser.rts = True
    time.sleep(0.001)  # 延时确保方向切换完成
    
    # 2. 发送指令（本质是向232串口发数据，硬件自动转485）
    ser.write(command_bytes)
    ser.flush()  # 确保数据全部发送
    
    # 3. 延时等待发送完成，再切换回接收方向
    time.sleep(0.001)
    ser.rts = False  # RTS低电平，切换为接收方向

def read_485_response():
    """读取步进驱动器返回的响应数据"""

    response = ser.read(ser.in_waiting)
    return response


# 示例：发送步进驱动器的控制指令（指令格式需按你的驱动器手册改）
if __name__ == "__main__":
    try:
        # 假设步进驱动器的“正转100步”指令是 0x01 0x02 0x64 0x00（仅示例，以手册为准）
        step_command = bytes([0x01, 0x03 ,0x00 ,0x03 ,0x00 ,0x10 ,0xb4 ,0x06])
        send_485_command(step_command)
        
        # 读取响应
        response = read_485_response()
        print(f"驱动器响应：{response.hex()}")
        
    finally:
        ser.close()  # 关闭串口