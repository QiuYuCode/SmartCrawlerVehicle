import serial
import time

# 初始化串口（操作的是工控机的 232/USB 串口，对应设备号根据实际情况改，比如 COM3、/dev/ttyS1）
# 参数必须和步进驱动器的 485 通信参数一致
ser = serial.Serial(
    port='/dev/ttyS0',              # 串口设备
    baudrate=9600,                    # 匹配步进驱动器的波特率
    bytesize=serial.EIGHTBITS,       # 8 位数据位
    parity=serial.PARITY_NONE,       # 无校验
    stopbits=serial.STOPBITS_ONE,    # 1 位停止位
    timeout=1.0                      # 接收超时时间（适当长一点，方便阻塞式读取）
)

def send_485_command(command_bytes):
    """
    向 485 总线发送指令（控制步进驱动器）
    :param command_bytes: 步进驱动器的控制指令（字节类型，需按驱动器协议来）
    """
    # 对于大多数“无源/自动收发”的 RS232/RS485 转换器，不需要也不能手动控制 RTS
    # 直接写串口即可，硬件会自动完成方向切换
    ser.write(command_bytes)
    ser.flush()  # 确保数据全部发送


def read_485_response(wait_time=1.0):
    """
    读取步进驱动器返回的响应数据
    wait_time: 总共等待多长时间收集数据（秒）
    """
    deadline = time.time() + wait_time
    data = b""

    while time.time() < deadline:
        # 如果当前缓冲区有多少就读多少；如果没有就至少读 1 个字节阻塞一会儿
        to_read = ser.in_waiting or 1
        chunk = ser.read(to_read)
        if chunk:
            data += chunk
        else:
            # 没数据就稍微睡一下，避免空转
            time.sleep(0.01)

    return data

# 示例：发送步进驱动器的控制指令（指令格式需按你的驱动器手册改）
if __name__ == "__main__":
    try:
        # 假设步进驱动器的“正转100步”指令是 0x01 0x02 0x64 0x00（仅示例，以手册为准）
        step_command = bytes([0x01, 0x03 ,0x00 ,0x03 ,0x00 ,0x10 ,0xb4 ,0x06])
        send_485_command(step_command)

        # 等待驱动器响应一段时间再读取（这里等待 1 秒，可根据实际情况调整）
        response = read_485_response(wait_time=1.0)
        if response:
            print(f"驱动器响应：{response.hex()}")
        else:
            print("没有数据")
        
    finally:
        ser.close()  # 关闭串口