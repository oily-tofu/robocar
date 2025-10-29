# 导入亚博智能串口通信库
# (Import Yahboom UART communication library)
from ybUtils.YbUart import YbUart
import time

# 创建串口实例，设置波特率为115200
# (Create UART instance with baud rate set to 115200)
# 波特率是指每秒传输的比特数，115200是常用的高速通信速率
# (Baud rate refers to bits per second, 115200 is a commonly used high-speed communication rate)
uart = YbUart(baudrate=115200)

# 发送数据到连接的设备
# (Send data to the connected device)
# 发送字符串"Hello Yahboom"并附加换行符
# (Send the string "Hello Yahboom" with a newline character)
uart.send("Hello Yahboom\n")


# 无限循环，持续监听串口数据
# (Infinite loop to continuously monitor serial port data)
while True:
    # 读取串口接收到的数据
    # (Read data received from the serial port)
    # 如果没有数据，将返回None或空字符串
    # (If no data is available, it will return None or an empty string)
    uart.send("#001P2500T1500!")
    print("#001P2500T1500!")
    time.sleep(1)
    uart.send("#001P500T1500!")
    time.sleep(1)
    print("#001P500T1500!")
    data = uart.read()

    # 检查是否接收到数据
    # (Check if data was received)
    if data:
        # 将接收到的数据打印到控制台
        # (Print the received data to the console)
        print(data)

# 注意：由于上面的无限循环，这段代码实际上永远不会执行
# (Note: Due to the infinite loop above, this code will never execute)
# 关闭串口，释放资源
# (Close the UART and release resources)
uart.deinit()
