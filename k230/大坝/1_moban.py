import time, math, os, gc, sys
from media.sensor import *      #sensor 负责摄像头操作
from media.display import *     #display 负责显示屏操作
from media.media import *       #media 管理媒体资源
from ybUtils.YbUart import YbUart
from ybUtils.YbKey import YbKey
from machine import Pin
from collections import deque
import cv_lite


#前置变量声明
WIDTH = 640
HEIGHT = 480
sensor = None
uart = YbUart(baudrate=115200)
key = YbKey()
led = Pin(42, Pin.OUT)


THRESHOLDS = [
    (0, 66, 7, 127, 3, 127),    # 红色阈值
    (71, 81, -8, 7, 3, 30),     # 橙
    (18, 47, -6, 3, -8, 6),       # 黑
    (46, 100, -8, 127, -13, 14)   # 白
]

def init_sensor():
    """初始化摄像头 / Initialize camera sensor"""
    sensor = Sensor(width=WIDTH, height=HEIGHT, fps=30)
    sensor.reset()
    sensor.set_framesize(width=WIDTH, height=HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    return sensor

def init_display():
    """初始化显示 / Initialize display"""
    Display.init(Display.ST7701, to_ide=True)
    MediaManager.init()



def key_1():
    global key_decide
    if key.is_pressed():
        print("检测到按键按下", "pressed")
        key_decide += 1
        if key_decide > 4:
            key_decide = 1
        time.sleep_ms(200)

def len_l(p1,p2):
    l1 = abs(p1[0] - p2[0])
    l2 = abs(p1[1] - p2[1])
    if l1 < 2:
        l = l2
    elif l2 < 2:
        l = l1
    else:
        l = math.sqrt(l1*l1 + l2*l2)
    return l


def find_r_r():
    rects = img_bin.find_rects(threshold=6000)
    for r in rects:
        x, y, w, h = r.rect()
        cx = x + w // 2
        cy = y + h // 2
        corners = r.corners()
        s = w * h
        c = w / h
        left =len_l(corners[2],corners[3])
        right =len_l(corners[0],corners[1])
        top =len_l(corners[1],corners[2])
        mod =len_l(corners[3],corners[0])
        if 6000 < s < 120000 and 0.75 < c < 1.55:
            if s > 25000:
            #img.draw_rectangle(r.rect(), color=(0,250,255), thickness=6)
                if left >= right:
                    print("左下")
                    cx = int(cx + (263 - mod) * 0.105)
                else:
                    print("右下",top, mod)
                    cx = int(cx - (263 - mod) * 0.105)
            x = int(cx)
            b = 240 + (w - 100) * 0.25


            img.draw_cross(320, int(b), color=(0,0,0), thickness=2)

            img.draw_cross(x, cy, color=(0, 255, 0), thickness=2)
            img.draw_cross(corners[0][0], corners[0][1], color=(0, 255, 0))
            img.draw_cross(corners[1][0], corners[1][1], color=(0, 255, 0))
            img.draw_cross(corners[2][0], corners[2][1], color=(0, 255, 0))
            img.draw_cross(corners[3][0], corners[3][1], color=(0, 255, 0))
            img.draw_line(corners[0][0], corners[0][1],corners[1][0], corners[1][1], color=(0, 255, 0), thickness=2)
            img.draw_line(corners[1][0], corners[1][1],corners[2][0], corners[2][1], color=(0, 255, 0), thickness=2)
            img.draw_line(corners[2][0], corners[2][1],corners[3][0], corners[3][1], color=(0, 255, 0), thickness=2)
            img.draw_line(corners[3][0], corners[3][1],corners[0][0], corners[0][1], color=(0, 255, 0), thickness=2)

            dx = cx - 320
            dy = int(b) - cy

            return (dx, dy, w, x, cy, h, left, right, top, mod)



def find_b_r():
    global key_decide
    blobs = img_bin.find_blobs([(100, 255)], area_threshold=50, x_stride=10, y_stride=10)
    left =0
    right =0
    top =0
    mod =0
    if blobs:
        for blob in blobs:
            w = blob[2]
            h = blob[3]
            s = w * h
            c = w / h
            a = 0
            d = 0
            if 6000 < s < 120000 and 0.75 < c < 1.55:
                b = 240 + (w - 100) * 0.25
#                print(w)
#                print(int(b))
                x = blob[5]
                y = blob[6]
                if key_decide == 1:
                    d = x - (263 - w) * 0.075
                elif key_decide == 2:
                    d = x + (263 - w) * 0.075
                else:
                    d = x
                x = int(d)
                y = blob[6]
                img.draw_cross(320, int(b), color=(0,0,0), thickness=2)

                img.draw_rectangle(blob[0:4], color=(0,250,255), thickness=6)
                img.draw_cross(x, y, color=(0,250,255), thickness=2)

                dx = x - 320
                dy = int(b) - y
                #img.draw_arrow(blob[5], blob[6], 320, int(b), color = (0,0,0), thickness=2)
                return (dx, dy, w, x, y, h, left, right, top, mod)


def uart_get():
    data = uart.read()
    buffer = ""
    in_message = False
    if data:
        data_1 = data.decode()
        print("uart: ",data_1)
        for ch in data_1:
            if ch == "@":
                in_message = True
                buffer = ""
            elif ch == "#" and in_message:
                in_message = False
                # 去掉两端空白并按空格分割
                parts = buffer.strip().split(" ")
                if len(parts) == 2:
                    try:
                        dx = int(parts[0])
                        dy = int(parts[1])
                        print("接收到 dx:", dx, "dy:", dy)
                    except:
                        print("解析失败：", buffer)
                        return (dx, dy)
                buffer = ""
            elif in_message:
                buffer += ch

#位置模式控制 \地址\模式\方向\速度  \加速度\     位置     \
#data =    b'\x00\xfd\x00\x00\x0a\x00\x00\x00\x03\xe6\x00\x00\x6b'
#立即停止  地址 + 0xFE + 0x98 + 多机同步标志 + 校验字节
#01 FE 98 00 6B


def uart_send(dx, dy):
#    data0 = b'\x00\xfd\x00\x00\x0a\x32\x00\x00\x00\x15\x00\x00\x6b'  #逆35
#    data1 = b'\x00\xfd\x01\x00\x0a\x32\x00\x00\x00\x15\x00\x00\x6b'  #顺
#    data2 = b'\01\FE\98\00\6B'  #停
#    if dx >= 5:
#        uart.send(data1)
#    elif -5 < dx < 5:
#        uart.send(data2)
#    else:
#        uart.send(data0)
#    time.sleep_ms(110)
    print("find jx", dx)
    print("find jy", dy)
    uart.send("@")
    uart.send(str(dx))
    uart.send(" ")
    uart.send(str(dy))
    uart.send("#")
    uart.send("%")

def find_c(x,y,w):
    r = w * 7 / 29
    #print("draw r")
    #img.draw_circle(dx, dy, int(r), color = (0,250,255), thickness=2)

def find_e(x,y,w,h,left, right, top, mod):
    rx = int(w * 7 / 29)
    ry = int(h * 7 / 21)
    s = w * h
    if s > 25000:
    #img.draw_rectangle(r.rect(), color=(0,250,255), thickness=6)
        if left > right:
            print("左下")
            jd = int((263 - w) * 30 / 102)
        else:
            print("右下")
            jd = int((w - 263) * 30 / 102)
    else:
        jd = 0

    img.draw_ellipse(x, y, rx, ry, jd, color = (0,250,255), thickness=2)


# PID参数
Kp = 2.5
Ki = 0.01
Kd = 5

# PID变量
error_last_x = 0
integral_x = 0
error_last_y = 0
integral_y = 0
# 电机配置
motor_addr_1 = 0x01
motor_addr_2 = 0x02  # 电机地址
motor_dir_x = 0      # 方向默认 CW
motor_dir_y = 0
motor_speed = 1000  # 速度 RPM
motor_acc = 50     # 加速度
step_factor = 1   # 将dy转换为脉冲的因子，视结构设置


# 主循环中的 PID 控制逻辑
def position_pid_x(dx, dy):
    global error_last_x, integral_x, error_last_y, integral_y

    target = 0                    #设置 目标值
    error_x = target - dx           #计算当前的 误差
    integral_x += error_x             #累加历史误差
    derivative_x = error_x - error_last_x  #计算误差变化率 = 当前误差 - 上一次误差
    error_last_x = error_x

    error_y = target - dy           #计算当前的 误差
    integral_y += error_y             #累加历史误差
    derivative_y = error_y - error_last_y  #计算误差变化率 = 当前误差 - 上一次误差
    error_last_y = error_y

    output_x = Kp * error_x + Ki * integral_x + Kd * derivative_x
    output_y = Kp * error_y + Ki * integral_y + Kd * derivative_y

    # 限制输出
    output_x = max(min(output_x, 10), -10)
    output_y = max(min(output_y, 10), -10)
    # 计算脉冲数（可以乘个因子放大）
    pulse_x = int(abs(output_x) * step_factor)
    pulse_y = int(abs(output_y) * step_factor)
#    pulse_x = int(abs(output_x))
#    pulse_y = int(abs(output_y))
    if abs(error_x) < 5:
        pulse_x =0
    if abs(error_y) < 5:
        pulse_y =0

    # 设置方向
    if output_x >= 0:
        motor_dir_x = 0  # 顺时针 CW
    else:
        motor_dir_x = 1

    if output_y >= 0:
        motor_dir_y = 1  # 顺时针 CW
    else:
        motor_dir_y = 0
    # 构造位置控制数据帧（相对位移）
    cmd_x = bytearray(13)
    cmd_x[0]  = motor_addr_2
    cmd_x[1]  = 0xFD  # 功能码：位置模式
    cmd_x[2]  = motor_dir_x
    cmd_x[3]  = (motor_speed >> 8) & 0xFF
    cmd_x[4]  = motor_speed & 0xFF
    cmd_x[5]  = motor_acc
    cmd_x[6]  = (pulse_x >> 24) & 0xFF
    cmd_x[7]  = (pulse_x >> 16) & 0xFF
    cmd_x[8]  = (pulse_x >> 8) & 0xFF
    cmd_x[9]  = pulse_x & 0xFF
    cmd_x[10] = 0x00  # 0 = 相对模式
    cmd_x[11] = 0x00  # 0 = 不启用多机同步
    cmd_x[12] = 0x6B  # 校验字节
#   print(b'cmd')
    cmd_y = bytearray(13)
    cmd_y[0]  = motor_addr_1
    cmd_y[1]  = 0xFD  # 功能码：位置模式
    cmd_y[2]  = motor_dir_y
    cmd_y[3]  = (motor_speed >> 8) & 0xFF
    cmd_y[4]  = motor_speed & 0xFF
    cmd_y[5]  = motor_acc
    cmd_y[6]  = (pulse_y >> 24) & 0xFF
    cmd_y[7]  = (pulse_y >> 16) & 0xFF
    cmd_y[8]  = (pulse_y >> 8) & 0xFF
    cmd_y[9]  = pulse_y & 0xFF
    cmd_y[10] = 0x00  # 0 = 相对模式
    cmd_y[11] = 0x00  # 0 = 不启用多机同步
    cmd_y[12] = 0x6B  # 校验字节
    #   print(b'cmd')
    uart.send(cmd_x)
    time.sleep_ms(4)
    uart.send(cmd_y)


def position_pid_y(dy):
    global error_last, integral   #计算 积分 和 微分

    target = 0                    #设置 目标值
    error = target - dy           #计算当前的 误差
    integral += error             #累加历史误差
    derivative = error - error_last  #计算误差变化率 = 当前误差 - 上一次误差
    error_last = error

    output = Kp * error + Ki * integral + Kd * derivative

    # 限制输出
    output = max(min(output, 10), -10)

    # 计算脉冲数（可以乘个因子放大）
    pulse = int(abs(output) * step_factor)
    if abs(error) < 3:
        pulse =0

    # 设置方向
    if output >= 0:
        motor_dir = 0  # 顺时针 CW
    else:
        motor_dir = 1

    # 构造位置控制数据帧（相对位移）
    cmd = bytearray(13)
    cmd[0]  = motor_addr_1
    cmd[1]  = 0xFD  # 功能码：位置模式
    cmd[2]  = motor_dir
    cmd[3]  = (motor_speed >> 8) & 0xFF
    cmd[4]  = motor_speed & 0xFF
    cmd[5]  = motor_acc
    cmd[6]  = (pulse >> 24) & 0xFF
    cmd[7]  = (pulse >> 16) & 0xFF
    cmd[8]  = (pulse >> 8) & 0xFF
    cmd[9]  = pulse & 0xFF
    cmd[10] = 0x00  # 0 = 相对模式
    cmd[11] = 0x00  # 0 = 不启用多机同步
    cmd[12] = 0x6B  # 校验字节
#   print(b'cmd')
    uart.send(cmd)





try:
    sensor = init_sensor()
    init_display()

    # 启动传感器
    sensor.run()

    key_decide = 0

    fps = time.clock()

    frame = 0



    while True:
        fps.tick()    #时间戳
        # 这里检查退出点，某些平台可调用，确认平台支持再用
        # os.exitpoint()
        img = sensor.snapshot()    #抓取一帧当前画面图像
        #Display.show_image(img)    #把图像显示在屏幕或者IDE窗口中
        gc.collect()

        #垃圾回收

        key_1()


        # 图像预处理（灰度、二值化、闭运算）
        img_gray = img.to_grayscale()
        img_bin = img_gray.binary([(100, 255)])#.dilate(1).erode(1)

        #find_O()
        #find_r()
       # aaa = find_r_r()
        aaa = find_b_r()
#        finf_r()
        if aaa is not None:
            dx, dy, w, x, y, h, left, right, top, mod = aaa
            uart_send(dx, dy)
            #find_c(x, y, w)
            find_e(x,y,w,h,left, right, top, mod)
            #position_pid_x(dx, dy)
            #position_pid_y(dy)

#        img.draw_rectangle((40, 70, 560, 340), color=(0,250,255), thickness=4)



        #uart_get()
        #decide()
        # 显示图像（显示带框的原图）
        fps_s = fps.fps()
        #print(fps_s)          #打印实时帧率
        img.draw_string_advanced(0, 0, 30, f'FPS: {fps_s:.3f}', color=(255, 255, 255))
        Display.show_image(img)

        #frame += 1
        #print("Frame:", frame)
        #time.sleep_ms(300)


except KeyboardInterrupt:
    print("用户停止程序")

except BaseException as e:
    print(f"程序异常: {e}")

finally:
    if isinstance(sensor, Sensor):   #检查sensor是否有值
        sensor.stop()
    Display.deinit()                 #释放屏幕显示资源
    uart.deinit()
    # 允许睡眠模式
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)  #进入低功耗睡眠模式
    time.sleep_ms(100)
    MediaManager.deinit()         #停止音频、图像或视频等媒体资源管理器


#*********************************************
#                   _ooOoo_
#                  o8888888o
#                  88" . "88
#                  (| -_- |)
#                  O\  =  /O
#               ____/`---'\____
#             .'  \\|     |//  `.
#            /  \\|||  :  |||//  \
#           /  _||||| -:- |||||-  \
#           |   | \\\  -  /// |   |
#           | \_|  ''\---/''  |   |
#           \  .-\__  `-`  ___/-. /
#         ___`. .'  /--.--\  `. . __
#      ."" '&lt;  `.___\_&lt;|>_/___.'  >'"".
#     | | :  `- \`.;`\ _ /`;.`/ - ` : | |
#     \  \ `-.   \_ __\ /__ _/   .-` /  /
#======`-.____`-.___\_____/___.-`____.-'======
#                   `=---='

#                    佛祖保佑       电赛国一
#^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^




#         查找矩形区域
#        rects = img_bin.find_rects(threshold=5000)
#        for r in rects:
#            img_bin.draw_rectangle(r.rect(), color=(255, 0, 0), thickness=3)
#            x, y, w, h = r.rect()
#            cx = x + w // 2
#            cy = y + h // 2
#            img_bin.draw_cross(cx, cy, color=(0, 255, 0))
#            print("Rect:", r)

#        线段
#        lines = img.find_lines(threshold=1000, theta_margin=10, rho_margin=10)
