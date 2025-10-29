import time, math, os, gc, sys
from media.sensor import *      #sensor 负责摄像头操作
from media.display import *     #display 负责显示屏操作
from media.media import *       #media 管理媒体资源
#from ybUtils.YbUart import YbUart


#前置变量声明
WIDTH = 640
HEIGHT = 480
sensor = None
#uart = YbUart(baudrate=115200)

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




def find_b_r():
    blobs = img_bin.find_blobs([(100, 255)], area_threshold=50, x_stride=10, y_stride=10)

    if blobs:
        for blob in blobs:
            w = blob[2]
            h = blob[3]
            s = w * h
            c = w / h
            a = 0
            if 6000 < s < 120000 and 1.25 < c < 1.55:
                b = 240 + (w - 100) * 0.25
#                print(w)
#                print(int(b))

                img.draw_cross(320, int(b), color=(0,0,0), thickness=2)

                img.draw_rectangle(blob[0:4], color=(0,250,255), thickness=6)
                img.draw_cross(blob[5], blob[6], color=(0,250,255), thickness=2)
                x = blob[5]
                y = blob[6]
                dx = x - 320
                dy = 240 - y
                return (dx , dy)

#def uart_get():
#    data = uart.read()
#    buffer = ""
#    in_message = False
#    if data:
#        data_1 = data.decode()
#        print("uart: ",data_1)
#        for ch in data_1:
#            if ch == "@":
#                in_message = True
#                buffer = ""
#            elif ch == "#" and in_message:
#                in_message = False
#                # 去掉两端空白并按空格分割
#                parts = buffer.strip().split(" ")
#                if len(parts) == 2:
#                    try:
#                        dx = int(parts[0])
#                        dy = int(parts[1])
#                        print("接收到 dx:", dx, "dy:", dy)
#                    except:
#                        print("解析失败：", buffer)
#                        return (dx, dy)
#                buffer = ""
#            elif in_message:
#                buffer += ch



#def uart_send(dx, dy):
#    if dx > 2:

#    print("find jx", dx)
#    print("find jy", dy)
#    uart.send("@")
#    uart.send(str(dx))
#    uart.send(" ")
#    uart.send(str(dy))
#    uart.send("#")
#    uart.send("%")

#def find_b_r_new():
#    aaa = uart_get()
#    if aaa is not None:
#        num, lc = aaa
#        if num == 1:
#            pass
#        if num == 2:
#            pass
#        if num == 3:
#            pass
#        if num == 4:
#            pass






try:
    sensor = init_sensor()
    init_display()

    # 启动传感器
    sensor.run()



    fps = time.clock()

    frame = 0



    while True:
        fps.tick()    #时间戳
        # 这里检查退出点，某些平台可调用，确认平台支持再用
        # os.exitpoint()
        img = sensor.snapshot()    #抓取一帧当前画面图像
        #Display.show_image(img)    #把图像显示在屏幕或者IDE窗口中
        gc.collect()              #垃圾回收
        #print(fps.fps())          #打印实时帧率



        # 图像预处理（灰度、二值化、闭运算）
        img_gray = img.to_grayscale()
        img_bin = img_gray.binary([(100, 255)])#.dilate(1).erode(1)

        #find_O()
        #find_r()
        aaa = find_b_r()
        if aaa is not None:
            dx, dy = aaa
#            uart_send(dx, dy)

#        img.draw_rectangle((40, 70, 560, 340), color=(0,250,255), thickness=4)



#        uart_get()
        #decide()
        # 显示图像（显示带框的原图）
        Display.show_image(img)

        #frame += 1
        #print("Frame:", frame)
       # time.sleep(1)


except KeyboardInterrupt:
    print("用户停止程序")

except BaseException as e:
    print(f"程序异常: {e}")

finally:
    if isinstance(sensor, Sensor):   #检查sensor是否有值
        sensor.stop()
    Display.deinit()                 #释放屏幕显示资源
#    uart.deinit()
    # 允许睡眠模式
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)  #进入低功耗睡眠模式
    time.sleep_ms(100)
    MediaManager.deinit()         #停止音频、图像或视频等媒体资源管理器

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
