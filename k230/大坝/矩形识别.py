import time, math, os, gc, sys
from media.sensor import *      #sensor 负责摄像头操作
from media.display import *     #display 负责显示屏操作
from media.media import *       #media 管理媒体资源
from ybUtils.YbUart import YbUart
#前置变量声明
WIDTH = 640
HEIGHT = 480
sensor = None
uart = YbUart(baudrate=115200)

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

def get_closest_rgb(lab_threshold):
    """根据LAB阈值计算最接近的RGB颜色 / Calculate closest RGB color based on LAB threshold"""
    # 获取LAB空间的中心点值
    l_center = (lab_threshold[0] + lab_threshold[1]) // 2
    a_center = (lab_threshold[2] + lab_threshold[3]) // 2
    b_center = (lab_threshold[4] + lab_threshold[5]) // 2
    return image.lab_to_rgb((l_center,a_center,b_center))


def midpoint(p1, p2):
    return ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)

def find_r_r():
    rects = img_bin.find_rects(threshold=6000)
    for r in rects:
        x, y, w, h = r.rect()
        cx = x + w // 2
        cy = y + h // 2
        corners = r.corners()
        s = w * h
        c = w / h
        if 6000 < s < 120000 and 0.75 < c < 1.55:

            img.draw_cross(cx, cy, color=(0, 255, 0), thickness=2)
            img.draw_cross(corners[0][0], corners[0][1], color=(0, 0, 0))
            img.draw_cross(corners[1][0], corners[1][1], color=(0, 255, 0))
            img.draw_cross(corners[2][0], corners[2][1], color=(0, 255, 0))
            img.draw_cross(corners[3][0], corners[3][1], color=(0, 255, 0))
            img.draw_line(corners[0][0], corners[0][1],corners[1][0], corners[1][1], color=(0, 255, 0), thickness=2)
            img.draw_line(corners[1][0], corners[1][1],corners[2][0], corners[2][1], color=(0, 255, 0), thickness=2)
            img.draw_line(corners[2][0], corners[2][1],corners[3][0], corners[3][1], color=(0, 255, 0), thickness=2)
            img.draw_line(corners[3][0], corners[3][1],corners[0][0], corners[0][1], color=(0, 255, 0), thickness=2)

#            print("find jx", dx)
#            print("find jy", dy)
            uart.send("@")
            uart.send(str(dx))
            uart.send(" ")
            uart.send(str(dy))
            uart.send("#")
            uart.send("%")



try:
    sensor = init_sensor()
    init_display()


    # 启动传感器
    sensor.run()

    fps = time.clock()

    frame = 0

    THRESHOLDS = [
        (0, 66, 7, 127, 3, 127),    # 红色阈值
        (42, 100, -128, -17, 6, 66),     # 绿色阈值
        (43, 99, -43, -4, -56, -7),       # 蓝色阈值
        (69, 89, -20, 1, -14, -2)   # 白
    ]

    # 选择要检测的颜色索引 (0:红, 1:绿, 2:蓝) / Select color index to detect
    color_index = 3  # 可以修改这个值来选择检测不同的颜色
    threshold = THRESHOLDS[color_index]
    detect_color = get_closest_rgb(threshold)




    while True:
        fps.tick()    #时间戳
        # 这里检查退出点，某些平台可调用，确认平台支持再用
        # os.exitpoint()
        img = sensor.snapshot()    #抓取一帧当前画面图像
        #Display.show_image(img)    #把图像显示在屏幕或者IDE窗口中
        gc.collect()              #垃圾回收
        print(fps.fps())          #打印实时帧率

        # 图像预处理（灰度、二值化、闭运算）
        img_gray = img.to_grayscale()
        img_bin = img_gray.binary([(100, 255)])

        find_r_r()

#        lines = img.find_lines(threshold=1000, theta_margin=10, rho_margin=10)









#         查找矩形区域

            #print("Rect:", r)

#        for blob in img_bin.find_blobs([(100, 255)], area_threshold=10000):
#            # 可视化绘制旋转的矩形框
#            img.draw_rectangle(blob.rect(), color=(255, 0, 0), thickness=2)
#            img.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))

        # 显示图像（显示带框的原图）
        Display.show_image(img)

#        frame += 1
#        print("Frame:", frame)
        #time.sleep(1)

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



