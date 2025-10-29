import time, math, os, gc, sys
from media.sensor import *      #sensor 负责摄像头操作
from media.display import *     #display 负责显示屏操作
from media.media import *       #media 管理媒体资源

#前置变量声明
WIDTH = 640
HEIGHT = 480
sensor = None


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


def find_board():
    rect_list.clear()
    for blob in img_bin.find_blobs([(100, 255)], area_threshold=5000):
        x, y, w, h = blob.rect()
        if w > 200 and h > 200:
                continue
        img_bin.draw_rectangle(blob.rect(), color=(255, 0, 0), thickness=2)
        img_bin.draw_cross(blob.cx(), blob.cy(), color=(0, 255, 0))
        rect_list.append(blob.rect())
        print("Rect:", blob.rect())
    if rect_list != []:
        return True
    else:
        return False

def draw_board():
    if estimate_board == True:
        for rect in rect_list:
            x, y, w, h = rect
            cx = x + w // 2
            cy = y + h // 2
            img_bin.draw_rectangle(rect, color=(255, 0, 0), thickness=2)
            img_bin.draw_cross(cx, cy, color=(0, 255, 0))


def order(der):
    n = len(der)
    for i in range(n):
        for j in range(0, n - i - 1):
            # der = (x, y, w, h)
            if der[j][1] > der[j + 1][1]:  # 按照 y 坐标比较
                der[j], der[j + 1] = der[j + 1], der[j]
    return der

def odom_num():
    rule.clear()
    if estimate_board == True:
       rule = rect_list[0:6]


try:
    sensor = init_sensor()
    init_display()

    # 启动传感器
    sensor.run()

    fps = time.clock()

    frame = 0

    rule = []
    rect_list = []
    estimate_board = False



    while True:
        fps.tick()    #时间戳

        img = sensor.snapshot()    #抓取一帧当前画面图像

        gc.collect()              #垃圾回收



        img_gray = img.to_grayscale()
        img_bin = img_gray.binary([(100, 255)])



        if estimate_board == False:
            estimate_board = find_board()

        draw_board()

        if estimate_board == True:
            order_y = order(rect_list)
            for i in order_y:
                print(i)

        odom_num()



        # 显示图像（显示带框的原图）
        Display.show_image(img)

        frame += 1
        print("Frame:", frame)
        time.sleep(1)

except KeyboardInterrupt:
    print("用户停止程序")

except BaseException as e:
    print(f"程序异常: {e}")

finally:
    if isinstance(sensor, Sensor):   #检查sensor是否有值
        sensor.stop()
    Display.deinit()                 #释放屏幕显示资源
    # 允许睡眠模式
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)  #进入低功耗睡眠模式
    time.sleep_ms(100)
    MediaManager.deinit()         #停止音频、图像或视频等媒体资源管理器


