import time, math, os, gc, sys
from media.sensor import *      #sensor 负责摄像头操作
from media.display import *     #display 负责显示屏操作
from media.media import *       #media 管理媒体资源
from ybUtils.YbUart import YbUart


#前置变量声明
WIDTH = 640
HEIGHT = 480
sensor = None

THRESHOLDS = [
    (46, 100, -8, 127, -13, 14), #灰度
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
####################################################################################################
def get_closest_rgb(lab_threshold):
    """根据LAB阈值计算最接近的RGB颜色 / Calculate closest RGB color based on LAB threshold"""
    # 获取LAB空间的中心点值
    l_center = (lab_threshold[0] + lab_threshold[1]) // 2
    a_center = (lab_threshold[2] + lab_threshold[3]) // 2
    b_center = (lab_threshold[4] + lab_threshold[5]) // 2
    return image.lab_to_rgb((l_center,a_center,b_center))

def process_blobs(img, blobs, color):
    """处理检测到的色块 / Process detected color blobs"""
    for blob in blobs:
        img.draw_rectangle(blob[0:4], color=color, thickness=4)
        img.draw_cross(blob[5], blob[6], color=color, thickness=2)

def find_O():
    threshold = THRESHOLDS[3]   # 颜色
    detect_color = get_closest_rgb(threshold)
    blobs = img.find_blobs([threshold], area_threshold=5000, merge=True)
    if blobs:
        process_blobs(img, blobs, detect_color)

def find_jg():
    threshold = THRESHOLDS[0]
    detect_color = get_closest_rgb(threshold)

    blobs = img_bin.find_blobs([(228, 255)], area_threshold=50)
    if blobs:
        for blob in blobs:
            w = blob[2]
            h = blob[3]
            if w < 25 and h < 25:
                img_bin.draw_rectangle(blob[0:4], color=(0,250,255), thickness=4)
                img_bin.draw_cross(blob[5], blob[6], color=(0,250,255), thickness=2)
#                x = blob[0]
#                y = blob[1]
                a = w * h
                print("find bai", a)
    else:
        print("not bai")

####################################################################################################



def is_purple(hd):
    if 73 < hd < 146:
        return True





try:
    sensor = init_sensor()
    init_display()

    # 启动传感器
    sensor.run()
    uart = YbUart(baudrate=115200)

    fps = time.clock()

    frame = 0



    while True:
        fps.tick()
        count = 0
        img = sensor.snapshot()
        img_bin = img.to_grayscale()#.binary([(73, 146)])

        gc.collect()
        print(fps.fps())


        for y in range(0, HEIGHT, 8):  # 每隔4行扫描，加快处理速度
            for x in range(0, WIDTH, 8):  # 每隔4列扫描
                hd = img_bin.get_pixel(x, y)
                if is_purple(hd):
                    img.draw_cross(x, y, color=(255, 0, 255), thickness=2)
                    count += 1
        print("找到紫色点数量：", count)

        Display.show_image(img)
#(38, 64, 10, 21, -19, -5)
#50，16,-18
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
    uart.deinit()
    # 允许睡眠模式
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)  #进入低功耗睡眠模式
    time.sleep_ms(100)
    MediaManager.deinit()         #停止音频、图像或视频等媒体资源管理器






