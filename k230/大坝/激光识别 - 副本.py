import time, math, os, gc, sys
from media.sensor import *      #sensor 负责摄像头操作
from media.display import *     #display 负责显示屏操作
from media.media import *       #media 管理媒体资源
from ybUtils.YbUart import YbUart
import cv_lite


#前置变量声明
WIDTH = 640
HEIGHT = 480
sensor = None

def init_sensor():
    """初始化摄像头 / Initialize camera sensor"""
    sensor = Sensor(width=WIDTH, height=HEIGHT, fps=30)
    sensor.reset()
    sensor.set_framesize(width=WIDTH, height=HEIGHT)
    sensor.set_pixformat(Sensor.GRAYSCALE)
    return sensor

def init_display():
    """初始化显示 / Initialize display"""
    Display.init(Display.ST7701, to_ide=True)
    MediaManager.init()
####################################################################################################



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


def find_b_r():
    blobs = img_bin.find_blobs([(100, 255)], area_threshold=50, x_stride=10, y_stride=10)

    if blobs:
        for blob in blobs:
            w = blob[2]
            h = blob[3]
            s = w * h
            c = w / h
            a = 0
            if 6000 < s < 120000 and 0.75 < c < 1.55:
                b = 240 + (w - 100) * 0.25
#                print(w)
#                print(int(b))

                img.draw_cross(320, int(b), color=(0,0,0), thickness=2)

                img.draw_rectangle(blob[0:4], color=(0,250,255), thickness=6)
                img.draw_cross(blob[5], blob[6], color=(0,250,255), thickness=2)
                x = blob[5]
                y = blob[6]
                dx = x - 320
                dy = int(b) - y
                img.draw_arrow(blob[5], blob[6], 320, int(b), color = (0,0,0), thickness=2)
                return (dx , dy)

####################################################################################################


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

        img = sensor.snapshot()
        #img_gray = img.to_grayscale()


        gc.collect()

        img_bin = img.binary([(100, 255)])

        image_shape = [480,640]  # 高，宽

        # -------------------------------
        # 矩形检测可调参数 / Adjustable rectangle detection parameters
        # -------------------------------
        canny_thresh1      = 50        # Canny 边缘检测低阈值 / Canny low threshold
        canny_thresh2      = 150       # Canny 边缘检测高阈值 / Canny high threshold
        approx_epsilon     = 0.04      # 多边形拟合精度比例（越小拟合越精确）/ Polygon approximation accuracy
        area_min_ratio     = 0.001     # 最小面积比例（相对于图像总面积）/ Min area ratio
        max_angle_cos      = 0.3       # 最大角度余弦（越小越接近矩形）/ Max cosine of angle between edges
        gaussian_blur_size = 5         # 高斯模糊核尺寸（奇数）/ Gaussian blur kernel size

        # 拍摄一帧图像 / Capture a frame
        img_np = img_bin.to_numpy_ref()

        # 调用底层矩形检测函数
        # 返回格式：[x0, y0, w0, h0, x1, y1, w1, h1, ...]
        rects = cv_lite.grayscale_find_rectangles(
            image_shape, img_np,
            canny_thresh1, canny_thresh2,
            approx_epsilon,
            area_min_ratio,
            max_angle_cos,
            gaussian_blur_size
        )
        for i in range(0, len(rects), 4):
            x = rects[i]
            y = rects[i + 1]
            w = rects[i + 2]
            h = rects[i + 3]
            img.draw_rectangle(x, y, w, h, color=(255, 0, 0), thickness=6)


        frame += 1
        print("Frame:", frame)
        #time.sleep_ms(50)

        fps_s = fps.fps()
        img.draw_string_advanced(0, 0, 30, f'FPS: {fps_s:.3f}', color=(0, 0, 0))

        Display.show_image(img)




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






