import time, math, os, gc, sys
from media.sensor import *      #sensor 负责摄像头操作
from media.display import *     #display 负责显示屏操作
from media.media import *       #media 管理媒体资源
from ybUtils.YbUart import YbUart
from ybUtils.YbKey import YbKey
from machine import Pin
from collections import deque
import cv_lite
import _thread









import time, math, os, gc, sys
from media.sensor import *      #sensor 负责摄像头操作
from media.display import *     #display 负责显示屏操作
from media.media import *       #media 管理媒体资源
from ybUtils.YbUart import YbUart
from ybUtils.YbKey import YbKey
from machine import Pin
from collections import deque
import cv_lite
import _thread


#前置变量声明
image_shape = [480, 640]
sensor = None
uart = YbUart(baudrate=115200)
key = YbKey()



def init_sensor():
    global image_shape
    """初始化摄像头 / Initialize camera sensor"""
    sensor = Sensor(id=2, width=image_shape[1], height=image_shape[0], fps = 30)
    sensor.reset()
    sensor.set_framesize(width=image_shape[1], height=image_shape[0])
    sensor.set_pixformat(Sensor.GRAYSCALE)
    return sensor

def init_display():
    """初始化显示 / Initialize display"""
    Display.init(Display.ST7701, to_ide=True)
    MediaManager.init()


def len_l(p1,p2):
    l1 = abs(p1[0] - p2[0])
    l2 = abs(p1[1] - p2[1])
    if l1 < 2:
        l = abs(l2)
    elif l2 < 2:
        l = abs(l1)
    else:
        l = math.sqrt(l1*l1 + l2*l2)
    return l

def decide(p1,p2):
    l = len_l(p1,p2)
    print(l)
    if l < 40:
        return True
    else:
        return False

def find_b_r():

    blobs = img_bin.find_blobs([(100, 255)], area_threshold=50, x_stride=10, y_stride=10)
    if blobs:
        for blob in blobs:
            w = blob[2]
            h = blob[3]
            s = w * h
            c = w / h
            if 6000 < s < 120000 and 0.75 < c < 1.55:
                img.draw_rectangle(blob[0:4], color=(0,250,255), thickness=4)
            #if True:
                b = 240 + (w - 100) * 0.25
                bx = blob[5]
                by = blob[6]
                return (bx, by)


def find_r_r_r():

    image_shape = [480,640]
    canny_thresh1      = 50        # Canny 边缘检测低阈值 / Canny low threshold
    canny_thresh2      = 150       # Canny 边缘检测高阈值 / Canny high threshold
    approx_epsilon     = 0.04      # 多边形拟合精度比例（越小拟合越精确）/ Polygon approximation accuracy
    area_min_ratio     = 0.001     # 最小面积比例（相对于图像总面积）/ Min area ratio
    max_angle_cos      = 0.3       # 最大角度余弦（越小越接近矩形）/ Max cosine of angle between edges
    gaussian_blur_size = 5         # 高斯模糊核尺寸（奇数）/ Gaussian blur kernel size
    # 拍摄一帧图像 / Capture a frame
    img_np = img_bin.to_numpy_ref()

    # 调用底层矩形检测函数
    # 返回格式：[[x0, y0, w0, h0, c1.x, c1.y, c2.x, c2.y, c3.x, c3.y, c4,x, c4.y], [x1, y1, w1, h1,c1.x, c1.y, c2.x, c2.y, c3.x, c3.y, c4,x, c4.y], ...]
    rects = cv_lite.grayscale_find_rectangles_with_corners(
        image_shape, img_np,
        canny_thresh1, canny_thresh2,
        approx_epsilon,
        area_min_ratio,
        max_angle_cos,
        gaussian_blur_size
    )

    if not rects:
        return None


    for rect in rects:

        x, y, w, h = rect[0:4]
        c1_x, c1_y = rect[4], rect[5]
        c2_x, c2_y = rect[6], rect[7]
        c3_x, c3_y = rect[8], rect[9]
        c4_x, c4_y = rect[10], rect[11]
        cx = x + w//2
        cy = y + h//2
        s = w * h
        c = w / h
        left_l = len_l((c2_x, c2_y),(c3_x, c3_y))
        right_l = len_l((c1_x, c1_y),(c4_x, c4_y))
        top_l = len_l((c1_x, c1_y),(c2_x, c2_y))
        mod_l = len_l((c3_x, c3_y),(c4_x, c4_y))

        if 6000 < s < 120000 and 0.75 < c < 1.55:
        # 画矩形框

            #img.draw_cross(cx, cy, color = (0, 0, 255), thickness=2)
            img.draw_rectangle(rect[0:4], color=(0,250,255), thickness=4)

            img.draw_line(c1_x, c1_y, c2_x, c2_y, color=(255, 0, 0), thickness=4)
            img.draw_line(c2_x, c2_y, c3_x, c3_y, color=(255, 0, 0), thickness=4)
            img.draw_line(c3_x, c3_y, c4_x, c4_y, color=(255, 0, 0), thickness=4)
            img.draw_line(c4_x, c4_y, c1_x, c1_y, color=(255, 0, 0), thickness=4)

            img.draw_circle(c1_x, c1_y, 20, color=(255, 255, 255), thickness=2)
            img.draw_circle(c2_x, c2_y, 20, color=(255, 255, 255), thickness=2)
            img.draw_circle(c3_x, c3_y, 20, color=(255, 255, 255), thickness=2)
            img.draw_circle(c4_x, c4_y, 20, color=(255, 255, 255), thickness=2)

            return (cx, cy, left_l, right_l, top_l, mod_l, w, h)


def zx(cx, cy, left_l, right_l, top_l, mod_l, w, h, bx, by):
    d = False
    d = decide((cx, cy), (bx, by))

    if d:
        s = w * h
        b = int(240 + (w - 100) * 0.25)
        if s > 25000:
            if left_l >= right_l:
                #print("左下")
                cx = int(cx + (263 - mod_l) * 0.075)
            else:
                #print("右下",top_l, mod_l)
                cx = int(cx - (263 - mod_l) * 0.075)
        img.draw_cross(320, b, color = (0, 0, 0), thickness=2)
        img.draw_cross(cx, by, color = (0, 255, 255), thickness=2)

        dx = cx - 320
        dy = b - cy
        uart_send(dx, dy)


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

        img_bin = img.binary([(80, 255)])

        _thread.start_new_thread(find_b_r, ())
        _thread.start_new_thread(find_r_r_r, ())


        fps_s = fps.fps()
        #print(fps_s)          #打印实时帧率
        img.draw_string_advanced(0, 0, 30, f'FPS: {fps_s:.3f}', color=(255, 0, 255))
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



