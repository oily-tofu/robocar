import time, math, os, gc, sys
from media.sensor import *
from media.display import *     #media 管理媒体资源


WIDTH = 640
HEIGHT = 480
sensor = None

THRESHOLDS = [
    (0, 66, 7, 127, 3, 127),    # 红色阈值
    (71, 81, -8, 7, 3, 30),     # 橙
    (18, 47, -6, 3, -8, 6),       # 黑
    (46, 100, -8, 127, -13, 14)   # 白
]

def init_sensor():
    sensor = Sensor(width=WIDTH, height=HEIGHT, fps=30)
    sensor.reset()
    sensor.set_framesize(width=WIDTH, height=HEIGHT)
    sensor.set_pixformat(Sensor.RGB565)
    return sensor

def init_display():
    Display.init(Display.ST7701, to_ide=True)
    MediaManager.init()

def get_closest_rgb(lab_threshold):
    l_center = (lab_threshold[0] + lab_threshold[1]) // 2
    a_center = (lab_threshold[2] + lab_threshold[3]) // 2
    b_center = (lab_threshold[4] + lab_threshold[5]) // 2
    return image.lab_to_rgb((l_center,a_center,b_center))


def process_blobs(img, blobs, color):
    for blob in blobs:
        img.draw_rectangle(blob[0:4], color=color, thickness=4)
        img.draw_cross(blob[5], blob[6], color=color, thickness=2)
        x = blob[0]
        y = blob[1]
        w = blob[2]
        h = blob[3]

def find_O():
    threshold = THRESHOLDS[3]   # 颜色
    detect_color = get_closest_rgb(threshold)
    blobs = img.find_blobs([threshold], area_threshold=5000, merge=True)
    if blobs:
        process_blobs(img, blobs, detect_color)

def find_X():
    threshold = THRESHOLDS[2]   # 颜色
    detect_color = get_closest_rgb(threshold)
    blobs = img.find_blobs([threshold], area_threshold=5000, merge=True)
    if blobs:
        process_blobs(img, blobs, detect_color)

def find_Q():
    threshold = THRESHOLDS[1]   # 颜色
    detect_color = get_closest_rgb(threshold)
    blobs = img.find_blobs([threshold], area_threshold=5000, merge=True)
    if blobs:
        process_blobs(img, blobs, detect_color)


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
            if der[j][1] > der[j + 1][1]:  # y
                der[j], der[j + 1] = der[j + 1], der[j]
    return der


def decide_yaw(yaw):
    yaw_1 = yaw[0:3]
    n = len(yaw_1)
    for i in range(n):
        for j in range(0, n - i - 1):
            # der = (x, y, w, h)
            if yaw_1[1][j] > yaw_1[1][j + 1]:  #x
                yaw_1[j], yaw_1[j + 1] = yaw_1[j + 1], yaw_1[j]
    ddd = yaw[0][1] - yaw[2][1]
    return ddd





def odom_num(odom,yaw):
    rule.clear()
    if 0 < yaw and yaw < 20:
        rule.append(odom[2])
        rule.append(odom[1])
        rule.append(odom[0])
        rule.append(odom[5])
        rule.append(odom[4])
        rule.append(odom[3])
        rule.append(odom[8])
        rule.append(odom[7])
        rule.append(odom[6])
        img.draw_string_advanced(rule[0][0], rule[0][1], 30, str(1), color=(255, 255, 255), scale=2)
        img.draw_string_advanced(rule[1][0], rule[1][1], 30, str(2), color=(255, 255, 255), scale=2)
        img.draw_string_advanced(rule[2][0], rule[2][1], 30, str(3), color=(255, 255, 255), scale=2)
        img.draw_string_advanced(rule[3][0], rule[3][1], 30, str(4), color=(255, 255, 255), scale=2)
        img.draw_string_advanced(rule[4][0], rule[4][1], 30, str(5), color=(255, 255, 255), scale=2)
        img.draw_string_advanced(rule[5][0], rule[5][1], 30, str(6), color=(255, 255, 255), scale=2)
        img.draw_string_advanced(rule[6][0], rule[6][1], 30, str(7), color=(255, 255, 255), scale=2)
        img.draw_string_advanced(rule[7][0], rule[7][1], 30, str(8), color=(255, 255, 255), scale=2)
        img.draw_string_advanced(rule[8][0], rule[8][1], 30, str(9), color=(255, 255, 255), scale=2)
    elif -20 < yaw and yaw < 0:
        rule.append(rect_list[2])
        rule.append(rect_list[1])
        rule.append(rect_list[0])
        rule.append(rect_list[5])
        rule.append(rect_list[4])
        rule.append(rect_list[3])
        rule.append(rect_list[8])
        rule.append(rect_list[7])
        rule.append(rect_list[6])
        img.draw_string_advanced(rule[0][0], rule[0][1], 30, str(1), color=(255, 255, 255), scale=2)
        img.draw_string_advanced(rule[1][0], rule[1][1], 30, str(2), color=(255, 255, 255), scale=2)
        img.draw_string_advanced(rule[2][0], rule[2][1], 30, str(3), color=(255, 255, 255), scale=2)
        img.draw_string_advanced(rule[3][0], rule[3][1], 30, str(4), color=(255, 255, 255), scale=2)
        img.draw_string_advanced(rule[4][0], rule[4][1], 30, str(5), color=(255, 255, 255), scale=2)
        img.draw_string_advanced(rule[5][0], rule[5][1], 30, str(6), color=(255, 255, 255), scale=2)
        img.draw_string_advanced(rule[6][0], rule[6][1], 30, str(7), color=(255, 255, 255), scale=2)
        img.draw_string_advanced(rule[7][0], rule[7][1], 30, str(8), color=(255, 255, 255), scale=2)
        img.draw_string_advanced(rule[8][0], rule[8][1], 30, str(9), color=(255, 255, 255), scale=2)
    else:
        pass


def find_b_rect():
    rects = img_bin.find_rects(threshold=5000)
    for r in rects:
        img_bin.draw_rectangle(r.rect(), color=(255, 0, 0), thickness=3)
        x, y, w, h = r.rect()
        cx = x + w // 2
        cy = y + h // 2
        img_bin.draw_cross(cx, cy, color=(0, 255, 0))
        print("Rect:", r)

def find_cross(cross):
    cross_1 = []
    cross_2 = []
    cross_3 = []
    cross_1.clear()
    cross_2.clear()
    cross_3.clear()
    y_max = cross[0]
    y_min = cross[8]
    y_cross = cross[4]
    for i in cross:
        x = i[0] - y_cross[0]
        y = i[1] - y_cross[1]
        x2 = x*x
        y2 = y*y
        d = math.sqrt(x2 + y2)
        if 65 < d and d < 80:
            print(d)
            cross_1.append(i)
    cross_2 = order(cross_1)
    cross_3.append(cross_2[0])
    if cross_2[1][0] < cross_2[2][0]:
        cross_3.append(cross_2[1])
        cross_3.append(cross_2[2])
    else:
        cross_3.append(cross_2[2])
        cross_3.append(cross_2[1])
    cross_3.append(cross_2[3])
    img.draw_string_advanced(cross_3[0][0], cross_3[0][1], 30, str(2), color=(255, 255, 255), scale=2)
    img.draw_string_advanced(cross_3[1][0], cross_3[1][1], 30, str(4), color=(255, 255, 255), scale=2)
    img.draw_string_advanced(y_cross[0], y_cross[1], 30, str(5), color=(255, 255, 255), scale=2)
    img.draw_string_advanced(cross_3[2][0], cross_3[2][1], 30, str(6), color=(255, 255, 255), scale=2)
    img.draw_string_advanced(cross_3[3][0], cross_3[3][1], 30, str(8), color=(255, 255, 255), scale=2)

def find_cross_1(cross):
    cross_4 = []
    cross_5 = []
    cross_6 = []
    cross_4.clear()
    cross_5.clear()
    cross_6.clear()
    y_max = cross[0]
    y_min = cross[8]
    y_cross = cross[4]
    for i in cross:
        x = i[0] - y_cross[0]
        y = i[1] - y_cross[1]
        x2 = x*x
        y2 = y*y
        d = math.sqrt(x2 + y2)
        if 80 < d and d < 120:
            print(d)
            cross_4.append(i)
    cross_5 = order(cross_4)

    if cross_5[0][0] < cross_5[1][0]:
        cross_6.append(cross_5[0])
        cross_6.append(cross_5[1])
    else:
        cross_6.append(cross_5[1])
        cross_6.append(cross_5[0])
    if cross_5[2][0] < cross_5[3][0]:
        cross_6.append(cross_5[2])
        cross_6.append(cross_5[3])
    else:
        cross_6.append(cross_5[3])
        cross_6.append(cross_5[2])
    img.draw_string_advanced(cross_6[0][0], cross_6[0][1], 30, str(1), color=(255, 255, 255), scale=2)
    img.draw_string_advanced(cross_6[1][0], cross_6[1][1], 30, str(3), color=(255, 255, 255), scale=2)
    img.draw_string_advanced(cross_6[2][0], cross_6[2][1], 30, str(7), color=(255, 255, 255), scale=2)
    img.draw_string_advanced(cross_6[3][0], cross_6[3][1], 30, str(9), color=(255, 255, 255), scale=2)





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
        # 这里检查退出点，某些平台可调用，确认平台支持再用
        # os.exitpoint()
        img = sensor.snapshot()    #抓取一帧当前画面图像
        #Display.show_image(img)    #把图像显示在屏幕或者IDE窗口中
        gc.collect()              #垃圾回收
        #print(fps.fps())          #打印实时帧率

        # 图像预处理（灰度、二值化、闭运算）
        img_gray = img.to_grayscale()
#        img_bin = img_gray.binary([(100, 255)]).close(1)
        img_bin = img_gray.binary([(100, 255)])




        if estimate_board == False:
            estimate_board = find_board()

        draw_board()



        if estimate_board == True:
            order_y = order(rect_list)
            yaw = decide_yaw(order_y)
            if -20 < yaw and yaw < 20:
                odom_num(order_y,yaw)
            else:
                find_cross(order_y)
                find_cross_1(order_y)


#            for i in order_y:
#                print(i)

#        if estimate_board == True:
#            odom_num()
#        find_Q()



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


#        lines = img.find_lines(threshold=1000, theta_margin=10, rho_margin=10)

#         查找矩形区域
#        rects = img_bin.find_rects(threshold=5000)
#        for r in rects:
#            img_bin.draw_rectangle(r.rect(), color=(255, 0, 0), thickness=3)
#            x, y, w, h = r.rect()
#            cx = x + w // 2
#            cy = y + h // 2
#            img_bin.draw_cross(cx, cy, color=(0, 255, 0))
#            print("Rect:", r)
