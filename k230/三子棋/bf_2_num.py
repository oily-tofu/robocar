import time, math, os, gc, sys
from media.sensor import *      #sensor 负责摄像头操作
from media.display import *     #display 负责显示屏操作
from media.media import *       #media 管理媒体资源

#前置变量声明
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


def process_blobs(img, blobs, color):
    """处理检测到的色块 / Process detected color blobs"""
    for blob in blobs:
        img.draw_rectangle(blob[0:4], color=color, thickness=4)
        img.draw_cross(blob[5], blob[6], color=color, thickness=2)
        x = blob[0]
        y = blob[1]
        w = blob[2]
        h = blob[3]
#        pto_data = pto.get_color_data(x, y, w, h)
#        #uart.send(pto_data)
#        print(pto_data)

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
            if der[j][1] > der[j + 1][1]:  # 按照 y 坐标比较
                der[j], der[j + 1] = der[j + 1], der[j]
    return der


def odom_num():
    rule.clear()
    if estimate_board == True:
        a = (int(rect_list[8][1]) - int(rect_list[6][1]))*2
        print(a)
        b = int(rect_list[6][0]) - int(rect_list[8][0])
        print(b)
        if (a < b):
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


        if estimate_board == False:
            estimate_board = find_board()

        draw_board()

        if estimate_board == True:
            order_y = order(rect_list)
            for i in order_y:
                print(i)


        odom_num()
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


#    def group_rows(points, y_threshold=20):
#        """将点按照 y 坐标聚类为多行"""
#        points = sorted(points, key=lambda p: p[1])  # 按 y 排序
#        rows = []
#        current_row = []

#        for p in points:
#            if not current_row:
#                current_row.append(p)
#            elif abs(p[1] - current_row[-1][1]) < y_threshold:
#                current_row.append(p)
#            else:
#                rows.append(current_row)
#                current_row = [p]

#        if current_row:
#            rows.append(current_row)

#        return rows

#    rows = group_rows(centers)

#    # 2. 每行内按 X 排序
#    for row in rows:
#        row.sort(key=lambda p: p[0])

#    # 3. 整理成一个排序好的列表（从左到右、上到下）
#    sorted_centers = [pt for row in rows for pt in row]

#    # 4. 可视化编号
#    for i, (cx, cy) in enumerate(sorted_centers):
#        img.draw_string(cx - 10, cy - 10, str(i), color=(255, 255, 0))
