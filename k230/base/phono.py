import time, image
from media.sensor import *
from media.display import *
from media.media import *

# 初始化显示屏（640x480 分辨率）
Display.init(Display.ST7701, width=640, height=480, osd_num=1, to_ide=True)

# 初始化媒体资源管理器（必须调用一次）
MediaManager.init()

# 加载图像并预处理（灰度 + 二值 + 闭运算）
original_img = image.Image("sdcard/78.jpg", copy_to_fb=True)

# 死循环进行处理与绘制
frame = 0
while True:
    # 每次循环从原图生成处理图
    img_proc = original_img.to_grayscale().binary([(100, 255)])

    # 查找矩形并绘制
    rects = img_proc.find_rects(threshold=10000)
    for r in rects:
        img_proc.draw_rectangle(r.rect(), color=(255, 0, 0), thickness=3)
        print("Rect:", r)

    # 显示图像
    Display.show_image(img_proc)

    # 控制帧率输出
    frame += 1
    print("Frame:", frame)
    time.sleep(0.1)
