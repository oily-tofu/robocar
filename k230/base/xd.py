import time
from media.sensor import *
from media.display import *
from media.media import *
from libs.PipeLine import PipeLine

PICTURE_WIDTH = 160
PICTURE_HEIGHT = 120
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480

# 初始化显示
display_mode = "LCD"
pl = PipeLine(rgb888p_size=[640, 360], display_size=[DISPLAY_WIDTH, DISPLAY_HEIGHT], display_mode=display_mode)
pl.create(ch1_frame_size=[PICTURE_WIDTH, PICTURE_HEIGHT])

def scale_coordinates(data_tuple, target_resolution="640x480"):
    if not isinstance(data_tuple, tuple) or len(data_tuple) < 4:
        raise TypeError("期望输入至少包含4个元素的元组")

    x1, y1, x2, y2 = data_tuple[:4]
    scale_x = 640 / PICTURE_WIDTH
    scale_y = 480 / PICTURE_HEIGHT

    return (
        round(x1 * scale_x),
        round(y1 * scale_y),
        round(x2 * scale_x),
        round(y2 * scale_y),
    )

# 主测试循环
while True:
    # 获取图像
    img_src = pl.sensor.snapshot(chn=CAM_CHN_ID_1)

    # 查找线段
    lines = img_src.find_line_segments(merge_distance=15, max_theta_diff=10)

    # 创建用于显示的图像
    img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
    img.clear()

    # 绘制所有线段
    for line in lines:
        scaled = scale_coordinates(line.line())
        img.draw_line(scaled, color=(255, 0, 0), thickness=3)

    # 显示图像
    Display.show_image(img, 0, 0, Display.LAYER_OSD3)

    # 打印调试信息（可选）
    print(f"检测到 {len(lines)} 条线段")

    time.sleep_ms(100)
