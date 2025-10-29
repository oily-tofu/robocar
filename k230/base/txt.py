import time, math, os, gc, sys
from media.sensor import *      #sensor 负责摄像头操作
from media.display import *     #display 负责显示屏操作
from media.media import *       #media 管理媒体资源

#前置变量声明
WIDTH = 640
HEIGHT = 480
sensor = None

try:
    # 初始化传感器
    sensor = Sensor(width=WIDTH, height=HEIGHT, fps=30)   #创建摄像头对象，设置分辨率和帧率（30fps）
    sensor.reset()                                        #初始化
    #sensor.set_hmirror(False)                           #水平翻转
    #sensor.set_vflip(False)                             #垂直翻转
    sensor.set_framesize(width=WIDTH, height=HEIGHT)     #传入帧大小
    sensor.set_pixformat(Sensor.RGB565)                  #像素格式为RGB565

    # 初始化显示和媒体管理器
    Display.init(Display.ST7701, width=WIDTH, height=HEIGHT, to_ide=True)
    MediaManager.init()

    # 启动传感器
    sensor.run()

    fps = time.clock()

    while True:
        fps.tick()    #时间戳
        # 这里检查退出点，某些平台可调用，确认平台支持再用
        # os.exitpoint()

        img = sensor.snapshot()    #抓取一帧当前画面图像
        Display.show_image(img)    #把图像显示在屏幕或者IDE窗口中

        gc.collect()              #垃圾回收
        print(fps.fps())          #打印实时帧率

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
