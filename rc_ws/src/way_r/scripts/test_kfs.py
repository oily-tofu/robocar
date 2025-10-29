#!/usr/bin/env python3
import sys
import re
import time
import os

coordinates = []

def monitor_coordinates():
    print("开始", flush=True)

    
    pattern = re.compile(r'\((\d+),(\d+)\)')

    try:
        while True:
            line = input("输入坐标: ").strip()
            matches = pattern.findall(line)

            if matches:
                for match in matches:
                    x, y = map(int, match)
                    timestamp = time.strftime("%H:%M:%S")
                    coordinates.append((x, y, timestamp))
                    print(f" 已记录坐标: ({x}, {y})   {timestamp}", flush=True)
            else:
                print(" 格式错误(x, y)", flush=True)
    except KeyboardInterrupt:
        print("\n 停止监听", flush=True)

# def save_coordinates(filename="coordinates.log"):
#     if not coordinates:
#         print(" 没有记录任何坐标，未保存。", flush=True)
#         return

#     with open(filename, "w") as f:
#         for x, y, t in coordinates:
#             f.write(f"({x}, {y})  # {t}\n")

#     print(f" 保存 {os.path.abspath(filename)} （共 {len(coordinates)} 个）", flush=True)

if __name__ == "__main__":
    # if len(sys.argv) > 1:
    #     filename = sys.argv[1]
    # else:
    #     filename = "coordinates_" + time.strftime("%Y%m%d_%H%M%S") + ".log"

    monitor_coordinates()
    # save_coordinates(filename)
