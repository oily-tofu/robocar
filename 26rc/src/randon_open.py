import os
import random
import cv2

# 指定图片文件夹路径
folder_path = r"D:\ros\26rc-yolo\tool_kfs"  # ← 修改为你的图片文件夹路径

# 获取文件夹中所有图片文件
img_list = [f for f in os.listdir(folder_path)
            if f.lower().endswith(('.jpg', '.png', '.jpeg', '.bmp'))]

# 判断文件夹是否为空
if not img_list:
    raise FileNotFoundError("文件夹中没有图片")

# 随机选择一张图片
img_name = random.choice(img_list)
img_path = os.path.join(folder_path, img_name)

# 打开并显示图片
img = cv2.imread(img_path)
if img is None:
    raise ValueError(f"无法读取图片：{img_path}")

cv2.imshow("Random Image", img)
print(f"已打开随机图片：{img_name}")

# 按任意键退出
cv2.waitKey(0)
cv2.destroyAllWindows()
