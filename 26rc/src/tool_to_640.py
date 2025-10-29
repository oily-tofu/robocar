from PIL import Image
import os
import random
import cv2
import numpy as np

def random_open():
    folder_path = r"D:/ros/26rc-yolo/yolo/backgrond_ed"
    # 获取文件夹中所有图片文件
    img_list = [f for f in os.listdir(folder_path)
                if f.lower().endswith(('.jpg', '.png', '.jpeg', '.bmp'))]

    if not img_list:
        raise FileNotFoundError("文件夹中没有图片")

    # 随机打乱图片列表
    random.shuffle(img_list)
    return folder_path, img_list

# 调用函数
folder_path, img_list = random_open()

output_dir = r"D:\ros\26rc-yolo\backgrond_ing"
os.makedirs(output_dir, exist_ok=True)

num = 0
for img_name in img_list:
    num += 1
    img_path = os.path.join(folder_path, img_name)

    # 打开图片
    img = Image.open(img_path)

    # 获取尺寸
    w, h = img.size
    print(f"原始大小: {num}")

    # 调整大小
    img = img.resize((640, 640))
    img = img.convert("RGB")

    # 转换为OpenCV格式（BGR）
    img_cv = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

    # 生成保存路径
    save_path = os.path.join(output_dir, f"CQUPT_{num:05d}.jpg")

    # 保存图片
    cv2.imwrite(save_path, img_cv)

print("✅ 所有图片已随机打开并保存。")
