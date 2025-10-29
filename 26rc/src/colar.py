import cv2
import numpy as np
import random

# 创建一张640x640的黑色背景
img_back = np.zeros((640, 640, 4), dtype=np.uint8)

# 每行4个方块，总共16个
rows, cols = 4, 4
block_size = 640 // rows  # 每个方块的宽高

def random_green():
    g = random.randint(150, 255)
    r = random.randint(0, g // 2)
    b = random.randint(0, g // 3)
    return b, g, r

index = 0
for i in range(rows):
    for j in range(cols):
        index += 1
        num = index % 3
        x1 = j * block_size
        y1 = i * block_size
        x2 = x1 + block_size
        y2 = y1 + block_size
        color = random_green()
        cv2.rectangle(img_back, (x1, y1), (x2, y2), color, -1)




#######################################################################################



# 读取图片
img = cv2.imread('jx.jpg')
img_rgba = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
if img is None:
    raise ValueError("图片路径错误或文件不存在")

# 获取图片尺寸
(h, w) = img.shape[:2]
# 设置旋转中心（这里选择图片中心）
center = (w // 2, h // 2)

# 定义原图矩形四个顶点（顺序：左上、右上、右下、左下）
pts1 = np.float32([[0, 0],
                   [w-1, 0],
                   [w-1, h-1],
                   [0, h-1]])

# 定义目标梯形四个顶点
# 比如上边缩短变成梯形

t1 = random.uniform(0, 0.3)
t2 = random.uniform(0.7, 1)
t3 = random.uniform(0, 0.3)
t4 = random.uniform(0.7, 1)
t5 = random.uniform(0.9, 1)
t6 = random.uniform(0.9, 1)
pts2 = np.float32([[w*t1, 0],      # 左上
                   [w*t2, 0],      # 右上
                   [w*t4, h*t6],      # 右下
                   [w*t3, h*t5]])       # 左下

# 计算透视变换矩阵
M = cv2.getPerspectiveTransform(pts1, pts2)

# 应用透视变换
warped = cv2.warpPerspective(img_rgba, M, (w, h))


# 设置旋转角度（顺时针为正，逆时针为负）
angle = random.randint(0, 360)

# 设置缩放比例（1.0表示不缩放）
scale = 0.5

# 获取旋转矩阵
M = cv2.getRotationMatrix2D(center, angle, scale)

# 应用旋转变换
rotated = cv2.warpAffine(warped, M, (w, h))

########################################################################################

h_r, w_r = rotated.shape[:2]
h_i, w_i = img_back.shape[:2]
x_offset = (w_i - w_r) // 2
y_offset = (h_i - h_r) // 2
# img_back[y_offset:y_offset+h_r, x_offset:x_offset+w_r] = rotated
# cv2.imshow("Rotated Centered", img_back)

b, g, r, a = cv2.split(rotated)

# alpha mask 归一化到 0~1
alpha = a / 255.0

# 目标区域
roi = img_back[y_offset:y_offset+h_r, x_offset:x_offset+w_r]

# 对每个通道叠加
for c in range(3):
    roi[:,:,c] = b * alpha + roi[:,:,c] * (1 - alpha)


# alpha 通道更新（可选，如果想保留透明）
roi[:,:,3] = np.maximum(roi[:,:,3], a)

# 放回背景
img_back[y_offset:y_offset+h_r, x_offset:x_offset+w_r] = roi
cv2.imshow("Rotated Centered", img_back)







# 显示结果
# cv2.imshow("map", img)
# cv2.imshow('Rotated Image', rotated)


cv2.waitKey(0)
cv2.destroyAllWindows()
