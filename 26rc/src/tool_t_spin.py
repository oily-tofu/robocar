import cv2
import random
import numpy as np

# 读取图片
img = cv2.imread('jx.jpg')
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
warped = cv2.warpPerspective(img, M, (w, h))


# 设置旋转角度（顺时针为正，逆时针为负）
angle = random.randint(0, 360)

# 设置缩放比例（1.0表示不缩放）
scale = 0.5

# 获取旋转矩阵
M = cv2.getRotationMatrix2D(center, angle, scale)

# 应用旋转变换
rotated = cv2.warpAffine(warped, M, (w, h))

# 显示结果
cv2.imshow('Rotated Image', rotated)
cv2.waitKey(0)
cv2.destroyAllWindows()
