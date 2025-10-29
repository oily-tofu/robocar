import cv2
import numpy as np
import random

# 读取图片
img = cv2.imread('jx.jpg')
if img is None:
    raise ValueError("图片路径错误或文件不存在")

# 图片尺寸
h, w = img.shape[:2]

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

# 显示结果
cv2.imshow('Warped Image', warped)
cv2.waitKey(0)
cv2.destroyAllWindows()
