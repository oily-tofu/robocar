import cv2
import numpy as np
import random
import os
from PIL import Image

# ========== 取背景图 ==========
def back_open(folder_path):
    img_list = [f for f in os.listdir(folder_path)
                if f.lower().endswith(('.jpg', '.png', '.jpeg', '.bmp'))]
    if not img_list:
        raise FileNotFoundError("文件夹中没有图片")
    return folder_path, img_list


# ========== 生成透明通道 ==========
def alpha_test(img):
    h, w = img.shape[:2]
    img_rgba = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
    threshold = 10
    alpha = np.ones((h, w), dtype=np.uint8) * 255

    border = 10
    mask_black = (
        (img[:, :, 0] < threshold) &
        (img[:, :, 1] < threshold) &
        (img[:, :, 2] < threshold)
    )
    mask_black[border:-border, border:-border] = False
    alpha[mask_black] = 0
    img_rgba[:, :, 3] = alpha
    return img_rgba


# ========== 随机透视 + 旋转 ==========
def other_test(img_rgba):
    h, w = img_rgba.shape[:2]
    center = (w // 2, h // 2)

    pts1 = np.float32([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]])
    t1, t2 = random.uniform(0, 0.3), random.uniform(0.7, 1)
    t3, t4 = random.uniform(0, 0.3), random.uniform(0.7, 1)
    t5, t6 = random.uniform(0.9, 1), random.uniform(0.9, 1)
    pts2 = np.float32([[w * t1, 0], [w * t2, 0], [w * t4, h * t6], [w * t3, h * t5]])
    M_persp = cv2.getPerspectiveTransform(pts1, pts2)
    warped = cv2.warpPerspective(img_rgba, M_persp, (w, h))

    angle = random.randint(0, 360)
    scale = random.uniform(0.1, 0.4)
    M_rot = cv2.getRotationMatrix2D(center, angle, scale)
    rotated = cv2.warpAffine(warped, M_rot, (w, h))
    return rotated


# ========== 图像叠加 ==========
def blend_image(img_back, rotated):
    h_r, w_r = rotated.shape[:2]
    h_i, w_i = img_back.shape[:2]
    x_offset = (w_i - w_r) // 2
    y_offset = (h_i - h_r) // 2

    b, g, r, a = cv2.split(rotated)
    alpha = a / 255.0
    roi = img_back[y_offset:y_offset + h_r, x_offset:x_offset + w_r]

    for c, channel in enumerate([b, g, r]):
        roi[:, :, c] = np.where(alpha > 0, channel, roi[:, :, c])
    roi[:, :, 3] = np.maximum(roi[:, :, 3], a)
    img_back[y_offset:y_offset + h_r, x_offset:x_offset + w_r] = roi

    return img_back


# ========== 增强 ==========
def random_strong(image):
    r = random.random()
    # 随机亮度和对比度
    if r < 0.3:
        alpha = random.uniform(0.8, 1.2)  # 对比度
        beta = random.uniform(-20, 20)    # 亮度
        image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

    # 随机模糊
    if 0.3 <= r < 0.5:
        k = random.choice([3, 5])
        image = cv2.GaussianBlur(image, (k, k), 0)

    # 转灰度图
    if 0.5 <= r < 0.6:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # 灰度图转回三通道，保持尺寸一致
        image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    return image


# ========== 主流程 ==========
if __name__ == "__main__":
    input_dir = r"D:\ros\26rc-yolo\kfs_1"
    output_dir = r"D:/ros/26rc-yolo/CQUPT-1/F_30"    #输出保存
    folder_path = r"D:\ros\26rc-yolo\backgrond_ing"   #背景文件夹
    os.makedirs(output_dir, exist_ok=True)

    folder_path, img_list = back_open(folder_path)

    num = 3263
    for img_name in img_list:
        img_path = os.path.join(folder_path, img_name)
        img_b = np.array(Image.open(img_path))
        img_back = cv2.cvtColor(img_b, cv2.COLOR_RGB2BGRA)

        for filename in os.listdir(input_dir):
            for _ in range(5):
                num += 1
                if not filename.lower().endswith(('.png', '.jpg', '.jpeg')):
                    continue

                img_path = os.path.join(input_dir, filename)
                img = cv2.imread(img_path)
                if img is None:
                    print(f"无法读取图片 {filename}")
                    continue

                img_rgba = alpha_test(img)
                rotated = other_test(img_rgba)
                result = blend_image(img_back.copy(), rotated)
                result = random_strong(result)

                save_path = os.path.join(output_dir, f"CQUPT_{num:05d}.jpg")
                cv2.imwrite(save_path, result)
                print(f"已保存: {num},{save_path}")





    cv2.destroyAllWindows()



# .=%@#=.
#                                             -*@@@@@@@#=.
#                                          .+%@@@@@@@@@@@@#=
#                                        -#@@@@@@@* =@@@@@@@@*:
#                                      =%@@@@@@@@=   -@@@@@@@@@#-
#                                   .+@@@@@@@@@@-     .@@@@@@@@@@%=
#                                 .+@@@@@@@@@@@@-     +@@@@@@@@@@@@@+.
#                                +@@@@@@@@@@@@@@@    .@@@@@@@@@@@@@@@@+.
#                              =@@@@@@@@@@@@@@@%-     =%@@%@@@@@@@@@@@@@=
#                            -%@@@@@@@@@@@@+..     .       -@@@@@@@@@@@@@%-
#                          .#@@@@@@@@@@@@@#       -@+       +@@@@@@@@@@@@@@#:
#                         +@@@@@@@@@@@@@@@@+     +@@@+     =@@@@@@@@@@@@@@@@@+
#                       :%@@@@@@@@@@@@@@@@@+    *@@@@*     =@@@@@@@@@@@@@@@@@@%-
#                      +@@@@@@@@@@@@@@#+*+-   .#@@@@+       :+*+*@@@@@@@@@@@@@@@*
#                    :%@@@@@@@@@@@@@@+       :%@@@@-    .-       -@@@@@@@@@@@@@@@%:
#                   =@@@@@@@@@@@@@@@@-      -@@@@%:    .%@+      =@@@@@@@@@@@@@@@@@=
#                  *@@@@@@@@@@@@@@@@@@.    =@@@@#.    -@@@@+    =@@@@@@@@@@@@@@@@@@@#
#                .%@@@@@@@@@@@@@@@@@@+    +@@@@*     =@@@@%:    .#@@@@@@@@@@@@@@@@@@@%.
#               :@@@@@@@@@@@@@@@%:::.    #@@@@+     +@@@@#        .::.*@@@@@@@@@@@@@@@@-
#              -@@@@@@@@@@@@@@@%       .%@@@@=     *@@@@*     +-       *@@@@@@@@@@@@@@@@=
#             =@@@@@@@@@@@@@@@@@#.    -@@@@@-    :%@@@@=    .#@@+     +@@@@@@@@@@@@@@@@@@=
#            =@@@@@@@@@@@@@@@@@@@:    =====.     -+===:     :====     @@@@@@@@@@@@@@@@@@@@+
#           +@@@@@@@@@@@@@@@#%%#-                                     :*%%#%@@@@@@@@@@@@@@@+
#          =@@@@@@@@@@@@@@%.       ...........................              *@@@@@@@@@@@@@@@=
#         =@@@@@@@@@@@@@@@+      .#@@@@@@@@@@@@@@@@@@@@@@@@@@#     .*:      =@@@@@@@@@@@@@@@@-
#        -@@@@@@@@@@@@@@@@@=    .%@@@@@@@@@@@@@@@@@@@@@@@@@@#     :@@@-    =@@@@@@@@@@@@@@@@@@:
#       :@@@@@@@@@@@@@@@@@%.   -@@@@%+=====================:     -@@@@%    :%@@@@@@@@@@@@@@@@@@.
#       %@@@@@@@@@@@@@=-=:    =@@@@#.                           +@@@@#.      -=--%@@@@@@@@@@@@@%
#      #@@@@@@@@@@@@@:       +@@@@*      ............. .       *@@@@*             %@@@@@@@@@@@@@+
#     =@@@@@@@@@@@@@@#.     #@@@@+     +@@@@@@@@@@@@@@@#.    .#@@@@+     +#.     +@@@@@@@@@@@@@@@:
#    .@@@@@@@@@@@@@@@@-   .%@@@@=     *@@@@@@@@@@@@@@@#     :%@@@@-     *@@%:    @@@@@@@@@@@@@@@@%
#    %@@@@@@@@@@@%%%#=   :@@@@@:    .#@@@@+-----------     -@@@@@:     #@@@@=    :#%%%@@@@@@@@@@@@*
#   =@@@@@@@@@@@=       -@@@@%.    :%@@@@-                =@@@@%.    .%@@@@=          :%@@@@@@@@@@@:
#   @@@@@@@@@@@%.      =@@@@#     -@@@@%:    .:::-:      +@@@@#     :@@@@@:    .       +@@@@@@@@@@@#
#  +@@@@@@@@@@@@@.    *@@@@*     =@@@@#.    -@@@@@:     #@@@@+     =@@@@%.    -@#     +@@@@@@@@@@@@@-
# .@@@@@@@@@@@@@#    *@%@%=     +@@@@*     =@@@@#.    .#@@@%=     +@@@@#     =@@@%.   =@@@@@@@@@@@@@%
# +@@@@@@@@*-==-                .          .           . ..       .....      .....     .=+=+@@@@@@@@@-
# %@@@@@@@+                                                                                 -@@@@@@@@#
# @@@@@@@-       =#%#=     -#%%#-     -#%%*.     +%%%*.    .*%%#=     :#%%#-     =%%%*.      .#@@@@@@@
# @@@@@@=.::::::*@@@@@*:::-@@@@@@-:::=@@@@@%::::*@@@@@#::::%@@@@@+:---@@@@@@=---+@@@@@%------:=@@@@@@@
# =@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@+
#  *@@@@@@@@@@@@@@@@@@@@@@@@@@@%%##**++===----:::::------===++***##%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*
#   -#@@@@@@@@@@@@@@@@%#*+=-:.                                        ..::-=+*##%@@@@@@@@@@@@@@@@@#-
#     :=*%@@@@@%#*=-:                                                             .:-=+*#%%%%##+-.

#         K####      #####     ###    ###  ######.   ##########     K##    ### ###    ##W    ####W
#        #######    #######    ###    ###  ########  ##########     ###    ### ###   ###   W######
#       W###G####  ###W ####   ###    ###  ######### ##########     ###    ###  ##   ###   ###W####
#       ###   ###  ###   ###   ###    ##  ###    ###    ###         ###    ###  ### t##   ###   ###
#      G##    #   ###    ###   ##     ##  ###    ###    ###         ###    ###  ### ###   ##W
#      ###        ###    ###   ##    ###  ###    ###    ###         ##L    ##   ### ##   ###
#      ###        ###    ###  K##    ###  ###    ###    ###         ##     ##    #####   ###
#      ###       ,##     ###  ###    ###  ###   ###,    ##         G##    ###    ####    ###
#     W##        ###     ###  ###    ###  #########     ##         ##########    ####    ###
#     ###        ###     ###  ###    ###  ########     ###         ##########    ###i   K##
#     ###        ###     ###  ###    ##  #######       ###         ###    ###    ####   ###
#     ###        ###     ###  ##     ##  ###           ###         ###    ###   ##W##   ###
#     ###        ###     ##i  ##    ###  ###           ###         ###    ##    ## ##   ###
#     ###        ###    ###  ,##    ###  ###           ###         ##     ##   ### ##   ###
#     ###    ### ###    ###  K##    ###  ###           ##         t##    ###   ##  ###  ###    ###
#     ###   G##i ###   ###   .##   ###.  ##t           ##         ###    ###  ###  ###  W##,   ###
#      ########  W##W#####    ########   ##           ###         ###    ###  ##    ##   ####W###
#      #######    #######     #######   ###           ###         ###    ### ###    ##.  #######
#       #####      #####       #####    ###           ###         ###    ### ##W    ###   #####
#                    ###
#                    ###
#                    #####
#                     ####
#                       K
