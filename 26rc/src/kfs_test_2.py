import cv2
import numpy as np
import random
import os
from PIL import Image

# cqupt很不高兴为您服务@_@

def random_open(folder_path):
    # 获取文件夹中所有图片文件
    img_list = [f for f in os.listdir(folder_path)
                if f.lower().endswith(('.jpg', '.png', '.jpeg', '.bmp'))]

    if not img_list:
        raise FileNotFoundError("文件夹中没有图片")

    # 随机打乱图片列表
    return folder_path, img_list



# kfs原图处理
def alpha_test(h,w):

        img_rgba = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
    img_rgba[:, :, 3] = 255

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

def other_test(img_rgba):

    center = (w // 2, h // 2)

    # 旋转透视变换
    pts1 = np.float32([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]])
    t1, t2 = random.uniform(0, 0.3), random.uniform(0.7, 1)
    t3, t4 = random.uniform(0, 0.3), random.uniform(0.7, 1)
    t5, t6 = random.uniform(0.9, 1), random.uniform(0.9, 1)
    pts2 = np.float32([[w * t1, 0], [w * t2, 0], [w * t4, h * t6], [w * t3, h * t5]])
    M_persp = cv2.getPerspectiveTransform(pts1, pts2)
    warped = cv2.warpPerspective(img_rgba, M_persp, (w, h))

    angle = random.randint(0, 360)
    scale = 0.5
    M_rot = cv2.getRotationMatrix2D(center, angle, scale)
    rotated = cv2.warpAffine(warped, M_rot, (w, h))
    return rotated

def super(rotated):
    # 图片叠加
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


    cv2.imshow("Rotated Centered", img_back)

    return img_back

if __name__ == "__main__":

    input_dir = r"D:\ros\26rc\src"  # 图片输入文件夹
    output_dir = r"D:/ros/26rc-yolo/tool_kfs/"  # 保存输出的文件夹
    folder_path = r"D:\ros\26rc-yolo\backgrong_ing"  #背景文件夹
    os.makedirs(output_dir, exist_ok=True)
    folder_path, img_list = random_open(folder_path)
    for img_name in img_list:
        img_path = os.path.join(folder_path, img_name)
        img_b = Image.open(img_path)
        img_b = np.array(img_b)
        img_back = cv2.cvtColor(img_b, cv2.COLOR_BGR2BGRA)

        num = 722

        for _ in range(5):
            num += 1

            for filename in os.listdir(input_dir):
                if not filename.lower().endswith(('.png', '.jpg', '.jpeg')):
                    continue

                img_path = os.path.join(input_dir, filename)

                img = cv2.imread(img_path)
                if img is None:
                    print(f"无法读取图片 {filename}")
                    continue

                rows, cols = 4, 4
                block_size = 640 // rows
                h, w = img.shape[:2]

                img_rgba = alpha_test(h,w)
                rotated = other_test(img_rgba)
                result = super(rotated)

                save_path = os.path.join(output_dir, f"CQUPT_{num:05d}.jpg")
                cv2.imwrite(save_path, result)
                print(f"已保存: {save_path}")

    cv2.waitKey(0)
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
