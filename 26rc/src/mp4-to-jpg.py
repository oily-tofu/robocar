import cv2
import os
import random


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

def mp4_to_jpg(video_path, output_dir):
    # 检查视频文件是否存在
    if not os.path.exists(video_path):
        print(f"文件不存在: {video_path}")
        return

    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)

    # 打开视频
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("无法打开视频文件")
        return

    frame_count = 0
    num = 0
    success, frame = cap.read()

    while success:
        frame_count += 1
        # 生成保存路径
        if frame_count % 15 == 1 or frame_count % 15 == 7:
            num += 1
            frame = random_strong(frame)
            save_path = os.path.join(output_dir, f"CQUPT_{num:05d}.jpg")
            # 保存为 JPG 图片
            cv2.imwrite(save_path, frame)

            # 打印进度
            if num % 50 == 0:
                print(f"已保存 {num} 帧...")

        # 读取下一帧
        success, frame = cap.read()



    cap.release()
    print(f"完成 共导出 {num} 帧到 {output_dir}")

if __name__ == "__main__":

    video_path = "D:/ros/26rc-yolo/yolo-photo/CQUPT-1.mp4"
    output_dir = "D:/ros/26rc-yolo/yolo-photo"
    mp4_to_jpg(video_path, output_dir)
