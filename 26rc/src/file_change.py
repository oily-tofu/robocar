import os
import random
import shutil

def split_dataset(base_dir, output_dir, train_ratio=0.8, val_ratio=0.1, test_ratio=0.1):
    # 原始路径
    img_dir = os.path.join(base_dir, "images")
    label_dir = os.path.join(base_dir, "labels")

    # 新的数据集结构
    for subset in ["train", "val", "test"]:
        for subfolder in ["images", "labels"]:
            os.makedirs(os.path.join(output_dir, subset, subfolder), exist_ok=True)

    # 获取所有图片文件（jpg 或 png）
    img_files = [f for f in os.listdir(img_dir) if f.lower().endswith((".jpg", ".png"))]
    random.shuffle(img_files)

    total = len(img_files)
    train_end = int(total * train_ratio)
    val_end = int(total * (train_ratio + val_ratio))

    # 划分
    train_files = img_files[:train_end]
    val_files = img_files[train_end:val_end]
    test_files = img_files[val_end:]

    def copy_files(file_list, subset):
        for img_name in file_list:
            name, _ = os.path.splitext(img_name)
            img_path = os.path.join(img_dir, img_name)
            label_path = os.path.join(label_dir, f"{name}.txt")

            # 复制图片
            shutil.copy(img_path, os.path.join(output_dir, subset, "images", img_name))
            # 若标签存在则复制
            if os.path.exists(label_path):
                shutil.copy(label_path, os.path.join(output_dir, subset, "labels", f"{name}.txt"))

    # 执行复制
    copy_files(train_files, "train")
    copy_files(val_files, "val")
    copy_files(test_files, "test")

    print(f"✅ 数据集划分完成！共 {total} 张图片：")
    print(f" - 训练集：{len(train_files)} 张")
    print(f" - 验证集：{len(val_files)} 张")
    print(f" - 测试集：{len(test_files)} 张")
    print(f"已保存到：{output_dir}")

if __name__ == "__main__":
    # 你的原始路径
    base_dir = r"D:\ros\26rc-yolo\yolov11_t_1\train"  # 修改为你的train所在路径
    output_dir = r"D:\ros\26rc-yolo\yolov11_t_2"  # 输出新路径
    split_dataset(base_dir, output_dir)
