import os

def change_label_index(folder_path, new_index=16, save_to=None):
    """
    修改 YOLO 标签文件的类别编号为 new_index。
    :param folder_path: 标签文件夹路径
    :param new_index: 新的类别编号（默认为 16）
    :param save_to: 若指定此参数，则将修改后的文件保存到新目录，否则覆盖原文件
    """
    os.makedirs(save_to, exist_ok=True) if save_to else None

    for filename in os.listdir(folder_path):
        if filename.endswith(".txt"):
            file_path = os.path.join(folder_path, filename)

            # 读取原文件内容
            with open(file_path, "r") as f:
                lines = f.readlines()

            new_lines = []
            for line in lines:
                parts = line.strip().split()
                if len(parts) >= 5:  # YOLO格式至少5个数
                    parts[0] = str(new_index)
                    new_lines.append(" ".join(parts) + "\n")

            # 保存修改后的文件
            out_path = os.path.join(save_to, filename) if save_to else file_path
            with open(out_path, "w") as f:
                f.writelines(new_lines)

            print(f"✅ 已修改: {filename}")

    print("🎉 全部修改完成！")

# ===== 示例用法 =====
if __name__ == "__main__":
    folder = r"D:\ros\26rc-yolo\yolo_zip\kfs_F_31.v2-kfs_f_31_1.yolov11\train\labels" # 改成你的标签文件夹路径
    change_label_index(folder, new_index=30)
