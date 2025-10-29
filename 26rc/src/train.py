# train_yolov11.py
from ultralytics import YOLO

# 初始化模型（可以使用官方权重 yolov11.pt 或自定义模型）
model = YOLO("yolov11.pt")  # 或 "yolov11n.pt" / "yolov11s.pt" 等

# 开始训练
model.train(
    data=r"D:\ros\26rc-yolo\yolov11_t_2\data.yaml",  # 只保留数据集信息的 YAML
    imgsz=640,                 # 输入图片大小
    batch=16,                  # 批次大小
    epochs=100,                # 训练轮数
    lr0=0.01,                  # 初始学习率
    lrf=0.1,                   # 最大学习率比例
    weight_decay=0.0005,       # 权重衰减
    momentum=0.937,            # 动量
    optimizer="SGD",           # 优化器，支持 "SGD" 或 "Adam"
    scheduler="cosine",        # 学习率调度策略
    project="runs/train",      # 保存结果目录
    name="yolov11_t_1", # 实验名称
    exist_ok=True              # 如果存在同名目录则覆盖
)

print("训练完成！")
