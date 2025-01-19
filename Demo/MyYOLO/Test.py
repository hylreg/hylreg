
from ultralytics import YOLO

# 加载模型
# model = YOLO("model/yolo11.yaml")
model = YOLO("yolo11n.pt")
# model = YOLO("model/yolo11.yaml").load("yolo11n.pt")


# 训练模型
train_results = model.train(
    data="dataset/coco8.yaml",  # 数据集 YAML 路径
    epochs=100,  # 训练轮次
    imgsz=640,  # 训练图像尺寸
    device="0",  # 运行设备，例如 device=0 或 device=0,1,2,3 或 device=cpu
    batch=32,
)

# 评估模型在验证集上的性能
metrics = model.val()

# 在图像上执行对象检测
results = model("bus.jpg")
results[0].show()

# 将模型导出为 ONNX 格式
# path = model.export(format="onnx")  # 返回导出模型的路径
# print(path)


# search_space = {
#     "lr0": (1e-5, 1e-1),
#     "degrees": (0.0, 45.0),
# }
#
# # Tune hyperparameters on COCO8 for 30 epochs
# model.tune(
#     data="dataset/coco8.yaml",
#     epochs=30,
#     iterations=300,
#     optimizer="AdamW",
#     space=search_space,
#     plots=False,
#     save=False,
#     val=False,
# )