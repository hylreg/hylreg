from ultralytics import YOLO

weights_path = "yolo11n.pt"
# model = YOLO(weights_path, task="detect")


results = {}

# Define your additional arguments here
batch = 16
project = "kfold_demo"
epochs = 10


dataset_yaml = "../dataset/coco8.yaml"
model = YOLO(weights_path)
model.train(data=dataset_yaml, epochs=epochs, batch=batch)  # include any train arguments
model.metrics  # save output metrics for further analysis