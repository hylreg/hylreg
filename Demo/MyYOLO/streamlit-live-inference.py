from ultralytics import solutions

inf = solutions.Inference(
    model="yolo11n.pt",  # You can use any model that Ultralytics support, i.e. YOLO11, or custom trained model
)

inf.inference()

### Make sure to run the file using command `streamlit run <file-name.py>`