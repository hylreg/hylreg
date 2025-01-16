import cv2

from ultralytics import solutions

cap = cv2.VideoCapture(0)
assert cap.isOpened(), "Error reading video file"
w, h, fps = (int(cap.get(x)) for x in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS))

# # Video writer
# out = cv2.VideoWriter(
#     "ultralytics_analytics.avi",
#     cv2.VideoWriter_fourcc(*"MJPG"),
#     fps,
#     (1920, 1080),  # This is fixed
# )

# Init analytics
analytics = solutions.Analytics(
    show=True,  # Display the output
    analytics_type="line",  # Pass the analytics type, could be "pie", "bar" or "area".
    model="yolo11n.pt",  # Path to the YOLO11 model file
    # classes=[0, 2],  # If you want to count specific classes i.e person and car with COCO pretrained model.
)

# Process video
frame_count = 0
while cap.isOpened():
    success, im0 = cap.read()
    if success:
        frame_count += 1
        im0 = analytics.process_data(im0, frame_count)  # update analytics graph every frame
        # out.write(im0)  # write the video file
    else:
        break

cap.release()
# out.release()
cv2.destroyAllWindows()