import cv2

from ultralytics import solutions

cap = cv2.VideoCapture(0)
assert cap.isOpened(), "Error reading video file"
w, h, fps = (int(cap.get(x)) for x in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS))

# Define region points
region_points = [(150, 150), (1130, 150), (1130, 570), (150, 570)]

# Video writer
# video_writer = cv2.VideoWriter("object_counting_output.avi", cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))

# Init TrackZone (Object Tracking in Zones, not complete frame)
trackzone = solutions.TrackZone(
    show=True,  # Display the output
    region=region_points,  # Pass region points
    model="yolo11n.pt",  # You can use any model that Ultralytics support, i.e. YOLOv9, YOLOv10
    # line_width=2,  # Adjust the line width for bounding boxes and text display
    # classes=[0, 2],  # If you want to count specific classes i.e. person and car with COCO pretrained model.
)

# Process video
while cap.isOpened():
    success, im0 = cap.read()
    if not success:
        print("Video frame is empty or video processing has been successfully completed.")
        break
    im0 = trackzone.trackzone(im0)
    # video_writer.write(im0)

cap.release()
# video_writer.release()
cv2.destroyAllWindows()