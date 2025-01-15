import cv2

from ultralytics import solutions

cap = cv2.VideoCapture(0)
assert cap.isOpened(), "Error reading video file"

# Video writer
w, h, fps = (int(cap.get(x)) for x in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS))
# video_writer = cv2.VideoWriter("security_alarm_output.avi", cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))

from_email = "abc@gmail.com"  # The sender email address
password = "---- ---- ---- ----"  # 16-digits password generated via: https://myaccount.google.com/apppasswords
to_email = "xyz@gmail.com"  # The receiver email address

# Init SecurityAlarm
security = solutions.SecurityAlarm(
    show=True,  # Display the output
    model="yolo11n.pt",  # i.e. YOLO11s.pt
    records=1,  # Total detections count to send an email about security
)

security.authenticate(from_email, password, to_email)  # Authenticate the email server

# Process video
while cap.isOpened():
    success, im0 = cap.read()
    if not success:
        print("Video frame is empty or video processing has been successfully completed.")
        break
    im0 = security.monitor(im0)
    # video_writer.write(im0)

cap.release()
# video_writer.release()
cv2.destroyAllWindows()