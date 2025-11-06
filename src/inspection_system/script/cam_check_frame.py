#!/usr/bin/env python3

import cv2

# Crop settings
crop_width_percent = 0.3   # 30% from left and right
crop_height_percent = 0.15 # 15% from top and bottom

# Open webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Failed to open webcam.")
    exit()

print("Press 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    (H_full, W_full) = frame.shape[:2]

    # Calculate crop coordinates
    x_start = int(W_full * crop_width_percent)
    x_end = int(W_full * (1 - crop_width_percent))
    y_start = int(H_full * crop_height_percent)
    y_end = int(H_full * (1 - crop_height_percent))

    # Crop frame (centered)
    cropped_frame = frame[y_start:y_end, x_start:x_end]

    # Display the cropped frame
    cv2.imshow("Cropped Webcam View", cropped_frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
