import cv2
import numpy as np
from definizioni import *
from ultralytics import YOLO

model=None # Global variable for the YOLO model

def modello(strmodello):
    global model
    model = YOLO(strmodello) # Load the YOLOv8 model

def visualizza_yolo(frame, target_class):
    # Run the YOLO model on the input frame
    results = model(frame)
    max_area = 0 # To track the largest detected object area
    center = None # To store the center of the largest detected object

    # Loop over all detection results
    for result in results:
        boxes = result.boxes # Get detected bounding boxes
        for box in boxes:
            # Extract bounding box coordinates
            x1, y1, x2, y2 = box.xyxy[0]
            conf = box.conf[0] # Confidence score
            cls = box.cls[0] # Class ID
            label = model.names[int(cls)] # Convert class ID to label

            # Check if the detected label matches the desired class
            if label == target_class:
                area = (x2 - x1) * (y2 - y1) # Calculate the area of the bounding box

                # Se l'area è maggiore della massima trovata finora, aggiorna il centro
                if area > max_area:
                    max_area = area
                    center = (int((x1 + x2) / 2), int((y1 + y2) / 2))

                # Draw bounding box and label on the frame
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f'{label}: {conf:.2f}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.imshow('immagine', frame)
                cv2.waitKey(1)

    return center # Return the center of the largest matching object (or None)

# Function to detect the largest colored area in an image
def find_largest_green_area(color_image,lower_col,upper_col):
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV) # Convert image from BGR to HSV color space

    mask = cv2.inRange(hsv_image, lower_col, upper_col) # Create a binary mask where the target color is white

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, None

    # Find the contour with the largest area
    largest_contour = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest_contour)

    if M["m00"] > 0:
        # Calculate the center
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        return (cx, cy)
    return None, None
