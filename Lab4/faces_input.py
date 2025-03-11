# CEG 4166: Lab 4 - Get input face data.

import cv2
import os
import time
from picamera2 import Picamera2

# Initialize Picamera2
picam2 = Picamera2()

# Configure the height and width of the frame.
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

# Allow camera to warm up
time.sleep(2)

# Load the face detection model
cascadePath = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
faceDetector = cv2.CascadeClassifier(cascadePath)

# For each person, enter one numeric face id - start from 1
faceId = input("\nEnter Face ID (S.No): ")
print("\nLook at the camera sensor.")
count = 0

# Ensure dataset directory exists
dataset_path = "Dataset_Faces"
os.makedirs(dataset_path, exist_ok=True)

while True:
    # Capture frame
    img = picam2.capture_array()
    
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    faces = faceDetector.detectMultiScale(gray, 1.3, 5)
    
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
        count += 1
        
        # Save the captured image into the datasets folder
        face_filename = os.path.join(dataset_path, f"Tag.{faceId}.{count}.jpg")
        cv2.imwrite(face_filename, gray[y:y + h, x:x + w])
    
    # Display the current frame captured
    cv2.imshow("Face Capture", img)
    
    # Exit on pressing 'ESC' or after capturing 50 images for each user (unique face/ID)
    k = cv2.waitKey(100) & 0xFF
    if k == 27 or count >= 50:
        break

print("\nCapture complete. Exiting.")
cv2.destroyAllWindows()
picam2.stop()