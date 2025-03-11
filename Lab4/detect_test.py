# CEG 4166: Lab 4 - Face detection test.

import numpy as np
import cv2
import threading
import os
from picamera2 import Picamera2
import time

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

def face_detection_test(anything1, anything2):
    while True:
        # Capture frame as NumPy array
        img = picam2.capture_array()
        
        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # The function detectMultiScale takes in 4 parameters as arguments:
        # gray - Defines the input grayscale image passed.
        # scaleFactor - Specifies how much image size is reduced
        # (https://docs.opencv.org/2.4/doc/tutorials/imgproc/pyramids/pyramids.html).
        # minNeighbors – This parameter specifies how many neighbors each rectangle will have.
        # minSize – It is the minimum size of the rectangle which can be considered as a face.
        faces = faceDetector.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5, minSize=(20, 20))
        
        # If faces are found, the positions of all the detected faces are
        # bound by a rectangle with (x,y) set as top left corner, w as the
        # width and h as the height.
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
        
        cv2.imshow('Face Detection', img)
        k = cv2.waitKey(30) & 0xFF
        
        # Press 'ESC' key to exit
        if k == 27:
            break
    
    cv2.destroyAllWindows()
    picam2.stop()

# Start face detection in a separate thread
detectionThread = threading.Thread(target=face_detection_test, args=('any1', 'any2'))
detectionThread.start()
