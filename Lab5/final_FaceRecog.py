# CEG 4166: Lab 4 - Final code for facial recognition.

import cv2
import numpy as np
import os
import threading
import time
from picamera2 import Picamera2

# Load the face detection model
cascadePath = os.path.join(os.getcwd(), 'haarcascade_frontalface_default.xml')
faceDetector = cv2.CascadeClassifier(cascadePath)

# Load trained face recognition model
recognizer = cv2.face.LBPHFaceRecognizer_create()
recognizer.read('model.yml')

# Font settings
font = cv2.FONT_HERSHEY_SIMPLEX

# Name data (modify as needed). The names should be in the order of input in
# faces_input and stored in the Dataset_Faces.
name_data = ['none', 'Veeren', 'Anthony']

def face_recognition(any1, any2, picam):
    while True:
        # Capture frame
        img = picam.capture_array()
        
        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        faces = faceDetector.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5)
        
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            id, confidence = recognizer.predict(gray[y:y + h, x:x + w])
            
            # Check confidence level (lower is better)
            if confidence < 100:
                id = name_data[id] if id < len(name_data) else "Unknown"
                confidence_text = " {0}%".format(round(100 - confidence))
            else:
                id = "Unknown"
                confidence_text = " {0}%".format(round(100 - confidence))
            
            cv2.putText(img, str(id), (x + 5, y - 5), font, 1, (255, 255, 255), 2)
            cv2.putText(img, str(confidence_text), (x + 5, y + h - 5), font, 1, (255, 255, 0), 1)
        
        # Display the frame
        cv2.imshow('Stingray Face Detector', img)
        
        # Exit on pressing 'ESC'
        if cv2.waitKey(10) & 0xFF == 27:
            break
    
    print("\nExiting the program")
    cv2.destroyAllWindows()
    picam.stop()

def run_face_recognition():
    # Initialize Picamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(config)
    picam2.start()

    # Allow camera to warm up
    time.sleep(2)
    # Start face recognition in a separate thread
    faceRecognitionThread = threading.Thread(target=face_recognition, args=('anything1', 'anything2', picam2))
    faceRecognitionThread.start()


if __name__ == "__main__":  # Only runs when executed directly, not on import
    run_face_recognition()