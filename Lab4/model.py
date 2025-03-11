# CEG 4166: Lab 4 - Face detection training model.

import cv2
import numpy as np
from PIL import Image
import os
import time
from picamera2 import Picamera2

# Initialize Picamera2
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

# Allow camera to warm up
time.sleep(2)

# Path for the face image database
path = 'Dataset_Faces'

# Load the face detection model
cascadePath = os.path.join(os.getcwd(), 'haarcascade_frontalface_default.xml')
faceDetector = cv2.CascadeClassifier(cascadePath)

# Initialize face recognizer
recognizer = cv2.face.LBPHFaceRecognizer_create()

def training_function(path):
    images_dataset = [os.path.join(path, char) for char in os.listdir(path) if char.endswith('.jpg')]
    number_faces = []
    tags = []
    
    for image_path in images_dataset:
        # Convert to grayscale
        gray = Image.open(image_path).convert('L')
        img_128D = np.array(gray, 'uint8')
        
        # Extract the face ID from filename
        face_id = int(os.path.split(image_path)[-1].split(".")[1])
        faces = faceDetector.detectMultiScale(img_128D)
        
        for (x, y, w, h) in faces:
            number_faces.append(img_128D[y:y + h, x:x + w])
            tags.append(face_id)
    
    return number_faces, tags

faces, tags = training_function(path)

# The recognizer train function trains the FaceRecognizer with the given data
# (https://docs.opencv.org/4.0.1/dd/d65/classcv_1_1face_1_1FaceRecognizer.html)
if len(faces) > 0:
    recognizer.train(faces, np.array(tags))
    # Save the trained model
    recognizer.write('model.yml')
    print("\n {0} Faces training done".format(len(np.unique(tags))))
else:
    print("\n No faces found for training.")

# Cleanup
picam2.stop()
