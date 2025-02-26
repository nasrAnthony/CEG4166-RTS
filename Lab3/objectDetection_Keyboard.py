## Object detection in Stingray using TensorFlow Lite
## Save the file as object_detection.py
## Lab 3

import os
import argparse
import cv2
import numpy as np
import sys
import time
import threading
import importlib.util
from threading import Thread

class Video_PiCamera:
    def __init__(self, resolution=(640, 480), framerate=60):
        # Initializing the PiCamera of the Stingray
        self.stream = cv2.VideoCapture(0)
        self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.stream.set(3, resolution[0])
        self.stream.set(4, resolution[1])
        (self.grabbed, self.frame) = self.stream.read()  # Reading the initial frame from the video stream
        self.stopped = False

    def start(self):
        # Starting the thread to read from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                self.stream.release()  # Stop the thread when camera is stopped
                return
            (self.grabbed, self.frame) = self.stream.read()  # Read the next frame if the camera is running

    def read(self):
        # Return the most recent frame captured from the PiCamera
        return self.frame

    def stop(self):
        # Stops the thread and camera
        self.stopped = True

# Parsing the arguments for the TensorFlow Lite input
parser = argparse.ArgumentParser()
parser.add_argument('--modeldir', required=True)
parser.add_argument('--graph', default='detect.tflite')
parser.add_argument('--labels', default='labelmap.txt')
parser.add_argument('--threshold', default=0.5)
parser.add_argument('--resolution', default='600x300')
args = parser.parse_args()

# Assigning argument values to variables
model = args.modeldir
graph_n = args.graph
label_ = args.labels
minimum_confidence = float(args.threshold)
resW, resH = args.resolution.split('x')
imW, imH = int(resW), int(resH)

# Importing the TensorFlow Lite libraries
pkg = importlib.util.find_spec('tflite_runtime')
if pkg:
    from tflite_runtime.interpreter import Interpreter
else:
    from tensorflow.lite.python.interpreter import Interpreter

# Getting the paths to model files
current_dir = os.getcwd()
tflite_directory = os.path.join(current_dir, model, graph_n)
label_destination = os.path.join(current_dir, model, label_)

# Load the label map
with open(label_destination, 'r') as f:
    labels = [line.strip() for line in f.readlines()]
    if labels[0] == '???':  # First label is always '???', remove it to avoid errors
        del labels[0]

# Load the TensorFlow Lite model
model_interpreter = Interpreter(model_path=tflite_directory)
model_interpreter.allocate_tensors()
input = model_interpreter.get_input_details()
output = model_interpreter.get_output_details()
height = input[0]['shape'][1]
width = input[0]['shape'][2]
floating_model = (input[0]['dtype'] == np.float32)
input_mean = 127.5
input_std = 127.5

# Initializing the video streaming
Video_PiCamera = Video_PiCamera(resolution=(imW, imH), framerate=60).start()
time.sleep(1)

def detection(any, any2):
    while True:
        # Grab frame from video stream
        original_frame = Video_PiCamera.read()
        
        # Duplicating the frame and adjusting the size
        frame = original_frame.copy()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (width, height))
        input_data = np.expand_dims(frame_resized, axis=0)

        # For a non-quantized model, normalize the pixels
        if floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std
        
        model_interpreter.set_tensor(input[0]['index'], input_data)
        model_interpreter.invoke()

        # Retrieve detection results
        box = model_interpreter.get_tensor(output[0]['index'])[0]  # Bounding box coordinates
        classes = model_interpreter.get_tensor(output[1]['index'])[0]  # Class index of objects
        conf_value = model_interpreter.get_tensor(output[2]['index'])[0]  # Confidence of detected objects

        for i in range(len(conf_value)):
            # Comparing with the minimum threshold
            if minimum_confidence < conf_value[i] <= 1.0:
                # Get bounding box coordinates and draw box
                ymin = int(max(1, (box[i][0] * imH)))
                xmin = int(max(1, (box[i][1] * imW)))
                ymax = int(min(imH, (box[i][2] * imH)))
                xmax = int(min(imW, (box[i][3] * imW)))

                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 2)
                
                # Draw label around the box
                object_name = labels[int(classes[i])]
                label = f'{object_name}: {int(conf_value[i] * 100)}%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                label_ymin = max(ymin, labelSize[1] + 10)
                cv2.rectangle(frame, (xmin, label_ymin - labelSize[1] - 10),
                              (xmin + labelSize[0], label_ymin + baseLine - 10), (255, 255, 255), cv2.FILLED)
                cv2.putText(frame, label, (xmin, label_ymin - 7),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        
        cv2.imshow('Object Detection in Stingray', frame)

        # Press 'q' to quit
        if cv2.waitKey(1) == ord('q'):
            print("\nExiting the frame")
            break

    cv2.destroyAllWindows()
    Video_PiCamera.stop()

# Creating a thread for object detection
objectDetectionThread = threading.Thread(target=detection, args=('any1', 'any2'))
objectDetectionThread.start()
