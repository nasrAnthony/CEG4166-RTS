import os
import argparse
import cv2
import numpy as np
import sys
import time
import threading
import importlib.util
from threading import Thread
from picamera2 import Picamera2

class Video_PiCamera:
    def __init__(self, resolution=(640, 480), framerate=60):
        # Initialize Picamera2
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(main={"size": resolution, "format": "RGB888"}))
        self.picam2.start()
        self.frame = None
        self.stopped = False

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while not self.stopped:
            self.frame = self.picam2.capture_array()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.picam2.stop()

def parse_arguments():
    """ Parse command-line arguments if running standalone. """
    parser = argparse.ArgumentParser()
    parser.add_argument('--modeldir', required=True)
    parser.add_argument('--graph', default='detect.tflite')
    parser.add_argument('--labels', default='labelmap.txt')
    parser.add_argument('--threshold', type=float, default=0.5)
    parser.add_argument('--resolution', default='600x300')
    
    return parser.parse_args()
def load_model(args):
    # Assigning argument values to variables
    global labels, model_interpreter, input, output, width, height, floating_model, input_mean, input_std, imW, imH, minimum_confidence

    model = args.modeldir
    graph_n = args.graph
    label_ = args.labels
    minimum_confidence = float(args.threshold)
    resW, resH = args.resolution.split('x')
    imW, imH = int(resW), int(resH)

    # Import TensorFlow Lite
    pkg = importlib.util.find_spec('tflite_runtime')
    if pkg:
        from tflite_runtime.interpreter import Interpreter
    else:
        from tensorflow.lite.python.interpreter import Interpreter

    # Load model files
    tflite_directory = "/home/ceg4166/Desktop/CEG4166/Lab3/ssd_model/detect.tflite"
    label_destination = "/home/ceg4166/Desktop/CEG4166/Lab3/ssd_model/labelmap.txt"

    # Load the label map
    with open(label_destination, 'r') as f:
        labels = [line.strip() for line in f.readlines()]
        if labels[0] == '???':  # Remove the first label if it's '???'
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

def detection(any, any2, video_camera):
    log_file_path = "/home/ceg4166/Desktop/CEG4166/Lab5/log_file_run.txt"
    with open(log_file_path, 'a') as log_file:
        while True:
            # Grab frame from PiCamera
            original_frame = video_camera.read()
            if original_frame is None:
                print("Warning: Failed to grab frame")
                continue

            # Convert frame to OpenCV format (BGR)
            frame = cv2.cvtColor(original_frame, cv2.COLOR_RGB2BGR)
            frame_resized = cv2.resize(frame, (width, height))
            input_data = np.expand_dims(frame_resized, axis=0)

            # Normalize for floating point models
            if floating_model:
                input_data = (np.float32(input_data) - input_mean) / input_std

            model_interpreter.set_tensor(input[0]['index'], input_data)
            model_interpreter.invoke()

            # Get detection results
            box = model_interpreter.get_tensor(output[0]['index'])[0]  # Bounding box coordinates
            classes = model_interpreter.get_tensor(output[1]['index'])[0]  # Class index of objects
            conf_value = model_interpreter.get_tensor(output[2]['index'])[0]  # Confidence of detected objects

            for i in range(len(conf_value)):
                if minimum_confidence < conf_value[i] <= 1.0:
                    ymin = int(max(1, (box[i][0] * imH)))
                    xmin = int(max(1, (box[i][1] * imW)))
                    ymax = int(min(imH, (box[i][2] * imH)))
                    xmax = int(min(imW, (box[i][3] * imW)))

                    cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 2)
                    object_name = labels[int(classes[i])]
                    label = f'{object_name}: {int(conf_value[i] * 100)}%'

                    timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
                    log_file.write(f"{object_name} detected at {timestamp}\n")
                    
                    labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                    label_ymin = max(ymin, labelSize[1] + 10)

                    cv2.rectangle(frame, (xmin, label_ymin - labelSize[1] - 10),
                                (xmin + labelSize[0], label_ymin + baseLine - 10), (255, 255, 255), cv2.FILLED)
                    cv2.putText(frame, label, (xmin, label_ymin - 7),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

            cv2.imshow('Object Detection in Stingray', frame)

            if cv2.waitKey(1) == ord('q'):
                print("\nExiting the frame")
                break

    cv2.destroyAllWindows()
    video_camera.stop()

def run_detection():
    """ Starts object detection in a new thread. """
    args = parse_arguments()
    load_model(args)
    picam = Video_PiCamera(resolution=(imW, imH), framerate=60).start()
    time.sleep(1)

    # Start detection in a separate thread
    detection_thread = threading.Thread(target=detection, args=('any1', 'any2', picam,), daemon=True)
    detection_thread.start()

    return picam, detection_thread

if __name__ == "__main__":  # For standalone testing
    run_detection()
