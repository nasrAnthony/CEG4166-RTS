import os
import cv2
import time
import threading
import tkinter as tk
from object_detection import run_detection
from final_FaceRecog import run_face_recognition #this might compete with the object detection, since they both use camera at the same time. 
from matplotlib.pylab import *
from mpl_toolkits.axes_grid1 import host_subplot
import matplotlib.animation as animation

import pigpio
import RPi.GPIO as GPIO
from WheelEncoderGPIO import WheelEncoder
import matplotlib.pyplot as plt
import cv2
from HCSR04 import HCSR04
from picamera2 import Picamera2

#ServoWrite class
class ServoWrite:
    def __init__(self, pi, gpio, min_pw=1280, max_pw=1720, min_speed=-1, max_speed=1, min_degree=-90, max_degree=90):
        self.pi = pi
        self.gpio = gpio
        self.min_pw = min_pw
        self.max_pw = max_pw
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.min_degree = min_degree
        self.max_degree = max_degree

        self.slope = (self.min_pw - ((self.min_pw + self.max_pw) / 2)) / self.max_degree
        self.offset = (self.min_pw + self.max_pw) / 2

    def set_pw_speed(self, pulse_width):
        pulse_width = max(min(self.max_pw, pulse_width), self.min_pw)
        self.pi.set_servo_pulsewidth(user_gpio=self.gpio, pulsewidth=pulse_width)

    def set_pw(self, pulse_width):
        pulse_width = max(min(self.max_pw, pulse_width), self.min_degree)
        self.pi.set_servo_pulsewidth(user_gpio=self.gpio, pulsewidth=pulse_width)

    def calc_pw_speed(self, speed):
        return self.slope * speed + self.offset

    def calc_pw(self, degree):
        return self.slope * degree + self.offset

    def set_speed(self, speed):
        speed = max(min(self.max_speed, speed), self.min_speed)
        calculated_pw = self.calc_pw(speed=speed)
        self.set_pw(pulse_width=calculated_pw)

    def stop(self):
        self.set_pw(pulse_width=(self.min_pw + self.max_pw) / 2)

    def max_backward(self):
        self.set_pw(self.max_pw)

    def max_forward(self):
        self.set_pw(self.min_pw)

    def max_left(self):
        self.set_pw(self.max_pw)

    def max_right(self):
        self.set_pw(self.min_pw)

    def set_position(self, degree):
        degree = max(min(self.max_degree, degree), self.min_degree) 
        calculated_pw = self.calc_pw(degree=degree)
        self.set_pw(pulse_width=calculated_pw)

class SONAR(): 
    def __init__(self): 
        self.samples = 5
        #Creation of sonar sensor
        self.sensor = HCSR04(7, 12)
        self.obstacle_flag = [False]
        self.sonar_distance_data = []

    def run_sonar(self): 
        #def Sonar(sensor, samples, obstacle_flag, sonar_distance_data):
        while True:
            s = time.time()
            distance = int(self.sensor.measure(self.samples, "cm"))
            e = time.time()
            #print("Distance:", distance, "cm")
            #print("Used time:", (e - s), "seconds")
            #distance = int(distance)
            if distance < 10:
                print("Object Detected at Distance:", distance, "cm")
                self.obstacle_flag[0] = True
            else: 
                self.obstacle_flag[0] = False
            #print(distance, obstacle_flag)
            self.sonar_distance_data.append(distance)
            time.sleep(0.01)

class movement_controller: 
    def __init__(self, left_servo, right_servo, sonar_servo): 
        self.left_servo = left_servo    
        self.right_servo = right_servo
        self.sonar_servo = sonar_servo
        self.sonar = SONAR()
        self.sonar_turn_data = {
            "s": None, "L": None, "R": None
        }
        self.move = True

    #Movement functionality
    def move_forward(self, timer): 
        self.right_servo.set_position(90)
        self.left_servo.set_position(-90)
        time.sleep(timer)
            
    def move_backward(self, timer): 
        self.right_servo.set_position(-90)
        self.left_servo.set_position(90)
        time.sleep(timer)
        
    def stop_robot(self, timer): 
        self.right_servo.set_position(0)
        self.left_servo.set_position(0)
        time.sleep(timer)
        
    def turn_left(self, timer): 
        self.right_servo.set_position(90)
        self.left_servo.set_position(70)
        time.sleep(timer)
            
    def turn_right(self, timer): 
        self.right_servo.set_position(-90)
        self.left_servo.set_position(-70)
        time.sleep(timer)

    def set_sonar_center(self): 
        self.sonar_servo.set_position(0)
        
    def set_sonar_right(self):
        self.sonar_servo.set_position(90)
        
    def set_sonar_left(self): 
        self.sonar_servo.set_position(-90)
    
    def autonomous_movement(self): 
        #start sonar thread
        sensorThread = threading.Thread(target=SONAR.run_sonar, args=(), daemon = True)
        sensorThread.start()
        while self.move: 
            if not self.sonar.obstacle_flag[0]:  #obstacle not detected. 
                self.move_forward(0.1)
            elif self.sonar.obstacle_flag[0]:  #obstacle detected
                self.stop_robot(0.1) #halt movement. 
                #move sonar left, right, center, record data, check which is largest distance, turn in that direction and go forward again, reset the obstacle_flag. 

            

#GUI CLASS
class GUI_MAP(): 
    def __init__(self): 
        self.matrix=[
                    [1, 1, 1, 1, 1, 1, 1, 1],
                    [1, 0, 0, 0, 0, 0, 3, 1],
                    [1, 0, 1, 1, 1, 0, 0, 1],
                    [1, 0, 0, 0, 1, 0, 0, 1],
                    [1, 1, 1, 0, 1, 0, 0, 1],
                    [1, 1, 1, 0, 1, 0, 0, 1],
                    [1, 1, 1, 0, 1, 0, 0, 1],
                    [1, 1, 1, 0, 0, 0, 0, 1],
                    [1, 0, 0, 0, 1, 1, 0, 1],
                    [1, 0, 1, 0, 0, 0, 0, 1],
                    [1, 0, 1, 1, 0, 0, 0, 1],
                    [1, 0, 0, 0, 0, 1, 0, 1],
                    [1, 2, 0, 0, 0, 0, 0, 1],
                    [1, 1, 1, 1, 1, 1, 1, 1]
                ]
        
        self.color_map = {
            1: "black",  #obstacles
            0: "white",  #path
            2: "red",    #S
            3: "green"   #F
        }

        self.square_size = 50
        self.rows, self.cols = 14, 8
        self.width = self.cols * self.square_size
        self.height = self.rows * self.square_size
    def build_GUI(self, current_robot_movement= None):
        #Create the main window
        root = tk.Tk()
        root.title("Map")

        #Create a Canvas to draw the squares
        canvas = tk.Canvas(root, width=self.width, height=self.height)
        canvas.pack()

        # Draw the squares
        for row in range(self.rows):
            for col in range(self.cols):
                x1 = col * self.square_size
                y1 = row * self.square_size
                x2 = x1 + self.square_size
                y2 = y1 + self.square_size

                #Get the fill color from the matrix
                fill_color = self.color_map.get(self.matrix[row][col], "white")

                canvas.create_rectangle(x1, y1, x2, y2, fill=fill_color, outline="black")

        root.mainloop()

    def start_movement_control(self): 
        pass
            

if __name__ == "__main__":
    gui = GUI_MAP()
    gui.build_GUI()
