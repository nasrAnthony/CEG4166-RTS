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
from PlotDataRobot_Lab4 import multiplePlots, ServoWrite

import pigpio
import RPi.GPIO as GPIO
from WheelEncoderGPIO import WheelEncoder
import matplotlib.pyplot as plt
import cv2
from HCSR04 import HCSR04
from picamera2 import Picamera2

class SONAR(): 
    def __init__(self): 
        self.samples = 3
        #Creation of sonar sensor
        self.sensor = HCSR04(7, 12)
        self.obstacle_flag = [False]
        self.sonar_distance_data = [0]

    def run_sonar(self): 
        #def Sonar(sensor, samples, obstacle_flag, sonar_distance_data):
        counter = 0
        while True:
            s = time.time()
            distance = int(self.sensor.measure(self.samples, "cm"))
            e = time.time()
            #print("Distance:", distance, "cm")
            #print("Used time:", (e - s), "seconds")
            #distance = int(distance)
            if distance < 15:
                print("Object Detected at Distance:", distance, "cm")
                if counter == 0:
                    self.obstacle_flag[0] = True
                    counter = 10
                else: 
                    counter -= 1
            else: 
                self.obstacle_flag[0] = False
            #print(distance, obstacle_flag)
            self.sonar_distance_data.append(distance)
            time.sleep(0.1)

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
        self.move_map = [] #populate with tuples ex format ('f', '90') #forward 90 cm
        self.sonar_distance_data = [0]
        self.inc_x = False
        self.x = 0
        self.y = 0
                        

    #Movement functionality
    def move_forward(self, timer): 
        self.right_servo.set_position(30)
        self.left_servo.set_position(-28)
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
    
    def autonomous_control(self):
        #start sonar thread
        sensorThread = threading.Thread(target=self.sonar.run_sonar, args=(), daemon = True)
        sensorThread.start()
        
        #start object detection thread. 
        objectDetectionThread = threading.Thread(target=run_detection, args=(self), daemon = True)
        objectDetectionThread.start()
        self.set_sonar_center()
        while self.move:
            if not self.sonar.obstacle_flag[0]:  #obstacle not detected. 
                self.move_forward(0.1)
                self.move_map.append("F") #increments of 10 for now, update with distance traveled from the graphed data. 
                #self.stop_robot(0.1)
                if self.inc_x: 
                    self.x+=1
                else: 
                    self.y+=1
                time.sleep(0.1)
            elif self.sonar.obstacle_flag[0]:  #obstacle detected
                self.stop_robot(0.1) #halt movement. 
                #move sonar left, right, center, record data, check which is largest distance, turn in that direction and go forward again, reset the obstacle_flag. 
                self.set_sonar_left()
                time.sleep(1) #inject delay
                #grab data.
                self.sonar_turn_data['L'] = self.sonar.sonar_distance_data[-1]
                #check right.
                self.set_sonar_right()
                time.sleep(1)
                self.sonar_turn_data['R'] = self.sonar.sonar_distance_data[-1]
                print(self.sonar_turn_data)
                #turn robot in direction with highest highest distance. 
                if(self.sonar_turn_data['R'] >= self.sonar_turn_data['L']): 
                    self.turn_right(0.55)
                    print("decided to move right")
                    self.move_map.append("R")
                    self.inx_x = True
                elif(self.sonar_turn_data['L'] > self.sonar_turn_data['R']):
                    self.turn_left(0.55)
                    print("decided to move left")
                    self.move_map.append("L")
                    self.inc_x = False
                #center sonar again. 
                self.set_sonar_center()
                #after turn, reset move obstacle flag to allow forward movement. 
                self.sonar.obstacle_flag[0] = False
            self.sonar_distance_data = self.sonar.sonar_distance_data #update sonar data for graphing. 
        return sensorThread, objectDetectionThread

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

        self.leftEncoderCount = WheelEncoder(11, 1028, 5.65/2)
        self.rightEncoderCount = WheelEncoder(13, 1028, 5.65/2)
        self.plotData = multiplePlots(self.leftEncoderCount, self.rightEncoderCount, 5, 5)
        self.pi = pigpio.pi()
        self.left_servo = ServoWrite(pi=self.pi, gpio=23)
        self.right_servo = ServoWrite(pi=self.pi, gpio=24)
        self.sonar_servo = ServoWrite(pi=self.pi, gpio= 25,min_pw = 500, max_pw = 2500)
        self.MC = movement_controller(self.left_servo, self.right_servo, self.sonar_servo)

        # Find start position (green square)
        for row in range(self.rows):
            for col in range(self.cols):
                if self.matrix[row][col] == 3:
                    self.player_x = col
                    self.player_y = row
                    break
        
        self.direction = 'left'  # Initial direction
        self.directions = ['up', 'right', 'down', 'left']

    def build_GUI(self):
        # Create the main window
        self.root = tk.Tk()
        self.root.title("Map")

        # Create a Canvas to draw the squares
        self.canvas = tk.Canvas(self.root, width=self.width, height=self.height)
        self.canvas.pack()

        # Draw the squares
        self.draw_map()
        
        # Draw the arrow
        self.arrow = self.canvas.create_polygon(self.get_arrow_coords(), fill="black")

        #check move list next move...
        self.root.after(100, self.on_key_event)  # Check every 100 milliseconds

        self.root.mainloop()
    
    def draw_map(self):
        for row in range(self.rows):
            for col in range(self.cols):
                x1 = col * self.square_size
                y1 = row * self.square_size
                x2 = x1 + self.square_size
                y2 = y1 + self.square_size

                # Get the fill color from the matrix
                fill_color = self.color_map.get(self.matrix[row][col], "white")

                self.canvas.create_rectangle(x1, y1, x2, y2, fill=fill_color, outline="black")
    
    def get_arrow_coords(self):
        x = self.player_x * self.square_size + self.square_size // 2
        y = self.player_y * self.square_size + self.square_size // 2
        size = 15
        
        if self.direction == 'up':
            return [x, y - size, x - size, y + size, x + size, y + size]
        elif self.direction == 'right':
            return [x + size, y, x - size, y - size, x - size, y + size]
        elif self.direction == 'down':
            return [x, y + size, x - size, y - size, x + size, y - size]
        elif self.direction == 'left':
            return [x - size, y, x + size, y - size, x + size, y + size]

    def on_key_event(self):
        if (len(self.MC.move_map) != 0): #meaning a move was done by the robot. 
            move = self.MC.move_map.pop() #get the latest move.
            if move == "L":
                self.direction = self.directions[(self.directions.index(self.direction) - 1) % 4]
            elif move == "R":
                self.direction = self.directions[(self.directions.index(self.direction) + 1) % 4]
            elif move == "F":
                self.move_forward()
            
            self.canvas.coords(self.arrow, self.get_arrow_coords())

        self.root.after(100, self.on_key_event)
    
    def move_forward(self):
        prev_x, prev_y = self.player_x, self.player_y
        
        if self.direction == 'up' and self.player_y > 0 and self.matrix[self.player_y - 1][self.player_x] != 1:
            self.player_y -= 1
        elif self.direction == 'down' and self.player_y < self.rows - 1 and self.matrix[self.player_y + 1][self.player_x] != 1:
            self.player_y += 1
        elif self.direction == 'left' and self.player_x > 0 and self.matrix[self.player_y][self.player_x - 1] != 1:
            self.player_x -= 1
        elif self.direction == 'right' and self.player_x < self.cols - 1 and self.matrix[self.player_y][self.player_x + 1] != 1:
            self.player_x += 1
        
        # Draw trail from previous position to new position
        self.canvas.create_line(prev_x * self.square_size + self.square_size // 2,
                                prev_y * self.square_size + self.square_size // 2,
                                self.player_x * self.square_size + self.square_size // 2,
                                self.player_y * self.square_size + self.square_size // 2,
                                fill='black', width=2)
        


    def loopData(self, frame):
        self.plotData.updateData(self.MC.sonar_distance_data[-1])
        return self.plotData.p011, self.plotData.p012, self.plotData.p021, self.plotData.p022,self.plotData.p031
    
    def start_lab5(self):
        GUI_thread = threading.Thread(target= self.build_GUI, args=(), daemon= True)
        GUI_thread.start()
        print("GUI thread launched")
        movement_thread = threading.Thread(target=self.MC.autonomous_control, args=(), daemon = True)
        movement_thread.start()
        print("Movement thread launched")
        #Create an animation to plot the data, during 1 minute
        simulation = animation.FuncAnimation(fig=self.plotData.f0, func=self.loopData,
                    blit=False, frames=200, interval=20, repeat=False)
        print("Graphing thread launched")
        plt.show()
            

if __name__ == "__main__":
    gui = GUI_MAP()
    gui.start_lab5()
