#Import Libraries
from matplotlib.pylab import *
from mpl_toolkits.axes_grid1 import host_subplot
import matplotlib.animation as animation
import time
import threading
import pigpio
import RPi.GPIO as GPIO
from WheelEncoderGPIO import WheelEncoder
import matplotlib.pyplot as plt
import cv2
from picamera2 import Picamera2


#Keyboard control from lab 2 + sonar imports
from HCSR04 import HCSR04
from pynput import keyboard

class multiplePlots:
    def __init__(self, leftEncoderCount, rightEncoderCount,
                 samples, xmax):
        self.leftEncoderCount = leftEncoderCount
        self.rightEncoderCount = rightEncoderCount
        self.samples = samples
        self.xmax = xmax
        self.ymax = 200
		
        #Define sampleTime to calculate the speed and the tf variable, which
        #represents 
        #the end time of the speed measuring
        self.sampleTime = 1
        self.tf = time.time() + self.sampleTime

        # Sent for figure
        self.font = {'size'   : 9}
        matplotlib.rc('font', **self.font)

        # Setup figure and subplots
        self.f0 = figure(num = 0, figsize = (6, 4))
        self.f0.suptitle("Oscillation decay", fontsize=12)
        self.ax01 = subplot2grid((2, 2), (0, 0))
        self.ax02 = subplot2grid((2, 2), (0, 1))
        self.ax03 = subplot2grid((2, 2), (1, 0))

        # Data Placeholders
        self.yp1=zeros(0)
        self.yv1=zeros(0)
        self.yp2=zeros(0)
        self.yv2=zeros(0)
        self.t=zeros(0)
        #initialize empty array of zeros for sonar data
        self.sonar_values = zeros(0)

        # set plots
        self.p011, = self.ax01.plot(self.t,self.yp1,'b-', label="LeftWheel")
        self.p012, = self.ax01.plot(self.t,self.yp2,'g-', label="RightWheel")

        self.p021, = self.ax02.plot(self.t,self.yv1,'b-', label="LeftWheel")
        self.p022, = self.ax02.plot(self.t,self.yv2,'g-', label="RightWheel")
        
        self.p031, = self.ax03.plot(self.t,self.sonar_values,'r-', label="SonarData")

        # set legends
        self.ax01.legend([self.p011,self.p012],
                         [self.p011.get_label(),self.p012.get_label()])
        self.ax02.legend([self.p021,self.p022],
                         [self.p021.get_label(),self.p022.get_label()])
        #sonar data legend
        self.ax03.legend([self.p031.get_label()])

        # Data Update
        self.xmin = 0.0
        self.x = 0.0

        # Set titles of subplots
        self.ax01.set_title('Distance vs Time')
        self.ax02.set_title('Speed(cm/s) vs Time')

        # set y-limits
        self.ax01.set_ylim(0,200)
        self.ax02.set_ylim(0,50)

        # set x-limits
        self.ax01.set_xlim(0,5.0)
        self.ax02.set_xlim(0,5.0)

        # Turn on grids
        self.ax01.grid(True)
        self.ax02.grid(True)
        self.ax03.grid(True)

        # set label names
        self.ax01.set_xlabel("t")
        self.ax01.set_ylabel("Distance")
        self.ax02.set_xlabel("t")
        self.ax02.set_ylabel("Ticks")
        self.ax03.set_xlabel("t")
        self.ax03.set_ylabel("Distance")

	    #start the variables with 0
        self.totLeftDist = 0
        self.totRightDist = 0
        
        self.leftSpeed = 0
        self.rightSpeed = 0
        
        self.ini_pos_left = 0
        self.ini_pos_right = 0
		
		# measure the speed of the robot
    def getSpeed(self):
        if (time.time() >= self.tf):
            self.tf = time.time() + self.sampleTime
            self.leftSpeed = (self.leftEncoderCount.getTotalDistance() - self.ini_pos_left)/self.sampleTime
            self.rightSpeed = (self.rightEncoderCount.getTotalDistance() - self.ini_pos_right)/self.sampleTime
            self.ini_pos_left = self.leftEncoderCount.getTotalDistance()
            self.ini_pos_right = self.rightEncoderCount.getTotalDistance()
            print(self.leftSpeed, self.rightSpeed)

    def teste(self):
        return self.xmax, self.yp1

    def updateData(self, new_sonar_distance):
        self.totLeftDist = self.leftEncoderCount.getTotalDistance()
        self.totRightDist = self.rightEncoderCount.getTotalDistance()
        
        self.getSpeed()
        
        self.yp1=append(self.yp1,self.totLeftDist)
        self.yv1=append(self.yv1,self.leftSpeed)
        self.yp2=append(self.yp2,self.totRightDist)
        self.yv2=append(self.yv2,self.rightSpeed)
        
        #add new incoming sonar distance 
        #print("SONARVALUES", self.sonar_values)
        self.sonar_values=append(self.sonar_values, new_sonar_distance)
        #time
        self.t=append(self.t,self.x)

        self.x += 0.3

        self.p011.set_data(self.t,self.yp1)
        self.p012.set_data(self.t,self.yp2)

        self.p021.set_data(self.t,self.yv1)
        self.p022.set_data(self.t,self.yv2)
        
        self.p031.set_data(self.t, self.sonar_values)
		
		#actualizing data
        if self.yp1[-1] >= self.ymax-40.00:
            self.p011.axes.set_ylim(self.yp1[-1]-self.ymax+40.0,self.yp1[-1]+40.0)
        if self.yp2[-1] >= self.ymax-40.00:
            self.p011.axes.set_ylim(self.yp2[-1]-self.ymax+40.0,self.yp2[-1]+40.0)
        if self.yv1[-1] >= self.ymax-40.00:
            self.p021.axes.set_ylim(self.yv1[-1]-self.ymax+40.0,self.yv1[-1]+40.0)
        if self.yv2[-1] >= self.ymax-40.00:
            self.p021.axes.set_ylim(self.yv2[-1]-self.ymax+40.0,self.yv2[-1]+40.0)
        
        if self.sonar_values[-1] >= self.ymax-40.00:
            self.p031.axes.set_ylim(self.sonar_values[-1]-self.ymax+40.0,self.sonar_values[-1]+40.0)
		
        if self.x >= self.xmax-1.00:
            self.p011.axes.set_xlim(self.x-self.xmax+1.0,self.x+1.0)
            self.p021.axes.set_xlim(self.x-self.xmax+1.0,self.x+1.0)
            self.p031.axes.set_xlim(self.x-self.xmax+1.0,self.x+1.0)
            

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

class movement_controller: 
    def __init__(self, left_servo, right_servo, sonar_servo): 
        self.left_servo = left_servo    
        self.right_servo = right_servo
        self.sonar_servo = sonar_servo

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
        
samples = 5

#Creation of sonar sensor
sensor = HCSR04(7, 12)

#Function for sonar sensor takes HCSR04 object and sample number for accuracy of distance
def Sonar(sensor, samples, obstacle_flag, sonar_distance_data):
    while True:
        s = time.time()
        distance = int(sensor.measure(samples, "cm"))
        e = time.time()
        #print("Distance:", distance, "cm")
        #print("Used time:", (e - s), "seconds")
        #distance = int(distance)
        if distance < 10:
            print("Object Detected at Distance:", distance, "cm")
            obstacle_flag[0] = True
        else: 
            obstacle_flag[0] = False
        #print(distance, obstacle_flag)
        sonar_distance_data.append(distance)
        time.sleep(0.01)
        
key_states = {
    "w": False, #forward
    "a": False, #turn CCW
    "d": False, #turn CW
    "s": False, #backward
    "x": False, #stop
    "k": False, #Center Sonar
    "l": False, #Sonar left
    "j": False, #Sonar right
    "m": False, #Sweep sonar
    "p": False, #Stop motors + pi
    "c": False, #Show camera
    "v": False  #Close camera
}

def on_press(key):
    try:
        if key.char in key_states:
            key_states[key.char] = True
    except AttributeError:
        pass  # Ignore special keys

def on_release(key):
    try:
        if key.char in key_states:
            key_states[key.char] = False
        if key.char == "q":
            return False  # Stop listener
    except AttributeError:
        pass

#Returning values to plot the data
def loopData(frame):
    plotData.updateData(sonar_distance_data[-1])
    return plotData.p011, plotData.p012, plotData.p021, plotData.p022, plotData.p031
    

def keyboard_movement(controller: movement_controller, obstacle_flag, close_feed): 
    #Start the keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    controller.set_sonar_center() #center the sonar before starting

    while True:
        if obstacle_flag[0]: 
            controller.move_backward(1.25)
            
        if key_states["w"] and not(obstacle_flag[0]):
            controller.move_forward(0.1)
            
        if key_states["a"]:
            controller.turn_left( 0.1)
    
        if key_states["d"]:
            controller.turn_right(0.1)

        if key_states["s"]:
            controller.move_backward(0.1)
            
        if key_states["x"]:
            controller.stop_robot(0.01)
        
        if key_states["l"]: 
            controller.set_sonar_left()
            
        if key_states["j"]: 
            controller.set_sonar_right()
            
        if key_states["k"]: 
            controller.set_sonar_center()
            
        if key_states["v"]:    
            close_feed[0] = True
        
        if key_states["c"]:    
            close_feed[0] = False
            
        if key_states["p"]:
            break
        
        else: 
            controller.stop_robot(0.01)

#creation of two encoders using WheelEncoder class
#start graphing animation:
xmax = 5

leftEncoderCount = WheelEncoder(11, 1028, 5.65/2)
rightEncoderCount = WheelEncoder(13, 1028, 5.65/2)
plotData = multiplePlots(leftEncoderCount, rightEncoderCount, samples, xmax) 
sonar_distance_data = [0]


def camera(key_states):
    key_states_dict = key_states[0]
    picam2 = Picamera2()
    picam2.start()
    close_cam = False
    
    if key_states_dict["v"]: 
        close_cam = True

    if key_states_dict["c"]:
        close_cam = False
        
    while True:
        frame = picam2.capture_array()  # Capture frame as numpy array
        
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
            break
            
        if not close_cam:
            cv2.imshow("Stingray Live Video Feed", frame)
            
    cv2.destroyAllWindows()
    picam2.close()

def main():
    pi = pigpio.pi()
    left_servo = ServoWrite(pi=pi, gpio=23)
    right_servo = ServoWrite(pi=pi, gpio=24)
    sonar_servo = ServoWrite(pi=pi, gpio= 25,min_pw = 500, max_pw = 2500) #adjusted PW min max to allow for 90 degree rotations in both directions for the sonar sensor. 
    controller = movement_controller(right_servo, left_servo, sonar_servo)
    obstacle_flag = [False]
    close_feed = [False]
    
    #start keyboard movement thread
    movementThread = threading.Thread(target=keyboard_movement, args=(controller, obstacle_flag, close_feed), daemon = True)
    movementThread.start()
    
    #start sensor thread
    sensorThread = threading.Thread(target=Sonar, args=(sensor, samples, obstacle_flag, sonar_distance_data), daemon = True)
    sensorThread.start()
    
    #start camera thread
    cameraThread = threading.Thread(target=camera, args=([key_states]), daemon = True)
    cameraThread.start()
    
    #Create an animation to plot the data, during 1 minute
    simulation = animation.FuncAnimation(fig=plotData.f0, func=loopData,
                    blit=False, frames=200, interval=20, repeat=False)
    plt.show()

if __name__ == "__main__":
    main()