# Parallax Servo 360 Python test code to control angular rotations and speeds of the motors
# SEG 4145
# Lab2
from WheelEncoderGPIO import WheelEncoder
import math
import statistics
import time
import pigpio
import threading
import RPi.GPIO as gpioy    
from PID_encoder import straight
#from set_sonar import Sonar 
from keyboard_input import getch
from HCSR04 import HCSR04
from pynput import keyboard

samples = 5

class MotorControl:
    # These are the fixed dimensions of the stingray
    def __init__(self, pi, width_robot=205, diameter_wheels=50, unitsFC=360,
                 dcMin_l=27.3, dcMax_l=969.15, dcMin_r=27.3, dcMax_r=978.25,
                 l_wheel_gpio=16, r_wheel_gpio=20,
                 servo_l_gpio=17, min_pw_l=1280, max_pw_l=1720, min_speed_l=-1, max_speed_l=1,
                 servo_r_gpio=27, min_pw_r=1280, max_pw_r=1720, min_speed_r=-1, max_speed_r=1,
                 sampling_time=0.01, Kp_p=0.1):
        self.pi = pi
        self.width_robot = width_robot
        self.diameter_wheels = diameter_wheels
        self.unitsFC = unitsFC
        self.dcMin_l = dcMin_l
        self.dcMax_l = dcMax_l
        self.dcMin_r = dcMin_r
        self.dcMax_r = dcMax_r
        self.sampling_time = sampling_time
        self.Kp_p = Kp_p

        self.l_wheel = ServoRead(pi=self.pi, gpio=l_wheel_gpio)
        self.r_wheel = ServoRead(pi=self.pi, gpio=r_wheel_gpio)

        self.servo_l = ServoWrite(pi=self.pi, gpio=servo_l_gpio,
                                  min_pw=min_pw_l, max_pw=max_pw_l, min_speed=min_speed_l, max_speed=max_speed_l)
        self.servo_r = ServoWrite(pi=self.pi, gpio=servo_r_gpio,
                                  min_pw=min_pw_r, max_pw=max_pw_r, min_speed=min_speed_r, max_speed=max_speed_r)

        time.sleep(1)
        
    
    def Robot_forward(self, left_speed, right_speed):
        """
        Go forward :)
        """
        self.servo_l.set_speed(left_speed)
        self.servo_r.set_speed(right_speed)
        
    def motorStop(self): 
        self.servo_l.stop()
        self.servo_r.stop()
        
    def get_angle_l(self):
        angle_l = (self.unitsFC - 1) - ((self.l_wheel.read() - self.dcMin_l) * self.unitsFC) / (self.dcMax_l - self.dcMin_l + 1)
        angle_l = max(min((self.unitsFC - 1), angle_l), 0)
        return angle_l

    def get_angle_r(self):
        angle_r = (self.r_wheel.read() - self.dcMin_r) * self.unitsFC / (self.dcMax_r - self.dcMin_r + 1)
        angle_r = max(min((self.unitsFC - 1), angle_r), 0)
        return angle_r

    def set_speed_l(self, speed):
        self.servo_l.set_speed(-speed)

    def set_speed_r(self, speed):
        self.servo_r.set_speed(speed)

    def get_total_angle(self, angle, unitsFC, prev_angle, turns):
        if (angle < (0.25 * unitsFC)) and (prev_angle > (0.75 * unitsFC)):
            turns += 1
        elif (prev_angle < (0.25 * unitsFC)) and (angle > (0.75 * unitsFC)):
            turns -= 1

        if turns >= 0:
            total_angle = (turns * unitsFC) + angle
        else:
            total_angle = ((turns + 1) * unitsFC) - (unitsFC - angle)

        return turns, total_angle

    def get_target_angle(self, number_ticks, angle):
        return angle + number_ticks

    def tick_length(self):
        return math.pi * self.diameter_wheels / self.unitsFC

    def arc_circle(self, degree):
        return degree * math.pi * self.width_robot / 360.0

    def turn(self, degree):
        number_ticks = self.arc_circle(degree) / self.tick_length()
    
    def moveForward(self, left_servo, right_servo, timer): 
        right_servo.set_position(90)
        left_servo.set_position(-90)
        time.sleep(timer)
    
    def stopRobot(self, left_servo, right_servo, timer): 
        right_servo.set_position(0)
        left_servo.set_position(0)
        time.sleep(timer)
        
    def mc_straight(self, distance_in_mm):
        number_ticks = distance_in_mm / self.tick_length()
        print("HERE")
        self.straight_pid(self.left_encoder, self.right_encoder, 2) 
        
    def turn_left(self, left_servo, right_servo, timer): 
        right_servo.set_position(90)
        left_servo.set_position(70)
        time.sleep(timer)
        
    def turn_right(self, left_servo, right_servo, timer): 
        right_servo.set_position(-90)
        left_servo.set_position(-70)
        time.sleep(timer)


class ServoRead:
    def __init__(self, pi, gpio):
        self.pi = pi
        self.gpio = gpio
        self.period = 1 / 910 * 1000000
        self.tick_high = None
        self.duty_cycle = None
        self.duty_scale = 1000
        self.pi.set_mode(gpio=self.gpio, mode=pigpio.INPUT)

    def read(self):
        return self.duty_cycle


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


pi = pigpio.pi()
           
def run_path_1(mc, left_servo, right_servo): 
    print("Executing Path 1")
    #running path 1
    mc.moveForward(left_servo, right_servo, 2)
    mc.stopRobot(left_servo, right_servo, 1)
    mc.turn_left(left_servo, right_servo, 0.5)
    mc.moveForward(left_servo, right_servo, 2)
    mc.stopRobot(left_servo, right_servo, 1)
    mc.turn_right(left_servo, right_servo, 0.5)
    mc.stopRobot(left_servo, right_servo, 1)
    mc.moveForward(left_servo, right_servo, 2) 
    mc.stopRobot(left_servo, right_servo, 1)
    mc.turn_right(left_servo, right_servo, 0.5)
    mc.stopRobot(left_servo, right_servo, 1)
    mc.moveForward(left_servo, right_servo, 2)
    mc.stopRobot(left_servo, right_servo, 1)
    mc.turn_left(left_servo, right_servo, 0.5)
    mc.stopRobot(left_servo, right_servo, 1)
    mc.moveForward(left_servo, right_servo, 2)
    mc.stopRobot(left_servo, right_servo, 1)
    
    
def run_path_2(mc, left_servo, right_servo): 
    print("Executing Path 2") 
    #running path 2 
    mc.moveForward(left_servo, right_servo, 1)
    mc.stopRobot(left_servo, right_servo, 1)
    mc.turn_right(left_servo, right_servo, 0.5)
    mc.stopRobot(left_servo, right_servo, 1)
    mc.moveForward(left_servo, right_servo, 0.5)
    mc.stopRobot(left_servo, right_servo, 1)
    mc.turn_left(left_servo, right_servo, 0.7)
    mc.stopRobot(left_servo, right_servo, 1)
    mc.moveForward(left_servo, right_servo, 2.5)
    mc.stopRobot(left_servo, right_servo, 1)
    mc.turn_right(left_servo, right_servo, 0.9)
    mc.stopRobot(left_servo, right_servo, 1)
    mc.moveForward(left_servo, right_servo, 0.5)
    mc.stopRobot(left_servo, right_servo, 1)
    mc.turn_left(left_servo, right_servo, 0.5)
    mc.stopRobot(left_servo, right_servo, 1)
    mc.moveForward(left_servo, right_servo, 1)
    mc.stopRobot(left_servo, right_servo, 1)
    
    
    
def keyboard_control_test(): 
    while True: 
        char = getch()
        if char == "w":
            mc.moveForward(left_servo, right_servo, 1)
        if char == "a": 
            mc.turn_left(left_servo, right_servo, 0.1)
        if char == "d":  
            mc.turn_right(left_servo, right_servo, 0.1)
        if char == "s":
            mc.stopRobot(left_servo, right_servo, 0.2)
        if char == "q":
            break
                
samples = 5

#Creation of sonar sensor
sensor = HCSR04(7, 12)

#Function for sonar sensor takes HCSR04 object and sample number for accuracy of distance
def Sonar(sensor, samples, obstacle_flag):
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
        print(distance, obstacle_flag)
        time.sleep(0.01)
        
key_states = {
    "w": False,
    "a": False,
    "d": False,
    "s": False
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

def main():
    left_servo = ServoWrite(pi=pi, gpio=23)
    right_servo = ServoWrite(pi=pi, gpio=24)
    obstacle_flag = [False]

    mc = MotorControl(pi= pi)
    #run_path_1()
    run_path_2(mc, left_servo, right_servo)
    #sensorThread = threading.Thread(target=Sonar, args=(sensor, samples, obstacle_flag), daemon = True)
    sensorThread.start()
    #keyboard_control_test()
    
    # Start the keyboard listener
    #listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    #listener.start()

    #while True:
        #print(obstacle_flag[0])
          
    #    if key_states["w"] and not(obstacle_flag[0]):
    #        mc.moveForward(left_servo, right_servo, 0.01)
    #        
    #    if key_states["a"]:
    #        mc.turn_left(left_servo, right_servo, 0.01)
    #
    #    if key_states["d"]:
    #        mc.turn_right(left_servo, right_servo, 0.01)

    #    if key_states["s"]:
    #        mc.stopRobot(left_servo, right_servo, 0.01)
    #        
    #    else: 
    #        mc.stopRobot(left_servo, right_servo, 0.01)

        #time.sleep(0.05)  # Small delay to reduce CPU usage
    
    #while True: 
    #    if obstacle_flag:
    #        mc.stopRobot(left_servo, right_servo, 5)
    #    char = getch()
    #    if char == "w":
    #        mc.moveForward(left_servo, right_servo, 0.01)
    #        char.clear()
    #    if char == "a": 
    #        mc.turn_left(left_servo, right_servo, 0.1)
    #    if char == "d":  
    #        mc.turn_right(left_servo, right_servo, 0.1)
    #    if char == "s":
    #        mc.stopRobot(left_servo, right_servo, 0.2)
    #    if char == "q":
    #        break 
           
    
if __name__ == "__main__":
    main()
