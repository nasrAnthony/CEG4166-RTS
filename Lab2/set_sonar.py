import threading
import time
from HCSR04 import HCSR04

samples = 5

#Creation of sonar sensor
sensor = HCSR04(7, 12)

#Function for sonar sensor takes HCSR04 object and sample number for accuracy of distance
def Sonar(sensor, samples):
    while True:
        s = time.time()
        distance = sensor.measure(samples, "cm")
        e = time.time()
        print("Distance:", distance, "cm")
        print("Used time:", (e - s), "seconds")
        time.sleep(0.01)

sensorThread = threading.Thread(target=Sonar, args=(sensor, samples))
sensorThread.start()
