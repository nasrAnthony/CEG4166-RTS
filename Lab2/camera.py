from picamera2 import Picamera2
import cv2

# Initialize the camera
picam2 = Picamera2()
picam2.start()

while True:
    frame = picam2.capture_array()  # Capture frame as numpy array
    cv2.imshow("Live Camera Feed", frame)  # Display the frame
    
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
        break

cv2.destroyAllWindows()
