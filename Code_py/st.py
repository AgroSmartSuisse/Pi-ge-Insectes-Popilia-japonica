# This script does the following
# 1) get data from camera (using picamera)
# 2) using OpenCV to analyse the image and count the number of objectes larger than a certain size
# 3) transmit integer value (nbeetles) to GRASSHOPPER over the serial port


######### Capturing camera data to an OpenCV object ###############
import io
import time
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
rawCapture = PiRGBArray(camera)

# allow the camera to warmup
time.sleep(0.1)

# grab an image from the camera
camera.capture(rawCapture, format="bgr")
image = rawCapture.array
# image = cv2.resize(image, None, fx=0.5, fy=0.5)
#cv2.imshow("Image", image)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
#cv2.waitKey(1)

print("Image captured")

######## Analyze image with OpenCV ############################

# Function to display contour area
def get_contour_areas(contours):
    # returns the areas of all contours as list
    all_areas = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        all_areas.append(area)
    return all_areas

# Load image (for test purposes)
# image = cv2.imread('/home/pi/IMG_1795_small.jpg')

# CONTRAST (2=double)  & BRIGHTNESS (from -127 to +127)
alpha = 4
beta = 80
image2 = cv2.addWeighted(image, alpha, np.zeros(image.shape, image.dtype), beta, 0)

# MeanShift
#shifted = cv2.pyrMeanShiftFiltering(image2, 21, 51) # must be color image; parms: spatial & color window radius

# Grayscale
gray = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

# Blur
blurred = cv2.GaussianBlur(gray,(5,5),0)

# Threshold
ret, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)

# Finding Contours and apply filter
edged, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
areas = get_contour_areas(contours)
areas = list(filter(lambda num: num > 400 and num < 800, areas))

print("Total number of Contours found = " + str(len(contours)))
print("Number of contours within size range: :" + str(len(areas)))

# Filter contours with areas >700, choose one of these:
#nbeetles = len(list(filter(lambda x: x > 700, areas)))
#nbeetles = len( [1 for i in areas if i > 700])

#nbeetles = sum(i > 700 for i in areas)
nbeetles = len(areas)

print("Number of Beetles found = " + str(nbeetles))

# write image with contours to SD card
# expo_img = cv2.resize(gray, None, fx=0.5, fy=0.5)
fname = (time.strftime("%d-%b-%y_%Hh%M"))
variable = "/home/pi/Pictures/" + fname + ".jpg"
cv2.drawContours(gray, contours, -1, (0,255,0), 3)
cv2.imwrite(variable, gray)


############## Transmit nbeetles to Grasshopper via serial port #######################
# Find port name on  RPi:  ls -l /dev/tty* (with and without Grasshopper pluged in)
# NOTE: the code below returns a TypeError with Python 3!! Use Python 2)
import serial
nbeetles = str(nbeetles)
#ser = serial.Serial('/dev/tty.usbmodem145221', 9600) # Mac
ser = serial.Serial('/dev/ttyACM0', 9600)  # Rpi
#ser = serial.Serial('/dev/ttyAMA0', 9600)  # Rpi
ser.write(nbeetles.encode())
ser.write('\n')
ser.close()

print("End of serial transmission")
