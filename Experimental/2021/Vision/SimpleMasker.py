import cv2
import numpy as np
import sys

captureBlue = False
windowName = 'Simple Masking'
filename = "Power Ball.jpg"

# Yellow(ish)
low_H = 21 # scaleHueForOpenCV(raw_low_H)
low_S = 148
low_V = 93
high_H = 30
high_S = 250
high_V = 255

if len(sys.argv) > 1:
    filename = sys.argv[1]

print("filename=",filename)

# Read an image
image = cv2.imread(filename)
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

if captureBlue:
    # define range of blue color in HSV
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
else:
    # define range of yellow color in HSV
    lower_yellow = np.array([low_H,low_S,low_V])
    upper_yellow = np.array([high_H,high_S,high_V])
    # Threshold the HSV image to get only yellow colors
    mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

# Bitwise-AND mask and original image
res = cv2.bitwise_and(image,image, mask= mask)

# Create a window and set Mousecallback to a function for that window
cv2.namedWindow(windowName)
# Do until esc pressed
while (1):
    cv2.imshow(windowName, res)
    k = cv2.waitKey(10) 
    if k == ord('q') or k == 27:
        break
# if esc is pressed, close all windows.
cv2.destroyAllWindows()
