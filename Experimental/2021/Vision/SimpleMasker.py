import cv2
import numpy as np
import sys

captureGreen = True
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

# Rough shades for green (from photoreflective tape and LED rings)
# Pass 1: 38/20/212 - 75/86/255 (Good for Reflective-1)
# Pass 2: 55/20/162 - 75/102/255 (Good for Reflective-2)
# Pass 3: 42/20/162 - 75/102/255 (reasonably good across Reflective-[123])
green_low_H = 42
green_low_S = 20
green_low_V = 162
green_high_H = 75
green_high_S = 102
green_high_V = 255

if len(sys.argv) > 1:
    filename = sys.argv[1]

print("filename=",filename)

# Read an image
image = cv2.imread(filename)
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

if captureGreen:
    lower_green = np.array([green_low_H,green_low_S,green_low_V])
    upper_green = np.array([green_high_H,green_high_S,green_high_V])
    mask = cv2.inRange(hsv_image, lower_green, upper_green)
elif captureBlue:
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

# Create a window
cv2.namedWindow(windowName)
cv2.imshow(windowName, res)

# Do until esc pressed
while (1):
    k = cv2.waitKey(10) 
    if k == ord('q') or k == 27:
        break

# Close all windows on shutdown.
cv2.destroyAllWindows()
