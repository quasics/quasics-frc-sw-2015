import cv2 as cv
import numpy as np

captureBlue = False

# Yellow(ish)
yellow_low_H = 21 # scaleHueForOpenCV(raw_low_H)
yellow_low_S = 148
yellow_low_V = 93
yellow_high_H = 30
yellow_high_S = 250
yellow_high_V = 255

# Blue shades from OpenCV tutorial
blue_low_H = 110 # scaleHueForOpenCV(raw_low_H)
blue_low_S = 50
blue_low_V = 50
blue_high_H = 130
blue_high_S = 255
blue_high_V = 255

cap = cv.VideoCapture(0)
while(1):
    # Take each frame
    _, frame = cap.read()
    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    if captureBlue:
        # define range of blue color in HSV
        lower = np.array([blue_low_H,blue_low_S,blue_low_V])
        upper = np.array([blue_high_H,blue_high_S,blue_high_V])
    else:
        # define range of yellow color in HSV
        lower = np.array([yellow_low_H,yellow_low_S,yellow_low_V])
        upper = np.array([yellow_high_H,yellow_high_S,yellow_high_V])

    # Threshold the HSV image to get only the targeted color
    mask = cv.inRange(hsv, lower, upper)

    # Bitwise-AND mask and original image
    res = cv.bitwise_and(frame,frame, mask= mask)
    cv.imshow('frame',frame)
    cv.imshow('mask',mask)
    cv.imshow('res',res)
    k = cv.waitKey(5)
    if k == 27 or k == ord('q'):
        break
cv.destroyAllWindows()
