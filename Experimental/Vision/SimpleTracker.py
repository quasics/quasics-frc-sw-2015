import cv2 as cv
import numpy as np

captureBlue = False

cap = cv.VideoCapture(0)
while(1):
    # Take each frame
    _, frame = cap.read()
    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    if captureBlue:
        # define range of blue color in HSV
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])
        # Threshold the HSV image to get only blue colors
        mask = cv.inRange(hsv, lower_blue, upper_blue)
    else:
        # define range of yellow color in HSV
        lower_yellow = np.array([5,114,29])
        upper_yellow = np.array([23,210,148])
        # Threshold the HSV image to get only yellow colors
        mask = cv.inRange(hsv, lower_yellow, upper_yellow)
    # Bitwise-AND mask and original image
    res = cv.bitwise_and(frame,frame, mask= mask)
    cv.imshow('frame',frame)
    cv.imshow('mask',mask)
    cv.imshow('res',res)
    k = cv.waitKey(5)
    if k == 27 or key == ord('q'):
        break
cv.destroyAllWindows()
