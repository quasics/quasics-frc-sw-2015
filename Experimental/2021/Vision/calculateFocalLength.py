# import the necessary packages
import numpy as np
import cv2
import sys

# If "showImage" is True, then we'll pop up a masked-out version of the
# image, showing the region that we've identified as the target.
def findTarget(image, showImage):
    # convert the image to grayscale, blur it, and detect edges
    frame_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    frame_threshold = cv2.inRange(frame_HSV, low_range, high_range)

    # find the contours in the edged image and keep the largest one;
    # we'll assume that this is our piece of paper in the image
    contours, _ = cv2.findContours(frame_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    count = len(contours)

    index = -1
    bestIndex = -1
    best = None
    for contour in contours:
        index = index + 1
        
        # Does the current image beat our best-scoring one so far?
        if bestIndex == -1 or cv2.contourArea(contour) > cv2.contourArea(best):
            best = contour
            bestIndex = index

    if showImage:
        output_img = cv2.bitwise_and(image, image, mask = frame_threshold)
        cv2.drawContours(output_img, contours, bestIndex, color = (0, 0, 255), thickness = 3)
        while (1):
            cv2.imshow('Color Picker', output_img)
            k = cv2.waitKey(10) 
            if k == ord('q') or k == 27:
                break
        cv2.destroyAllWindows()

    # compute the bounding box of the of the paper region and return it
    return cv2.minAreaRect(best)


######################################################################
#
# Core configuration data begins here
#
######################################################################

# Default file to use (for demo/test purposes)
filename = "c920_176cm.jpg"

#
# Note: the following data will all need to be tweaked if you use a different file.
#

# Initialize the known object width: in this case, it is a piece of
# paper 11 inches wide
KNOWN_WIDTH = 27.94     # cm (11in)

# Color range for the sheet of paper in the default file.
low_range = (10, 30, 213)
high_range = (147, 65, 255)

# initialize the known distance from the camera to the object, which
# in this case is 174cm
KNOWN_DISTANCE = 174.0


######################################################################
#
# Main execution starts here
#
######################################################################

if len(sys.argv) > 1:
    filename = sys.argv[1]

# load the image that contains an object (a piece of paper) that is KNOWN_DISTANCE
# from our camera, then find the paper marker in the image, and calculate the focal
# length
image = cv2.imread(filename)

marker = findTarget(image, False)
focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH

# This also implies (for verification) that:
#      perceivedWidth = focalLength * knownWidth / knownDistance

print("Focal length is {} pixels".format(focalLength))

