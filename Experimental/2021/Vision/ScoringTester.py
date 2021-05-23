# A sample application (based on a published sample for the CV2 library) for
# determining the color range needed in order to isolate a specific target
# in an image.  The program will default to reading from a connected webcam,
# but can also be used with a sample (still) image.
#
# This was used in 2021 to figure out what color range should be used to
# "see" the yellow balls used in the FRC game "Infinite Recharge", allowing
# the vision-processing code on our robots to be tuned appropriately.

from __future__ import print_function
import argparse
import cv2
import math
import numpy as np

# Note: openCV uses a constrained range of 0..179 for H, and 0..255 for S/V
# So we need to normalize accordingly.
max_value = 255
max_value_H = 360//2

# Will be used to communicate the size of the image (or frame of video) being
# evaluated for possible targets between the various functions (so that we
# don't need to keep sending it around, just in case it's needed).
#
# Note that it's *cleaner* to send it around, but we'll trade off some
# cleanliness here (and for a few other things) for improved simplicity.
frame_height = 0
frame_width = 0

# If true, we're looking for the exhaust port (using a green LED ring
# and the retroreflective tape); otherwise, we're looking for the
# power cells (as in the "IR @ Home" challenges).
portDetection = True

# Default yellow(ish) colors, for use when detecting power cells.
yellow_low_H = 21 # scaleHueForOpenCV(raw_low_H)
yellow_low_S = 148
yellow_low_V = 93
yellow_high_H = 30
yellow_high_S = 250
yellow_high_V = 255

# Default shades for green (from photoreflective tape and LED rings), for
# use when we're detecting the exhaust port.
# 
# Pass 1: 38/20/212 - 75/86/255 (Good for large Reflective-1)
# Pass 2: 55/20/162 - 75/102/255 (Good for large Reflective-2)
# Pass 3: 42/21/162 - 75/102/255 (reasonably good across large Reflective-[123])
# 
# Unfortunately, those #s don't work for the 160x120 sample images, as the
# particle board can't be distinguished.
# Pass 1 on small images: 7/34/145 - 72/121/250
green_low_H = 42
green_low_S = 21
green_low_V = 162
green_high_H = 75
green_high_S = 102
green_high_V = 255

# Names of various windows/widgets in the GUI.
window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
window_masking_name = 'Masked Image'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

#-----------------------------------------------------------------------------
# Functions related to movement of the sliders controlling vision thresholds
#-----------------------------------------------------------------------------
def reportStats():
    print("H:", low_H, '/', high_H, "S:", low_S, '/', high_S, "V:", low_V, '/', high_V)

def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv2.setTrackbarPos(low_H_name, window_detection_name, low_H)
    reportStats()
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv2.setTrackbarPos(high_H_name, window_detection_name, high_H)
    reportStats()
def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv2.setTrackbarPos(low_S_name, window_detection_name, low_S)
    reportStats()
def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv2.setTrackbarPos(high_S_name, window_detection_name, high_S)
    reportStats()
def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv2.setTrackbarPos(low_V_name, window_detection_name, low_V)
    reportStats()
def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv2.setTrackbarPos(high_V_name, window_detection_name, high_V)
    reportStats()

#-----------------------------------------------------------------------------
# Functions related to scoring possible targets
#-----------------------------------------------------------------------------

# Converts a ratio with ideal value of 1 to a score (range: [0.0 - 1.0]).
def ratioToScore(ratio):
	return (max(0.0, min((1-abs(1-ratio)), 1.0)))

# Reflective tape on InfiniteRecharge target has bounding box of 17" x 39.25".
def portBoundingRatioScore(contour):
    expectedRatio = float(17) / 39.25
    minAreaRect = cv2.minAreaRect(contour)
    center, size, angle = minAreaRect
    width, height = size
    actualRatio = float(width) / float(height)
    comparativeRatio = actualRatio / expectedRatio
    return ratioToScore(comparativeRatio)

# Pick bigger things....
def percentOfFieldScore(contour):
    global frame_height
    global frame_width
    minAreaRect = cv2.minAreaRect(contour)
    center, size, angle = minAreaRect
    width, height = size
    return (float(width * height)) / (float(frame_width * frame_height))

# The height and width of the bounding box should be roughly the same (since it's a sphere).
def sphereBoundingRatioScore(bounds):
    x,y,w,h = bounds
    return ratioToScore(float(h)/float(w))

# Ideally, the ratio of the area of the target to that of its bounding box should be
# approximately pi / 4.  (Area of a circle is pi * r * r; area of a square is 4*r*r.)
def coverageAreaScore(contour, bounds):
    _,_,w,h = bounds
    return ratioToScore(cv2.contourArea(contour)/(float(w)*float(h)) / (math.pi / 4))

# A trivial additional score (for use with ball detection)
def convexityScore(contour):
    if cv2.isContourConvex(contour):
        return 1
    else:
        return 0

#-----------------------------------------------------------------------------
# Main execution starts here
#-----------------------------------------------------------------------------

# Default colors will depend on what we're looking for....
if portDetection:
    low_H = green_low_H
    high_H = green_high_H
    low_S = green_low_S
    high_S = green_high_S
    low_V = green_low_V
    high_V = green_high_V
else:
    low_H = yellow_low_H
    high_H = yellow_high_H
    low_S = yellow_low_S
    high_S = yellow_high_S
    low_V = yellow_low_V
    high_V = yellow_high_V

# Assume we're using the camera; overridden by command-line flag settings.
useCamera = True

parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
parser.add_argument('--file', help='Still image file to be used.', default="")
# Seed the default values for the HSV ranges, based on the numbers set above.
parser.add_argument('--low_H', help='Initial low value for hue.', default=str(low_H))
parser.add_argument('--low_S', help='Initial low value for saturation.', default=str(low_S))
parser.add_argument('--low_V', help='Initial low value for value.', default=str(low_V))
parser.add_argument('--high_H', help='Initial high value for hue.', default=str(high_H))
parser.add_argument('--high_S', help='Initial high value for saturation.', default=str(high_S))
parser.add_argument('--high_V', help='Initial high value for value.', default=str(high_V))
args = parser.parse_args()

if args.file == "":
    useCamera = True
    cap = cv2.VideoCapture(args.camera)
    print("Using data from camera")
else:
    useCamera = False
    still_image = cv2.imread(args.file)
    print("Using data from file '{}'".format(args.file))

# Convert the program arguments for HSV values (default or otherwise) into the
# values to be applied @ program start.
low_H = int(args.low_H)
high_H = int(args.high_H)
low_S = int(args.low_S)
high_S = int(args.high_S)
low_V = int(args.low_V)
high_V = int(args.high_V)

# Set up the windows and controls for editing the color range (and seeing results).
cv2.namedWindow(window_capture_name)
cv2.namedWindow(window_detection_name)
cv2.namedWindow(window_masking_name)
cv2.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
cv2.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
cv2.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
cv2.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
cv2.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
cv2.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)

# Used for morphological ops below
kernel = np.ones((3, 3), np.uint8)

while True:
    if useCamera:
        ret, frame = cap.read()
        if frame is None:
            break
    else:
        frame = still_image.copy()

    # Get some basic information about the image we've gotten.
    frame_height, frame_width, frame_channels = frame.shape

    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_threshold = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))

    # Perform "opening": erosion (to remove noise), followed by dilation
    # (to remove small holes within larger regions).
    frame_threshold = cv2.morphologyEx(frame_threshold, cv2.MORPH_OPEN, kernel, iterations = 2)
    
    # Find the contours of the possible targets
    contours, _ = cv2.findContours(frame_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    scores = []
    if len(contours) > 0:
        # Generate "fitness scores" for each of the possible targets.
        #
        # Other examples of scoring can be seen at:
        #   https://docs.opencv.org/master/dd/d49/tutorial_py_contour_features.html
        for contour in contours:
            targetScore = 0.0
            if cv2.contourArea(contour) >= 60:
                if portDetection:
                    targetScore = (portBoundingRatioScore(contour)
                                    * percentOfFieldScore(contour)
                                )
                else:
                    # I should really also be figuring out the "roundness" of it, which
                    # might be approximated by checking how close the centroid is to the
                    # center of the bounding area.
                    boundingBox = cv2.boundingRect(contour)
                    targetScore = (sphereBoundingRatioScore(boundingBox)
                                    + coverageAreaScore(contour, boundingBox)
                                    + convexityScore(contour)
                                )
            scores.append(targetScore)

        bestIndex = 0
        for index in range(len(contours)):
            if scores[index] > scores[bestIndex]:
                bestIndex = index
        best = contours[bestIndex]

        cv2.drawContours(frame, contours, -1, (255,0,255), 1)
        cv2.drawContours(frame, contours, bestIndex, (0,0,255), 3)
        center, radius = cv2.minEnclosingCircle(best)
        cv2.circle(frame, (int(center[0]), int(center[1])), 2, (255,255,255), 1)
    
    masked_img = cv2.bitwise_and(frame, frame, mask=frame_threshold)

    cv2.imshow(window_capture_name, frame)
    cv2.imshow(window_detection_name, frame_threshold)
    cv2.imshow(window_masking_name, masked_img)
    
    key = cv2.waitKey(30)
    if key == ord('q') or key == 27:
        break
