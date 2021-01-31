from __future__ import print_function
import cv2
import numpy as np
import argparse

max_value = 255
max_value_H = 360//2
low_H = 19 # 0
low_S = 55 # 0
low_V = 104 # 0
high_H = 45 # max_value_H
high_S = 179 # max_value
high_V = 255 # max_value
window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'
def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv2.setTrackbarPos(low_H_name, window_detection_name, low_H)
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv2.setTrackbarPos(high_H_name, window_detection_name, high_H)
def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv2.setTrackbarPos(low_S_name, window_detection_name, low_S)
def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv2.setTrackbarPos(high_S_name, window_detection_name, high_S)
def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv2.setTrackbarPos(low_V_name, window_detection_name, low_V)
def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv2.setTrackbarPos(high_V_name, window_detection_name, high_V)

parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
args = parser.parse_args()
cap = cv2.VideoCapture(args.camera)

cv2.namedWindow(window_capture_name)
cv2.namedWindow(window_detection_name)
cv2.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
cv2.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
cv2.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
cv2.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
cv2.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
cv2.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)

# Used for morphological ops below
kernel = np.ones((3, 3), np.uint8)

while True:
    
    ret, frame = cap.read()
    if frame is None:
        break
    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_threshold = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))

    # Perform erosion, to remove noise from the background
    # frame_threshold = cv2.erode(frame_threshold, kernel, iterations = 1)
    
    # Perform dilation, to remove small holes inside a larger region
    # frame_threshold = cv2.dilate(frame_threshold, kernel, iterations = 1)

    # "Opening" is erosion, followed by dilation
    frame_threshold = cv2.morphologyEx(frame_threshold, cv2.MORPH_OPEN, kernel, iterations = 2)

    # "Closing" is dilation, followed by erosion
    # frame_threshold = cv2.morphologyEx(frame_threshold, cv2.MORPH_CLOSE, kernel)
    
    # Find the contours of the possible targets
    contours, _ = cv2.findContours(frame_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) > 0:
        largestIndex = 0
        largest = contours[0]
        for index in range(len(contours)):
            current = contours[index]
            if cv2.contourArea(current) > cv2.contourArea(largest):
                largestIndex = index
                largest = current

        cv2.drawContours(frame, contours, -1, (0,255,0), 1)
        cv2.drawContours(frame, contours, largestIndex, (0,0,255), 3)
    
    cv2.imshow(window_capture_name, frame)
    cv2.imshow(window_detection_name, frame_threshold)
    
    key = cv2.waitKey(30)
    if key == ord('q') or key == 27:
        break
