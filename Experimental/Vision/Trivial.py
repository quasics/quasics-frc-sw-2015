# Import the camera server
from cscore import CameraServer

# Import OpenCV and NumPy
import cv2
import numpy as np
import time

from networktables import NetworkTablesInstance

server = False
team = 2656
width = 320
height = 240

def scaleHueForOpenCV(h):
    return (h / 2)

# Note: converting BGR colors to HSV is pretty straightforward:
#   >>> green = np.uint8([[[0,255,0 ]]])
#   >>> hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
#   >>> print( hsv_green )
#   [[[ 60 255 255]]]
# So, coming up with better values than the range below should hopefully
# be fairly simple.

# Note: openCV uses a constrained range of 0..179 for H, and 0..255 for S/V
# So we need to normalize accordingly.
raw_low_H = 5  # out of 360

low_H = 21 # scaleHueForOpenCV(raw_low_H)
low_S = 160
low_V = 136
high_H = 30
high_S = 250
high_V = 255


input_img = None    # Preallocated buffer for frame data
kernel = None       # Preallocated kernel for transforms
vision_nt = None

def processFrame(inputStream, outputStream):
    global low_H
    global high_H
    global low_S
    global high_S
    global low_V
    global high_V
    global vision_nt
    global input_img

    dbgBuffer = ""
    start_time = time.time()

    # Tell the CvSink to grab a frame from the camera and put it
    # in the source image.  If there is an error notify the output.
    frame_time, input_img = inputStream.grabFrame(input_img)
    if frame_time == 0:
        # Send the output the error.
        outputStream.notifyError(inputStream.getError());
        # skip the rest of the current iteration
        return

    img_height, img_width, img_channels = input_img.shape
    output_img = np.copy(input_img)

    # Generate a bitmask, looking for pixels in the desired color range
    frame_HSV = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
    frame_threshold = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
    output_img = cv2.bitwise_and(input_img,input_img, mask= frame_threshold)

    # Minor quality cleanup: "opening" is erosion (remove noise), followed by dilation (fill gaps)
    frame_threshold = cv2.morphologyEx(frame_threshold, cv2.MORPH_OPEN, kernel, iterations = 2)

    # Find the contours of the possible targets
    _, contours, _ = cv2.findContours(frame_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    all_targets_x_list = []
    all_targets_y_list = []
    index = 0
    largestIndex = -1
    largest = None
    for contour in contours:
        # Ignore small contours that could be because of noise/bad thresholding
        if cv2.contourArea(contour) < 15:
            continue
        if largestIndex == -1 or cv2.contourArea(contour) > cv2.contourArea(largest):
            largest = contour
            largestIndex = index

        rect = cv2.minAreaRect(contour)
        center, size, angle = rect
        center = [int(dim) for dim in center] # Convert to int so we can draw
        
        # Add the (x,y) coordinates for the target center to the "all targets" data we're publishing
        # TODO: Consider hanging onto the center identified here for the largest one, rather than the
        #       bpunding rect
        center_x = (center[0] - img_width / 2) / (img_width / 2)
        center_y = (center[1] - img_height / 2) / (img_height / 2)
        all_targets_x_list.append(center_x)
        all_targets_y_list.append(center_y)

        # Draw contour, bounding rectangle, and circle at center.
        cv2.drawContours(output_img, contours, index, color = (255, 255, 255), thickness = 3)
        # cv2.drawContours(output_img, np.int0(cv2.boxPoints(rect)), -1, color = (0, 0, 255), thickness = 2)
        # cv2.circle(output_img, center = center, radius = 3, color = (0, 0, 255), thickness = -1)
        
        index = index + 1
    
    # Explicit targeting for the largest
    target_x = []
    target_y = []
    if largest is not None:
        x,y,w,h = cv2.boundingRect(largest)
        dbgBuffer = "Target: x,y = " + str(x) + "," + str(y) + ", width/height = " + str(w) + "," + str(h)
        target_center_x = x + (w / 2)
        target_center_y = y + (h / 2)
        target_x.append((target_center_x - (img_width / 2)) / (img_width / 2))
        target_y.append((target_center_y - (img_height / 2)) / (img_height / 2))

    # Publish core debugging info
    vision_nt.putNumber("img_width", img_width)
    vision_nt.putNumber("img_height", img_height)
    
    print("Contours:", len(contours), " Published:", len(all_targets_x_list))
    if largest is not None:
        x,y,w,h = cv2.boundingRect(largest)
        dbgBuffer = "Bounds: x,y = " + str(x) + "," + str(y) + ", width/height = " + str(w) + "," + str(h)
    vision_nt.putString("dbg", dbgBuffer)

    # Publish center data for the largest of the targets we found.
    if not vision_nt.putNumberArray('target_x', target_x):
        print("Failed to publish target_x")
    if not vision_nt.putNumberArray('target_y', target_y):
        print("Failed to publish target_y")

    # Publish center data for all of the targets we found.
    if not vision_nt.putNumberArray('x_list', all_targets_x_list):
        print("Failed to publish x_list")
    if not vision_nt.putNumberArray('y_list', all_targets_y_list):
        print("Failed to publish y_list")

    processing_time = time.time() - start_time
    fps = 1 / processing_time
    # cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
    # cv2.line(output_img, (0, 0), (120, 160), (255, 0, 0), 3)

    # (optional) send the modified image back to the dashboard
    outputStream.putFrame(output_img)

def main():
    global vision_nt
    global input_img
    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)
        ntinst.startDSClient()

    # Table for vision output information
    vision_nt = ntinst.getTable('Vision')
    print("Got 'Vision' table")

    cs = CameraServer.getInstance()
    cs.enableLogging()

    # Capture from the first USB Camera on the system
    camera = cs.startAutomaticCapture()
    camera.setResolution(width, height)
    if not vision_nt.putNumber("frame_width", width):
        printf("Failed to publish frame_width")
    if not vision_nt.putNumber("frame_height", height):
        printf("Failed to publish frame_height")

    # Get a CvSink. This will capture images from the camera
    cvSink = cs.getVideo()

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    outputStream = cs.putVideo("Scribbled", width, height)

    # Allocating new images is very expensive, always try to preallocate
    input_img = np.zeros(shape=(width, height, 3), dtype=np.uint8)

    # Used for morphological ops below
    kernel = np.ones((3, 3), np.uint8)

    print("Running....")
    while True:
        processFrame(cvSink, outputStream)
        time.sleep(0.1)

main()
