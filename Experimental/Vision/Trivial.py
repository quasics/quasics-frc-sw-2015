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
input_img = None    # Preallocated buffer for frame data
kernel = None       # Preallocated kernel for transforms
vision_nt = None

def processFrame(inputStream, outputStream, input_img, vision_nt):
        start_time = time.time()

        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        frame_time, input_img = inputStream.grabFrame(input_img)
        if frame_time == 0:
            # Send the output the error.
            outputStream.notifyError(inputStream.getError());
            # skip the rest of the current iteration
            return

        output_img = np.copy(input_img)

        # Generate a bitmask, looking for pixels in the desired color range
        frame_HSV = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
        frame_threshold = cv2.inRange(frame_HSV, (19, 55, 104), (45, 179, 255))

        # Minor quality cleanup: "opening" is erosion (remove noise), followed by dilation (fill gaps)
        frame_threshold = cv2.morphologyEx(frame_threshold, cv2.MORPH_OPEN, kernel, iterations = 2)

        # Find the contours of the possible targets
        _, contours, _ = cv2.findContours(frame_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # if len(contours) > 0:
            # print("Found", len(contours), "targets")
            # largestIndex = 0
            # largest = contours[0]
            # for index in range(len(contours)):
                # current = contours[index]
                # if cv2.contourArea(current) > cv2.contourArea(largest):
                    # largestIndex = index
                    # largest = current
            # rect = cv2.minAreaRect(largest)
            # print("Largest:", rect)

        x_list = []
        y_list = []
        index = 0
        for contour in contours:
           # Ignore small contours that could be because of noise/bad thresholding
           if cv2.contourArea(contour) < 15:
              continue

           rect = cv2.minAreaRect(contour)
           center, size, angle = rect
           center = [int(dim) for dim in center] # Convert to int so we can draw
           
           # Add the (x,y) coordinates for the target center to the lists we're publishing
           center_x = (center[0] - width / 2) / (width / 2)
           center_y = (center[1] - height / 2) / (height / 2)
           x_list.append(center_x)
           y_list.append(center_y)

           # Draw contour, bounding rectangle, and circle at center.
           cv2.drawContours(output_img, contours, index, color = (255, 255, 255), thickness = -1)
           # cv2.drawContours(output_img, np.int0(cv2.boxPoints(rect)), -1, color = (0, 0, 255), thickness = 2)
           # cv2.circle(output_img, center = center, radius = 3, color = (0, 0, 255), thickness = -1)
         
           index = index + 1
           
        print("Contours:", len(contours), " Published:", len(x_list))

        # Publish center data for the targets we found.
        if not vision_nt.putNumberArray('target_x', x_list):
            print("Failed to publish target_x")
        if not vision_nt.putNumberArray('target_y', y_list):
            print("Failed to publish y_list")

        processing_time = time.time() - start_time
        fps = 1 / processing_time
        # cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
        cv2.line(output_img, (0, 0), (120, 160), (255, 0, 0), 3)

        # (optional) send the modified image back to the dashboard
        outputStream.putFrame(output_img)

def main():
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

    # Get a CvSink. This will capture images from the camera
    cvSink = cs.getVideo()

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    outputStream = cs.putVideo("Name", width, height)

    # Allocating new images is very expensive, always try to preallocate
    input_img = np.zeros(shape=(width, height, 3), dtype=np.uint8)

    # Used for morphological ops below
    kernel = np.ones((3, 3), np.uint8)

    print("Running....")
    while True:
        processFrame(cvSink, outputStream, input_img, vision_nt)

main()
