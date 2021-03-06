# Sample program, finding the largest contiguous "blob" of a
# particular color, and then publishing information about it
# via NetworkTables, for use in tracking it.

# Import the WPI packages we need (camera server and network tables).
from cscore import CameraServer
from networktables import NetworkTablesInstance

# Import OpenCV and NumPy.
import cv2
import numpy as np

# Additional imports, for utilities.
import time


# Controls if the script should bring up a NetworkTables server, or if
# we should look for one that's running on a robot.
server = False

# Team number (used in connecting to NetworkTables server running on
# a robot).
team = 2656

# Requested size of the video feed directly deployed from the camera.
# (This is apparently independent of the video sizing provided by the
# CameraServer code, which *appears* to be limited to 160x120.)
width = 160
height = 120

#Focal Length definition, and the known width definition
focal_length = 169.7
known_width = 17.78

#calcualation for the distance
def distanceToCamera(known_width, focal_length, perceivedWidth):
    # compute and return the distance 
    return (known_width * focal_length)/ perceivedWidth

# Convenience flag, used to pick between default sets of colors:
# * If True, use a set that worked well in the lab on 13Feb2020.
# * If False, use a set that worked well at Mr. Healy's home (under)
#   different lighting conditions).
inLab = True

# Note: Hue values would normally be in the range of 0-359.  However,
# OpenCV limits it to 0-179 (presumably in order to make sure that it
# fits into a single byte), so here's a convenient function for that,
# should we need one.
def scaleHueForOpenCV(h):
    return (h / 2)

# Note: converting BGR colors to HSV is pretty straightforward:
#   >>> green = np.uint8([[[0,255,0 ]]])
#   >>> hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
#   >>> print( hsv_green )
#   [[[ 60 255 255]]]
# So coming up with better values for the ranges below should hopefully
# be fairly simple, should we elect to start with RGB/BGR values.

# Define some default color ranges, absent overriding data via
# NetworkTables (handled below, in valueChanged()).
if inLab:
    # Color gamut needed in a nice, bright space at the lab
    low_H = 22
    low_S = 80
    low_V = 80
    high_H = 30
    high_S = 255
    high_V = 255
else:
    # Color gamut during Matt's testing at home
    low_H = 26
    low_S = 80
    low_V = 80
    high_H = 37
    high_S = 255
    high_V = 255

input_img = None    # Preallocated buffer for frame data
kernel = None       # Preallocated kernel for transforms
vision_nt = None    # Pre-fetched connection to Network Tables
                    # for publishing vision results.

# A trivial scoring function, which just looks at the size of
# the contour ("bigger" == "better").  If we were looking for
# something more sophisticated, such as the photoreflective
# tape surrounding a goal, etc., we'd want to use a more
# elaborate scoring algorithm.  (See the published examples
# from the 2017 game, for instance.)
def computeScore(contour):
    return cv2.contourArea(contour)
 
# Defining the function carried over from Identification Points

def identifyPathForChallenge(left_list, width_list, distance_list):

    global vision_nt

    pathId = -1

    numBalls = len(width_list)

    # Find the largest ball: assumption is that it should be right in front of us
    index = -1 
    bestIndex = -1 
    for width in width_list:
        index = index + 1

        # Determines if the current width beats the "best" width curretly. 

        if bestIndex == -1 or width_list[index] > width_list[bestIndex]: 
            bestIndex = index

    # Figure out which path is set up, based on positions of the balls
    # relative to the largest one.  (And how many we can "see".)
    if numBalls == 3:
        # Must be Path A.  Just need to figure out which route within it.
        if bestIndex == 0:
            otherindx1 = 1
            otherindx2 = 2

        elif bestIndex == 1:
            otherindx1 = 0
            otherindx2 = 2

        elif bestIndex == 2:
            otherindx1 = 0 
            otherindx2 = 1

        if left_list[otherindx1] < left_list[bestIndex] and left_list[otherindx2] < left_list[bestIndex]:
           pathId = 2

        elif left_list[otherindx1] > left_list[bestIndex] and left_list[otherindx2] > left_list[bestIndex]:
            pathId = -1

        else:
            pathId = 1

    elif numBalls == 2:
        # Must be path B
        if bestIndex == 0:
            otherindx = 1
        else:
            otherindx = 0

        if left_list[otherindx] > left_list[bestIndex]:
            pathId = 3
        else:
            pathId = 4

    elif numBalls == 1:
        # Something happened in  that the camera is too small to pick up the second ball visible. 
        # This must be  Path B.

        if distance_list[bestIndex] < 8 and distance_list[bestIndex] > 6:
            pathId = 3
        elif distance_list[bestIndex] < 16 and distance_list[bestIndex] > 14:
            pathId = 4
        else:
            pathId = -1 

    else:
        # Something weird, can't determine A or B
        pathId = -1
        
    vision_nt.putNumber('path_id', pathId)

# Processes the next frame of video from the CameraServer.
def processFrame(inputStream, outputStream):
    global low_H
    global high_H
    global low_S
    global high_S
    global low_V
    global high_V
    global vision_nt
    global input_img
    global focal_length
    global known_width

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

    # Make a copy of the source image, to use in sharing an annotated
    # set of results.
    output_img = np.copy(input_img)

    # Get some basic information about the image we've gotten.
    img_height, img_width, img_channels = input_img.shape

    # Convert the image to the HSV colorspace, which is what OpenCV works
    # with most efficiently.
    frame_HSV = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)

    # Generate a bitmask, looking for pixels in the desired color range.  (This basically
    # means a version of the image that is effectively black and white, with all the places
    # we want to keep data being white, and all of the areas we don't being the "mask" of
    # black data.)
    frame_threshold = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))

    # Minor quality cleanup, performing some rounds of "erosion" and "dilation".
    #
    # In OpenCV, an "opening" operation is erosion (which removes noise in the image),
    # followed by dilation (which fills in gaps).  These can also be done in the opposite
    # order (called "closing"), and erosion and dilation can also be done by themselves.
    frame_threshold = cv2.morphologyEx(frame_threshold, cv2.MORPH_OPEN, kernel, iterations = 3)

    # Now that we've cleaned up our extraction to isolate the areas of the image that we
    # think are the color(s) that we care about, we'll use that to isolate the corresponding
    # full-color version of the captured image, for later display.
    output_img = cv2.bitwise_and(input_img,input_img, mask= frame_threshold)

    # Find the contours of the possible targets (i.e., where we've got continguous blocks
    # of the colors that we care about).
    _, contours, _ = cv2.findContours(frame_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Go walking through the possible targets and find the best-scoring one.
    #
    # We'll also hang onto some basic information about all of the targets (to be
    # published through NetworkTables as debugging information), as well as drawing
    # some highlights for them in the output image.
    all_targets_x_list = []
    all_targets_y_list = []
    all_targets_top_list = []
    all_targets_left_list = []
    all_targets_width_list = []
    all_targets_height_list = []
    all_targets_distance_list = []
    index = -1
    bestIndex = -1
    best = None
    for contour in contours:
        index = index + 1

        # Ignore small contours that could be because of noise/bad thresholding
        if cv2.contourArea(contour) < 15:
            continue
        
        # Does the current image beat our best-scoring one so far?
        if bestIndex == -1 or computeScore(contour) > computeScore(best):
            best = contour
            bestIndex = index


        ########
        # Everything from here on in this loop is "gravy", used to give the
        # drivers (or debuggers) some additional information about what the
        # camera is seeing.  It could be left out in code running on the
        # robot in production, if we wanted to tweak the performance of the
        # processing code.
        #

        # Calculate basic information about the current contour
        rect = cv2.minAreaRect(contour)
        center, size, angle = rect
        center = [int(dim) for dim in center] # Convert to int so we can draw
        
        # Add the (x,y) coordinates for the target center to the "all targets"
        # data we're publishing, expressed as a %age of the distance from the
        # middle of the image to the edges.  (In other words, dead center would
        # be (0,0), while the top-left corner would be (-1, -1), etc.)
        center_x = (center[0] - img_width / 2) / (img_width / 2)
        center_y = (center[1] - img_height / 2) / (img_height / 2)
        
        x,y,w,h = cv2.boundingRect(contour)
        
        all_targets_x_list.append(center_x)
        all_targets_y_list.append(center_y)
        all_targets_top_list.append(y)
        all_targets_left_list.append(x)
        all_targets_width_list.append(w)
        all_targets_height_list.append(h)
        d = distanceToCamera(known_width, focal_length, w)
        all_targets_distance_list.append(d)

        # Draw a white outline of the contour, and put a small red circle at its center.
        cv2.drawContours(output_img, contours, index, color = (255, 255, 255), thickness = 1)
        cv2.circle(output_img, (int(center[0]), int(center[1])), 2, (0,0,255), 1)
    
    identifyPathForChallenge(all_targets_left_list, all_targets_width_list, all_targets_distance_list)
    
    # Compute data to be published for the best-scoring target, and highlight it
    # in the output image.
    target_x = []
    target_y = []
    if best is not None:
        # Compute the "center of mass" for the target.
        x,y,w,h = cv2.boundingRect(best)
        target_center_x = x + (w / 2)
        target_center_y = y + (h / 2)
        target_x.append((target_center_x - (img_width / 2)) / (img_width / 2))
        target_y.append((target_center_y - (img_height / 2)) / (img_height / 2))

        # Save some debugging data for the target, to be published below.
        dbgBuffer = "Target: x,y = " + str(x) + "," + str(y) + ", width/height = " + str(w) + "," + str(h)

        # Draw a red outline of the target on the output image.
        cv2.drawContours(output_img, contours, bestIndex, color = (0, 0, 255), thickness = 3)

    # Publish center data for the best of the targets we found.
    if not vision_nt.putNumberArray('target_x', target_x):
        print("Failed to publish target_x")
    if not vision_nt.putNumberArray('target_y', target_y):
        print("Failed to publish target_y")

    # Publish debugging information
    vision_nt.putString("dbg", dbgBuffer)
    vision_nt.putNumber("img_width", img_width)
    vision_nt.putNumber("img_height", img_height)
    vision_nt.putNumberArray('x_list', all_targets_x_list)
    vision_nt.putNumberArray('y_list', all_targets_y_list)
    vision_nt.putNumberArray('top_list', all_targets_top_list)
    vision_nt.putNumberArray('left_list', all_targets_left_list)
    vision_nt.putNumberArray('width_list', all_targets_width_list)
    vision_nt.putNumberArray('height_list', all_targets_height_list)
    vision_nt.putNumberArray('distance_list', all_targets_distance_list)


    processing_time = time.time() - start_time
    fps = 1 / processing_time
    # cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))

    # (optional, but highly useful) send the modified image back to the dashboard.
    outputStream.putFrame(output_img)

# This function will be invoked whenever NetworkTables says that there's a change to
# the color settings we're monitoring from the driver's station.  (This will allow us
# to tune the color sensitivity to reflect environmental conditions, etc.; it could
# also be used to completely change the color that we're looking for.)
def valueChanged(table, key, value, isNew):
    global low_H
    global high_H
    global low_S
    global high_S
    global low_V
    global high_V
    # print("valueChanged: key: '%s'; value: %s; isNew: %s" % (key, value, isNew))
    if key == "Low_H":
        low_H = int(value)
    if key == "High_H":
        high_H = int(value)
    if key == "Low_S":
        low_S = int(value)
    if key == "High_S":
        high_S = int(value)
    if key == "Low_V":
        low_V = int(value)
    if key == "High_V":
        high_V = int(value)
    print("Gamut is: H - %s/%s; S - %s/%s; V - %s/%s" % (low_H, high_H, low_S, high_S, low_V, high_V))

# Main function, which will set things up and then start running an infinite loop
# to process image data from the default camera.
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

    # Get a connection to the table we'll use to publish targeting information
    # from the processFrame() function, above.
    vision_nt = ntinst.getTable('Vision')

    # Listen to changes in the table used to configure vision-processing.
    ntinst.getTable('Shuffleboard/VisionSettings').addEntryListener(valueChanged)

    # Capture from the first USB Camera on the system
    print("Setting up camera")
    cs = CameraServer.getInstance()
    cs.enableLogging()
    camera = cs.startAutomaticCapture()
    camera.setResolution(width, height)
    if not vision_nt.putNumber("camera_width", width):
        printf("Failed to publish camera_width")
    if not vision_nt.putNumber("camera_height", height):
        printf("Failed to publish camera_height")
   

    # Get a CvSink. This will capture images from the camera, for use when processing
    # the frames.
    cvSink = cs.getVideo()

    # (optional) Set up a CvSource. This will be used to send processed/annotated
    # images back to the Dashboard.
    outputStream = cs.putVideo("Annotated", width, height)

    # Allocating new images is very expensive: always try to preallocate the space
    # that will be used for them, rather than doing this every time you process a
    # frame from the camera.
    input_img = np.zeros(shape=(width, height, 3), dtype=np.uint8)

    # Used for morphological ops during frame processing.  (This is pretty small,
    # and thus could conceivably be done for every frame, but it's always going to
    # be the same data, and thus we might as well pre-allocate it, too.)
    kernel = np.ones((3, 3), np.uint8)

    # Start the primary loop for the program.
    print("Running....")
    while True:
        processFrame(cvSink, outputStream)

        # If the following line is enabled, it will inject a brief pause
        # between frames, which can be tweaked as needed.
        #
        # We may want to be careful how far we send it in one direction or the
        # other.  (Too long, and the processing will be so laggy that it's not
        # helpful; too short, and we might waste bandwidth by reprocessing images
        # that haven't changed.)
        time.sleep(0.05)    # Note: the delay is expressed in seconds, and 0.005
                            # yields about 14-15 FPS (0.35Mbps) from a RasPi 3
                            # running the Romi image and in the "disabled" robot
                            # state, when testing on a lightly-loaded network.
main()


