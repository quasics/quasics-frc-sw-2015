# A sample program for vision processing in the FRC game "Infinite Recharge",
# which could run on the WPILibPi/FRCVision platform (RasPi hardware).
#
# This program will find the largest contiguous "blob" of a particular color,
# and then publish information about it via NetworkTables, for use in
# tracking it.  It will also publish information about all of the other
# candidate "blobs" that are spotted.

# Import the WPI packages we need (camera server and network tables).
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer, VideoMode
from networktables import NetworkTablesInstance

# Import OpenCV and NumPy.
import cv2
import numpy as np

# Additional imports, for utilities.
import time


# Controls if the script should bring up a NetworkTables server, or if
# we should look for one that's running on a robot.
server = False

# Controls the scoring algorith selection.  If True, we'll simply go
# with the largest target that seems to be of the correct color; if
# False, we'll also factor other information about a target into its
# scoring.
justUseSizeForScoring = False

# Team number (used in connecting to NetworkTables server running on
# a robot).
team = 2656

# Requested size of the video feed directly deployed from the camera.
# (This is apparently independent of the video sizing provided by the
# CameraServer code.)
width = 320
height = 240

# Convenience flag, used to pick between default sets of colors:
# * If True, use a set that worked well in the lab on 13Feb2020.
# * If False, use a set that worked well at Mr. Healy's home (under)
#   different lighting conditions).
inLab = True

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

# Note: Hue values would normally be in the range of 0-359.  However,
# OpenCV limits it to 0-179 (presumably in order to make sure that it
# fits into a single byte), so here's a convenient function for that,
# should we need one.
def scaleHueForOpenCV(h):
    return (h / 2)

###############################################################################
#
# Scoring functions
#
# These are used to calculate "scores" for the possible targets, which may be
# used to determine which target is "best".
#
# In this sample program, we're simply looking for something that looks like
# the closest (yellow) ball/fuel cell.
# 
# As a result, one trivial scoring algorithm would be to simply look at the
# size of the contour for a given target, and assume that "bigger" means
# "better".  However, it should be noted  that this requires pretty fine
# tuning of the color matching; any falsely detected targets (e.g., a wall
# painted yellow) would likely yield a bad result.
#
# Another option is to combine a couple of different scores into a composite
# value, by looking at different aspects of the target in order to see how
# well it matches up to our expectations.
#
# One way to combine them is to simply add them together, but we can still
# have cases where one bad result (e.g., size of that yellow wall) dominates
# the rest.  An alternative is to *multiply* them together, so that something
# that scores really low on one or more dimensions will help to ensure a low
# composite score (e.g., "absolute failure: score is 0" will drive the
# final score to 0, too).
# 
# We can further reduce the risk here by "normalizing" all of the different
# values to a single range, typically [0.0-1.0].  (For size, we might do this
# by taking the apparent area of the target and turning it into a %age of the
# total image area.)
#
# If we were looking for something more sophisticated than "the closest
# yellow ball", such as the photoreflective tape surrounding a goal, etc.,
# we'd want to use a more elaborate scoring algorithm.
#
# For some examples from the 2017 game, see:
# https://docs.wpilib.org/en/stable/docs/software/vision-processing/introduction/2017-vision-examples.html)
#

# Used to quickly evaluate a contour, in order to see if we think that
# it's reasonable to think that it's a valid target.  This is
# intended to help rule out "false detects" that are too high
# (e.g., similarly-colored walls), or badly shaped (since the
# targets we're looking for are spherical, and thus should have
# a *reasonably* square bounding box), etc.
def isLikelyTarget(contour):
    # Ignore small contours that could be because of noise/bad
    # thresholding. (I'm picking "15 pixels" in area as an
    # arbitrary minimum here.)
    if cv2.contourArea(contour) < 15:
        return False

    return True

# Converts a ratio with ideal value of 1 to a score.
def ratioToScore(ratio):
	return (max(0.0, min(100*(1-abs(1-ratio)), 100.0)))

# A trivial scoring function, which just looks at the size of
# the contour ("bigger" == "better").
def computeScoreFromSize(contour, frameWidth, frameHeight):
    return ratioToScore(float(cv2.contourArea(contour)) / float(frameWidth * frameHeight))

# Since the target is a sphere, the aspect ratio of its bounding rect
# (if we can see it all, and clearly) should approach 1:1.  So we'll
# use that to compute a scoring component based on the aspect ratio
# that will be 1.0 if it's perfectly square, and will "decay" towards
# 0 as it becomes more asymmetric.
def computeScoreForAspectRatio(contour):
    x,y,w,h = cv2.boundingRect(contour)
    ratio = 0
    if w < h:
        ratio = float(w) / float(h)
    else:
        ratio = float(h) / float(w)
    return ratioToScore(ratio)

# If the target appears to be above a certain starting height in the frame,
# we'll assume that it's less likely to be a ball on the ground (that's
# near to us).
#
# Note that this assumes the camera is comparatively low on the bot (but
# above the height of a ball resting on the floor), and looking *forward*.
# If it was looking up (e.g, from right above the ground), or the ball is
# right in front of the camera (filling the view) it wouldn't work as well.
def computeScoreForRelativePosition(contour, frameHeight):
    x,y,w,h = cv2.boundingRect(contour)
    # Arbitrary decision: anything above the midline in the frame is less
    # likely to be a ball resting on the floor.
    # Assumes top/left corner is (0,0).
    midLine = frameHeight / 2
    if y > midLine:
        # Below the midline, so just give it "full credit".
        return 1

    # The bounding rect starts above the midline; still allow it through,
    # but at a reduced score, trending to 0 as it hits the top of the frame.
    return ratioToScore(float(y) / float(midLine))

# The composite scoring algorithm, which looks at some characteristics we
# associate with "probably a ball on the ground", plus the apparent sizing
# (to identify something likely to be close).
def computeScore(contour, frameWidth, frameHeight):
    # Skip anything that doesn't look like a likely target.
    if not isLikelyTarget(contour):
        return -100
    
    # Component scores
    sizeScore = computeScoreFromSize(contour, frameWidth, frameHeight)
    positionScore = computeScoreForRelativePosition(contour, frameHeight)
    aspectScore = computeScoreForAspectRatio(contour)
    print("Scoring: size={}, position={}, aspect={}".format(sizeScore, positionScore, aspectScore))

    # Compute the overall score
    return sizeScore * positionScore * aspectScore

#
# End of scoring functions
###############################################################################

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

    dbgBuffer = ""
    start_time = time.time()

    # Tell the CvSink to grab a frame from the camera and put it
    # in the source image.  If there is an error notify the output.
    frame_time, input_img = inputStream.grabFrame(input_img)
    if frame_time == 0:
        # Send the output the error.
        outputStream.notifyError(inputStream.getError())
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

    # Generate a bitmask, looking for pixels in the desired color range.
    #
    # This basically means a version of the image that is effectively 0's
    # and 1's (black and white), with all the places we want to keep data --
    # i.e., where we see something in the color range that we care about --
    # being the 1's, and all of the areas we don't being the "mask" of 0's.
    frame_threshold = cv2.inRange(
        frame_HSV,                  # our source (color) image
        (low_H, low_S, low_V),      # low end of the color range
        (high_H, high_S, high_V))   # high end of the color range

    # Minor quality cleanup, performing some rounds of "erosion" and "dilation".
    #
    # In OpenCV, an "opening" operation is erosion (which removes noise in the
    # image), followed by dilation (which fills in gaps).  These can also be
    # done in the opposite order (called "closing"), and erosion and dilation
    # can also be done by themselves.
    frame_threshold = cv2.morphologyEx(
        frame_threshold, cv2.MORPH_OPEN, kernel, iterations = 3)

    # Now that we've cleaned up our extraction to isolate the areas of the
    # image that we think are the color(s) that we care about, we'll use
    # that to isolate the corresponding full-color version of the captured
    # image, for later display.
    output_img = cv2.bitwise_and(input_img,input_img, mask= frame_threshold)

    # Find the contours of the possible targets (i.e., where we've got
    # continguous blocks of the colors that we care about).
    _, contours, _ = cv2.findContours(
        frame_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Go walking through the possible targets and find the (single)
    # best-scoring one.  (Remember that in this example program, that simply
    # means the biggest possible target that we can see.)
    #
    # We'll also hang onto some basic information about all of the targets
    # (to be published through NetworkTables as debugging information), as
    # well as drawing some highlights for them in the output image.
    all_targets_top_list = []
    all_targets_left_list = []
    all_targets_width_list = []
    all_targets_height_list = []
    all_targets_x_list = []
    all_targets_y_list = []
    index = -1
    bestIndex = -1
    bestScore = 0
    currentScore = 0
    best = None
    for contour in contours:
        index = index + 1
        
        # Calculate score for the current target.
        if justUseSizeForScoring:
            # Skip anything that doesn't look like a likely target.
            if not isLikelyTarget(contour):
                continue
            
            # Just grab the size of the target
            currentScore = cv2.contourArea(contour)
        else:
            currentScore = computeScore(contour, img_width, img_height)

        # Does the current target beat our best-scoring one so far?
        if bestIndex == -1 or currentScore > bestScore:
            best = contour
            bestIndex = index
            bestScore = currentScore

        ########
        # Everything from here on in this loop is "gravy" (assuming that
        # we're just looking for the biggest target), which will be used
        # to give the drivers (or debuggers) some additional information
        # camera is seeing.  It could be left out in code running on the
        # about what the robot in production, if we wanted to tweak the
        # performance of the processing code.

        # Capture information about the current target's bounding rect.
        # Notice that this is different from the "minAreaRect", which
        # considers rotation as well.
        x,y,w,h = cv2.boundingRect(contour)
        all_targets_top_list.append(y)
        all_targets_left_list.append(x)
        all_targets_width_list.append(width)
        all_targets_height_list.append(height)

        # Calculate basic information about the current contour
        minAreaRect = cv2.minAreaRect(contour)
        center, size, angle = minAreaRect
        center = [int(dim) for dim in center] # Convert to int so we can draw
        
        # Calculate the (x,y) coordinates for the current (possible)
        # target's center, expressed as a %age of the distance from the
        # middle of the image to the edges.  (In other words, dead center
        # would be (0,0), while the top-left corner would be (-1, -1), etc.).
        center_x = (center[0] - img_width / 2) / (img_width / 2)
        center_y = (center[1] - img_height / 2) / (img_height / 2)
        
        # Add the center point's coordinates to the "all targets" collection of
        # data we're going to publish below.
        all_targets_x_list.append(center_x)
        all_targets_y_list.append(center_y)

        # Finally, update our output image (which will be sent out to Smart
        # Dashboard at the end of frame processing), by drawing a white
        # outline of the contour on our output image that, and putting a
        # small red circle at its center.
        cv2.drawContours(
            output_img, contours, index,
            color = (255, 255, 255), thickness = 1)
        cv2.circle(
            output_img,
            (int(center[0]), int(center[1])),   # Center of the circle
            2,                                  # Radius of the circle
            (0,0,255), 1)                       # Color and thickness
    
    # Compute data to be published for the best-scoring target, and highlight
    # it in the output image.
    target_x = []
    target_y = []
    if best is not None:
        # Compute the "center of mass" for the target, expressed as a %age
        # of the distance from the middle of the image to the edges.
        # 
        # In other words, dead center of our "field of vision" would be
        # (0,0), while the top-left corner would be (-1, -1), etc.
        x,y,w,h = cv2.boundingRect(best)
        target_center_x = x + (w / 2)
        target_center_y = y + (h / 2)
        target_x.append((target_center_x - (img_width / 2)) / (img_width / 2))
        target_y.append((target_center_y - (img_height / 2)) / (img_height / 2))

        # Save some debugging data for the target, to be published below.
        dbgBuffer = "Target: x,y = {0},{1}, width/height = {2},{3}".format(x, y, w, h)

        # Draw a red outline of the target on the output image.
        cv2.drawContours(
            output_img, contours, bestIndex,
            color = (0, 0, 255), thickness = 3)

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

    processing_time = time.time() - start_time
    fps = 1 / processing_time
    # cv2.putText(output_img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))

    # (optional, but highly useful) send the modified image back to the dashboard.
    outputStream.putFrame(output_img)

# This function will be invoked whenever NetworkTables says that there's a
# change to the color settings we're monitoring from the driver's station.
#
# This will allow us to tune the color sensitivity to reflect environmental
# conditions, etc.; it could also be used to completely change the color
# that we're looking for (e.g., for different items in different parts of
# the game)).
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

    # # Doesn't work
    # camera = UsbCamera("Camera 0", "/dev/video0")
    # if not camera.setVideoMode(VideoMode.PixelFormat.kBGR, width, height, 30):
    #     printf("Failed to configure video mode")
    # mjpegServer = cs.startAutomaticCapture(camera=camera)

    # # Doesn't work
    # videoSource = cs.startAutomaticCapture(return_server=False)
    # if not videoSource.setVideoMode(VideoMode.PixelFormat.kBGR, width, height, 30):
    #     printf("Failed to configure video mode")

    # Works, in that video data is made available.  (But doesn't set resolution.)
    mjpegServer = cs.startAutomaticCapture()
    if not mjpegServer.setResolution(width, height):
        print("Failed to set resolution={}x{}".format(width, height))


    if not vision_nt.putNumber("camera_width", width):
        print("Failed to publish camera_width")
    if not vision_nt.putNumber("camera_height", height):
        print("Failed to publish camera_height")

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
