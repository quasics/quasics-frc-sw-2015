import argparse
import cv2 as cv2
import numpy as np

useCamera = True


# Focal length for the webcam, measured in pixels.
# (Logitech c270: 1430 (reported); c920: 1252 (calculated)).
# (169.7 for Lifecam)
focal_length = 1252.0

# Known width of the target, measured in cm.
# (Ball size in cm, based on ~7in posted measurement.)
known_width = 17.78

# The following values define a yellow(ish) color range,
# matching the power cells from Infinite Recharge (at least
# when used during my testing, under prevailing light; YMMV).
yellow_low_H = 22
yellow_low_S = 50
yellow_low_V = 10
yellow_high_H = 30
yellow_high_S = 255
yellow_high_V = 255

# Used for morphological ops during frame processing.
kernel = np.ones((3, 3), np.uint8)

# Computes the distance to an object with a known width (e.g., 
# one of the power cells from Infinite Recharge, cargo from
# Destination: Deep Space, etc.), given a previously-calculated
# focal length for the camera, and its perceived width (in
# pixels) in the camera's field of vision.  (Note: known and
# perceived heights would be fine as an alternative, too.)
#
# From https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
def distanceToCamera(knownWidth, focalLength, perWidth):
	return (knownWidth * focalLength) / perWidth

# A trivial scoring function, which just looks at the size of
# the contour ("bigger" == "better").  If we were looking for
# something more sophisticated, such as the photoreflective
# tape surrounding a goal, etc., we'd want to use a more
# elaborate scoring algorithm.  (See the published examples
# from the 2017 game, for instance.)
def computeScore(contour):
    return cv2.contourArea(contour)

def processFrame(frame, lowColorRange, highColorRange):
    global kernel

    # Convert colors from BGR to HSV, to make it easier to work with.
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # "Threshold" the HSV image to identify the pixels in the targeted color range.
    mask = cv2.inRange(hsv, lowColorRange, highColorRange)

    # Perform a few rounds of "dilation" (removing small holes inside a larger region)
    # and "erosion" (removing noise from the background).
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations = 3)

    # Now that we've cleaned up our extraction to isolate the areas of the image that we
    # think are the color(s) that we care about, we'll use that to isolate the corresponding
    # full-color version of the captured image, for later display.
    output_img = cv2.bitwise_and(frame, frame, mask=mask)

    # Find the contours of the possible targets (i.e., where we've got continguous blocks
    # of the colors that we care about).
    #
    # Note: in FRCVision's version of CV2, this would be "_, contours, _ = ...."
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Walk through the contours and find the best (biggest) possible target.
    index = -1
    bestIndex = -1
    best = None
    for contour in contours:
        index = index + 1

        # Ignore small contours that could be because of noise/bad thresholding.
        # (I'm picking "15 pixels" in area as an arbitrary minimum here.)
        if cv2.contourArea(contour) < 15:
            continue
        
        # Does the current image beat our best-scoring one so far?
        if bestIndex == -1 or computeScore(contour) > computeScore(best):
            best = contour
            bestIndex = index

        ########
        # Everything from here on in this loop is "gravy" (assuming that
        # we're just looking for the biggest target), which will be used
        # to give the drivers (or debuggers) some additional information
        # camera is seeing.  It could be left out in code running on the
        # about what the robot in production, if we wanted to tweak the
        # performance of the processing code.

        # Calculate basic information about the current contour
        rect = cv2.minAreaRect(contour)
        center, size, angle = rect
        center = [int(dim) for dim in center] # Convert to int so we can draw

        # Finally, update our output image, by drawing a white outline of the contour
        # on our output image that, and putting a small red circle at its center.
        cv2.drawContours(output_img, contours, index, color = (255, 255, 255), thickness = 1)
        cv2.circle(output_img, (int(center[0]), int(center[1])), 2, (0,0,255), 1)

    if best is not None:
        center, size, angle = cv2.minAreaRect(best)
        width, height = size
        # Draw a red outline of the target on the output image.
        cv2.drawContours(output_img, contours, bestIndex, color = (0, 0, 255), thickness = 3)
        dist = distanceToCamera(knownWidth = known_width, focalLength = focal_length, perWidth = width)
        print("Distance: {} cm, perceived width: {} pixels".format(dist, width))

    # Display the various versions of stuff.
    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',output_img)

#########################################
# "Main" code starts here


parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
parser.add_argument('--file', help='Still image file to be used.', default="")
args = parser.parse_args()

if args.file == "":
    useCamera = True
    cap = cv2.VideoCapture(args.camera)
    print("Using data from camera")
else:
    useCamera = False
    still_image = cv2.imread(args.file)
    print("Using data from file '{}'".format(args.file))

if useCamera:
    # Source for image captures.
    cap = cv2.VideoCapture(0)
    while(1):
        ret, frame = cap.read()
        if frame is None:
            break

        # Process it
        processFrame(
            frame,
            np.array([yellow_low_H,yellow_low_S,yellow_low_V]),    # Low end of color range
            np.array([yellow_high_H,yellow_high_S,yellow_high_V])) # High end of color range

        k = cv2.waitKey(5)
        if k == 27 or k == ord('q'):
            break

else:
    frame = still_image.copy()

    # Process it
    processFrame(
        frame,
        np.array([yellow_low_H,yellow_low_S,yellow_low_V]),    # Low end of color range
        np.array([yellow_high_H,yellow_high_S,yellow_high_V])) # High end of color range

    while(1):
        k = cv2.waitKey(5)
        if k == 27 or k == ord('q'):
            break

cv2.destroyAllWindows()
