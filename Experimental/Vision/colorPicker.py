import cv2
def Life2CodingRGB(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE :  # checks mouse moves
        colorsBGR = image[y, x]
        colorsHSV = hsv_image[y, x]
        colorsRGB=tuple(reversed(colorsBGR)) #Reversing the OpenCV BGR format to RGB format
        print("RGB Value at ({},{}): RGB={} HSV={}".format(x,y,colorsRGB,colorsHSV))

# Read an image
image = cv2.imread("Power Ball.jpg")
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Create a window and set Mousecallback to a function for that window
cv2.namedWindow('Color Picker')
cv2.setMouseCallback('Color Picker', Life2CodingRGB)
# Do until esc pressed
while (1):
    cv2.imshow('Color Picker', image)
    k = cv2.waitKey(10) 
    if k == ord('q') or k == 27:
        break
# if esc is pressed, close all windows.
cv2.destroyAllWindows()
