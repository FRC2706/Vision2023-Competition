#THIS FILE CONSISTS OF VISION CONSTANTS, EXPECTED TO BE USED EVERY YEAR
import math
import numpy as np

# Field of View (FOV) of the microsoft camera (68.5 is camera spec)
# Lifecam 3000 from datasheet
# Datasheet: https://dl2jx7zfbtwvr.cloudfront.net/specsheets/WEBC1010.pdf
diagonalView = math.radians(68.5)

# The ELP-USBFHD01M-L21 (Wide angle is 170 degrees)
# Datasheet:  http://www.webcamerausb.com/elp-high-speed-120fps-pcb-usb20-webcam-board-2-mega-pixels-1080p-ov2710-cmos-camera-module-with-21mm-lens-elpusbfhd01ml21-p-78.html
# diagonalView = math.radians(170)

# MAY CHANGE IN FUTURE YEARS! It is better to use dynamic calculations as specified in the function
# below.   That will allow multiple cameras with different resolutions and FOV parameters to be used
# from the same pipeline code.

image_width = 1280 # 16  
image_height = 720 # 9 

#To calculate the aspect ratio, first find the greatest common divisor between the
# #image height and image width of the camera
resolution_gcd = math.gcd(image_width, image_height)

#The horizontal aspect is simply the imagewidth divided by the gcd
#for examle, a 640x480 should give a GCD of 160, and aspect ratio is 4:3
horizontalAspect = (image_width/resolution_gcd)

#The vertical Aspect ratio is the image height divided by the gcd
verticalAspect = (image_height/resolution_gcd)

# Reasons for using diagonal aspect is to calculate horizontal field of view.
diagonalAspect = math.hypot(horizontalAspect, verticalAspect)
# Calculations: http://vrguy.blogspot.com/2013/04/converting-diagonal-field-of-view-and.html
horizontalView = math.atan(math.tan(diagonalView / 2) * (horizontalAspect / diagonalAspect)) * 2
verticalView = math.atan(math.tan(diagonalView / 2) * (verticalAspect / diagonalAspect)) * 2

H_FOCAL_LENGTH = image_width / (2 * math.tan((horizontalView / 2)))
V_FOCAL_LENGTH = image_height / (2 * math.tan((verticalView / 2)))


#calculate the Horizontal and Vertical Camera Focal Lengths for this camera and resolution
#input cameraFOV is the manufacturer of the cameras advertised field of view.   For example, the
#                microsoft lifecam has a Fielf of View (FOV) of 68.5 degrees
#      image_width is the image width in pixels (Example 640)
#      image_height is the image width in pixels (Example 480)
#      Image width and image height are otherwise known as the resolution
def calculateFocalLengthsFromInput(image_width, image_height):
    diagonalView = math.radians(68.5)

    #To calculate the aspect ratio, first find the greatest common divisor between the
    #image height and image width of the camera
    resolution_gcd = math.gcd(image_width, image_height)

    #The horizontal aspect is simply the imagewidth divided by the gcd
    #for examle, a 640x480 should give a GCD of 160, and aspect ratio is 4:3
    horizontalAspect = (image_width/resolution_gcd)

    #The vertical Aspect ratio is the image height divided by the gcd
    verticalAspect = (image_height/resolution_gcd)

    #print("aspect: ", horizontalAspect, verticalAspect)
    # Reasons for using diagonal aspect is to calculate horizontal field of view.
    diagonalAspect = math.hypot(horizontalAspect, verticalAspect)
    # Calculations: http://vrguy.blogspot.com/2013/04/converting-diagonal-field-of-view-and.html
    horizontalView = math.atan(math.tan(diagonalView / 2) * (horizontalAspect / diagonalAspect)) * 2
    verticalView = math.atan(math.tan(diagonalView / 2) * (verticalAspect / diagonalAspect)) * 2

    H_FOCAL_LENGTH = image_width / (2 * math.tan((horizontalView / 2)))
    V_FOCAL_LENGTH = image_height / (2 * math.tan((verticalView / 2)))

    return  H_FOCAL_LENGTH, V_FOCAL_LENGTH 



#CARGO_HEIGHT is actual height (for cargo height in feet)   
CONE_HEIGHT = 1.06770833333

#image height is the y resolution calculated from image size
#223 is the pixel height of a a ball found at a measured distance (which is 4 feet away)
#65 is the pixel height of a scale image 6 feet away
KNOWN_CONE_PIXEL_HEIGHT = 155
KNOWN_CONE_DISTANCE = 4

# Focal Length calculations: https://docs.google.com/presentation/d/1ediRsI-oR3-kwawFJZ34_ZTlQS2SDBLjZasjzZ-eXbQ/pub?start=false&loop=false&slide=id.g12c083cffa_0_165
# H_FOCAL_LENGTH = image_width / (2 * math.tan((horizontalView / 2)))
# V_FOCAL_LENGTH = image_height / (2 * math.tan((verticalView / 2)))
# blurs have to be odd
green_blur = 1
orange_blur = 27
yellow_blur = 1
red_blur = 1
blue_blur = 1

# define colors
purple = (255, 255, 0)
blue = (255, 0, 0)
green = (0, 255, 0)
red = (0, 0, 255)
cyan = (252, 252, 3)
white = (255, 255, 255)
yellow = (0, 255, 255)
orange = (60, 255, 255)
black = (0,0,0)

# define range of green of retroreflective tape in HSV
lower_green = np.array([55, 85, 146])
upper_green = np.array([94, 255, 255])

# define range of green of retroreflective tape in HSV
#lower_green = np.array([23, 50, 35])
#upper_green = np.array([85, 255, 255])


lower_yellow = np.array([5,67,80])
upper_yellow = np.array([30,260,245])

# masks for red and blue cargo (HSV)
lower_red = np.array([138,106,123])
upper_red = np.array([180,255,255])

lower_blue = np.array([64,127,116]) 
upper_blue = np.array([115,213,255]) 

lower_white = np.array([240,240,240])
upper_white = np.array([255,255,255])

lower_purple = np.array([99,2,49])
upper_purple = np.array([182,196,213])

blingColour = 0
