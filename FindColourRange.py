from audioop import minmax
import cv2
import numpy as np
import math
from VisionUtilities import * 
from VisionConstants import *
from DistanceFunctions import *
from networktables import NetworkTablesInstance
from networktables.util import ntproperty

try:
    from PrintPublisher import *
except ImportError:
    from NetworkTablePublisher import *

# Note that findCone uses findBall which uses checkBall

# Draws on the image - > contours and finds center and yaw of nearest Cone
# Puts on network tables -> Yaw and Distance to nearest Cone ball
# frame is the original images, mask is a binary mask based on desired color
# centerX is center x coordinate of image
# MergeVisionPipeLineTableName is the Network Table destination for yaw and distance

# Finds the balls from the masked image and displays them on original stream + network tables
def findColourRange(frame):
    # Copies frame and stores it in image
    image = frame.copy()
    
    # Convert BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    h, s, v = cv2.split(hsv)

    contours = []
    hLower = 1
    while len(contours) == 0:
        hLower = hLower + 1
        tempMask = threshold_range(h, 0, hLower)
        tempMask = cv2.bitwise_and(tempMask, cv2.bitwise_and(s, v))
        if is_cv3():
            _, contours, _ = cv2.findContours(tempMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
        else:
            contours, _ = cv2.findContours(tempMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    print("hLower"+str(hLower))

    contours = []
    hUpper = 254
    while len(contours) == 0:
        hUpper = hUpper - 1
        tempMask = threshold_range(h, hUpper, 255)
        tempMask = cv2.bitwise_and(tempMask, cv2.bitwise_and(s, v))
        if is_cv3():
            _, contours, _ = cv2.findContours(tempMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
        else:
            contours, _ = cv2.findContours(tempMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    print("hUpper"+str(hUpper))




    contours = []
    sLower = 1
    while len(contours) == 0:
        sLower = sLower + 1
        tempMask = threshold_range(s, 0, sLower)
        tempMask = cv2.bitwise_and(h, cv2.bitwise_and(tempMask, v))
        if is_cv3():
            _, contours, _ = cv2.findContours(tempMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
        else:
            contours, _ = cv2.findContours(tempMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    print("sLower"+str(sLower))

    contours = []
    sUpper = 254
    while len(contours) == 0:
        sUpper = sUpper - 1
        tempMask = threshold_range(s, sUpper, 255)
        tempMask = cv2.bitwise_and(h, cv2.bitwise_and(tempMask, v))
        if is_cv3():
            _, contours, _ = cv2.findContours(tempMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
        else:
            contours, _ = cv2.findContours(tempMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    print("sUpper"+str(sUpper))



    contours = []
    vLower = 1
    while len(contours) == 0:
        vLower = vLower + 1
        tempMask = threshold_range(v, 0, vLower)
        tempMask = cv2.bitwise_and(h, cv2.bitwise_and(s, tempMask))
        if is_cv3():
            _, contours, _ = cv2.findContours(tempMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
        else:
            contours, _ = cv2.findContours(tempMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    print("vLower"+str(vLower))

    contours = []
    vUpper = 254
    while len(contours) == 0:
        vUpper = vUpper - 1
        tempMask = threshold_range(v, vUpper, 255)
        tempMask = cv2.bitwise_and(h, cv2.bitwise_and(s, tempMask))
        if is_cv3():
            _, contours, _ = cv2.findContours(tempMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
        else:
            contours, _ = cv2.findContours(tempMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    print("vUpper"+str(vUpper))
    
    return image

