import cv2
import numpy as np
import numpy as geek
import math
from VisionUtilities import * 
from VisionConstants import *
from DistanceFunctions import *

try:
    from PrintPublisher import *
except ImportError:
    from NetworkTablePublisher import *


# Finds the balls from the masked image and displays them on original stream + network tables
def DetectIntakeItem(frame, MergeVisionPipeLineTableName):

    H, W,_ = frame.shape

    # Copies frame and stores it in image
    image = frame.copy()

    #Create a yellow mask
    MaskYellow = threshold_video(lower_yellow, upper_yellow, image)
    #create a Purple Mask
    MaskPurple = threshold_video(lower_purple, upper_purple, image)
    
    
    #find the contours of the mask 
    if is_cv3():
        _, contours, _ = cv2.findContours(MaskYellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    else:
        contours, _ = cv2.findContours(MaskYellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    FoundYellow, coneRectFilledArea = FindRectFillAmount(contours,H,W, image)
    
        #find the contours of the mask 
    if is_cv3():
        _, contours, _ = cv2.findContours(MaskPurple, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    else:
        contours, _ = cv2.findContours(MaskPurple, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    FoundPurple, _ = FindRectFillAmount(contours,H,W, image)
    
    cv2.putText(frame, "Yellow: " + str(FoundYellow), (10, 350), cv2.FONT_HERSHEY_COMPLEX, .9, white)
    cv2.putText(frame, "Purple: " + str(FoundPurple), (10, 375), cv2.FONT_HERSHEY_COMPLEX, .9, white)
    #cv2.putText(image, "Fill Value: " + str(DesiredRectFilledArea), (450, 360), cv2.FONT_HERSHEY_COMPLEX, .4, white)
    # pushes cargo angle to network tables
    #publishNumber(MergeVisionPipeLineTableName, "YawToCargo", finalTarget[0])
    #publishNumber(MergeVisionPipeLineTableName, "DistanceToCargo", finalTarget[1])
    #publishNumber(MergeVisionPipeLineTableName, "CargoCentroid1Yaw", finalTarget[2])
    #cv2.line(image, (round(x), round(y)), (round(x+w), round(y+h)), white, 2)
    #cv2.imshow("area",image)
    
    if FoundYellow:
        publishBoolean(MergeVisionPipeLineTableName, "DetectCone", True)
        print("coneFound")
        if coneRectFilledArea > 0.9:
            publishBoolean(MergeVisionPipeLineTableName, "ConeTopInwards", True)
            print("ConeTopInwards")
        else:
            publishBoolean(MergeVisionPipeLineTableName, "ConeTopInwards", False)
    else:
        publishBoolean(MergeVisionPipeLineTableName, "DetectCone", False)
    if FoundPurple:
        publishBoolean(MergeVisionPipeLineTableName, "DetectCube", True)
    else:
        publishBoolean(MergeVisionPipeLineTableName, "DetectCube", False)
    return image

def FindRectFillAmount(contours,H,W, image):
    # Seen vision targets (correct angle, adjacent to each other)
    #cargo = []

    if len(contours) > 0:
        cnt = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)[0]
        _, _, cntw, cnth = cv2.boundingRect(cnt)
        # Calculate Contour area
        cntArea = cv2.contourArea(cnt)
        boundingRectFilledArea = cntArea/(cnth*cntw)
        #print("Area of contour: " + str(cntsArea))
        imageArea = W*H
        #percentage of contours in desired rect
        imageFilledArea = float(cntArea/imageArea)
        if imageFilledArea > 0.35:
            Found = True
        else:
            Found = False
    else:
        Found = False
        boundingRectFilledArea = 0.0 
    return Found, boundingRectFilledArea