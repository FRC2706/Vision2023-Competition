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

    screenHeight, screenWidth,_ = frame.shape
    #where on the screen do you want to check?
    x = round(screenWidth/2 - screenWidth/(4)/2)
    y = round(screenHeight - screenHeight/(4))
    w = round(screenWidth/(4))
    h = round(screenHeight/(4))

    # Copies frame and stores it in image
    image = frame.copy()
    cv2.rectangle(image,(0,0),(x,screenHeight),([0,0,0]),-1)
    cv2.rectangle(image,(x,0),(x+w,y),([0,0,0]),-1)
    cv2.rectangle(image,(x+w,0),(screenWidth,screenHeight),([0,0,0]),-1)

    #Create a yellow mask
    MaskYellow = threshold_video(lower_yellow, upper_yellow, image)
    displayMask = MaskYellow
    #create a Purple Mask
    MaskPurple = threshold_video(lower_purple, upper_purple, image)
    
    
    #find the contours of the mask 
    if is_cv3():
        _, contours, _ = cv2.findContours(MaskYellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    else:
        contours, _ = cv2.findContours(MaskYellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    image, FoundYellow, DesiredRectFilledArea = FindRectFillAmount(image,contours,x,y,w,h)
    
    if FoundYellow == False:
            #find the contours of the mask 
        if is_cv3():
            _, contours, _ = cv2.findContours(MaskPurple, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
        else:
            contours, _ = cv2.findContours(MaskPurple, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
        image,FoundPurple, DesiredRectFilledArea = FindRectFillAmount(image,contours,x,y,w,h)
        displayMask = MaskPurple
    else:
        FoundPurple = False
    
    cv2.putText(frame, "Yellow: " + str(FoundYellow), (10, 350), cv2.FONT_HERSHEY_COMPLEX, .9, white)
    cv2.putText(frame, "Purple: " + str(FoundPurple), (10, 375), cv2.FONT_HERSHEY_COMPLEX, .9, white)
    #cv2.putText(image, "Fill Value: " + str(DesiredRectFilledArea), (450, 360), cv2.FONT_HERSHEY_COMPLEX, .4, white)
    # pushes cargo angle to network tables
    #publishNumber(MergeVisionPipeLineTableName, "YawToCargo", finalTarget[0])
    #publishNumber(MergeVisionPipeLineTableName, "DistanceToCargo", finalTarget[1])
    #publishNumber(MergeVisionPipeLineTableName, "CargoCentroid1Yaw", finalTarget[2])
    #cv2.line(image, (round(x), round(y)), (round(x+w), round(y+h)), white, 2)
    #cv2.imshow("area",image)
    return frame

def FindRectFillAmount(image,contours,x,y,w,h):
    # Seen vision targets (correct angle, adjacent to each other)
    #cargo = []

    if len(contours) > 0:
        
        cntsArea = 0
        for cnt in contours:
            cntx, cnty, cntw, cnth = cv2.boundingRect(cnt)
            #print("cnth: " + str(cnth))
            
            cv2.drawContours(image, [cnt], 0, green, 2)
            # Calculate Contour area
            cntsArea += cv2.contourArea(cnt)
            #print("Area of contour: " + str(cntsArea))
        desiredRectArea = w*h
        #percentage of contours in desired rect
        desiredRectFilledArea = float(cntsArea/desiredRectArea)
        if desiredRectFilledArea > 0.1:
            Found = True
        else:
            Found = False
            
        return image, Found, desiredRectFilledArea
    return image, False, 0