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

    for row in image:
        for pixel in row:
            if pixel[0]-100 < 0:
                pixel[0] = 0
            else:
                pixel[0] -=100

    #Create a yellow mask
    MaskYellow = threshold_video(lower_yellow, upper_yellow, image)
    #create a Purple Mask
    MaskPurple = threshold_video(lower_purple, upper_purple, image)
    MaskPurple = MaskPurple[0:round(6*H/7), 0:W-1]
    
    
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
    
    #cv2.putText(frame, "Yellow: " + str(FoundYellow), (10, 350), cv2.FONT_HERSHEY_COMPLEX, .9, white)
    #cv2.putText(frame, "Purple: " + str(FoundPurple), (10, 375), cv2.FONT_HERSHEY_COMPLEX, .9, white)
    #cv2.putText(image, "Fill Value: " + str(DesiredRectFilledArea), (450, 360), cv2.FONT_HERSHEY_COMPLEX, .4, white)
    # pushes cargo angle to network tables
    #publishNumber(MergeVisionPipeLineTableName, "YawToCargo", finalTarget[0])
    #publishNumber(MergeVisionPipeLineTableName, "DistanceToCargo", finalTarget[1])
    #publishNumber(MergeVisionPipeLineTableName, "CargoCentroid1Yaw", finalTarget[2])
    #cv2.line(image, (round(x), round(y)), (round(x+w), round(y+h)), white, 2)
    #cv2.imshow("area",image)
    
    cropped_image_one = image[round(5*H/14):round(7*H/14), round(4*W/9):round(5*W/9)]
    hsv = cv2.cvtColor(cropped_image_one, cv2.COLOR_BGR2HSV)
    average_color_row_one = np.average(hsv, axis=0)
    average_color_one = np.average(average_color_row_one, axis=0)
    #cv2.imshow('Source image one',cropped_image_one)
    #print(average_color_one)

    cropped_image_two = image[round(5*H/7):round(6*H/7), round(2*W/5):round(3*W/5)]
    hsv = cv2.cvtColor(cropped_image_two, cv2.COLOR_BGR2HSV)
    average_color_row_two = np.average(hsv, axis=0)
    average_color_two = np.average(average_color_row_two, axis=0)
    #cv2.imshow('Source image two',cropped_image_two)
    #print(average_color_two)
    
    
        
    if FoundYellow:
        #print("yes")
        publishBoolean(MergeVisionPipeLineTableName, "DetectCone", True)
        if (average_color_one[2]> average_color_two[2]+25) and not colourBetween(lower_grey,upper_grey, average_color_one):
            publishBoolean(MergeVisionPipeLineTableName, "ConeNoseIn", False)
            print ("base in")
        else:
            publishBoolean(MergeVisionPipeLineTableName, "ConeNoseIn", True)
            print("nose in")
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
        cntx, cnty, cntw, cnth = cv2.boundingRect(cnt)
        #cv2.rectangle(image,[cntx, cnty], [cntw, cnth],[255,0,0],4)
        # Calculate Contour area
        cntArea = cv2.contourArea(cnt)
        boundingRectFilledArea = cntArea/(cnth*cntw)
        #print(boundingRectFilledArea)
        #print("Area of contour: " + str(cntsArea))
        imageArea = W*H
        #percentage of contours in desired rect
        imageFilledArea = float(cntArea/imageArea)
        if imageFilledArea > 0.3:
            Found = True
        else:
            Found = False
    else:
        Found = False
        boundingRectFilledArea = 0.0 
    return Found, boundingRectFilledArea