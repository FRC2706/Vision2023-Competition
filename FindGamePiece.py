import cv2
import numpy as np
import numpy as geek
import math
from VisionUtilities import * 
from VisionConstants import *
from DistanceFunctions import *
from ntcore import NetworkTableInstance
#from ntcore.util import ntproperty

try:
    from PrintPublisher import *
except ImportError:
    from NetworkTablePublisher import *

# Note that findCargo uses findBall which uses checkBall

# Draws on the image - > contours and finds center and yaw of nearest cargo
# Puts on network tables -> Yaw and Distance to nearest cargo ball
# frame is the original images, mask is a binary mask based on desired color
# centerX is center x coordinate of image
# centerY is center y coordinate of image
# MergeVisionPipeLineTableName is the Network Table destination for yaw and distance

# Finds the balls from the masked image and displays them on original stream + network tables
def DetectIntakeItem(frame, MergeVisionPipeLineTableName):

    screenHeight, screenWidth,_ = frame.shape
    #where on the screen do you want to check?
    x = screenWidth/2 - screenWidth/(6)/2
    y = screenHeight - screenHeight/(6)
    w = screenWidth/(6)
    h = screenHeight/(6)

    #Create a yellow mask
    MaskYellow = threshold_video(lower_yellow, upper_yellow, frame)
    displayMask = MaskYellow
    #create a Purple Mask
    MaskPurple = threshold_video(lower_purple, upper_purple, frame)
    
    # Take each frame
    # Copies frame and stores it in image
    image = frame.copy()

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
    
    cv2.putText(image, "Yellow: " + str(FoundYellow), (10, round(image_height/3)), cv2.FONT_HERSHEY_COMPLEX, .9, white)
    cv2.putText(image, "Purple: " + str(FoundPurple), (10, round(image_height/3)+40), cv2.FONT_HERSHEY_COMPLEX, .9, white)
    cv2.putText(image, "Fill Value: " + str(DesiredRectFilledArea), (450, 360), cv2.FONT_HERSHEY_COMPLEX, .4, white)
    # pushes cargo angle to network tables
    #publishNumber(MergeVisionPipeLineTableName, "YawToCargo", finalTarget[0])
    #publishNumber(MergeVisionPipeLineTableName, "DistanceToCargo", finalTarget[1])
    #publishNumber(MergeVisionPipeLineTableName, "CargoCentroid1Yaw", finalTarget[2])
    cv2.line(image, (round(x), round(y)), (round(x+w), round(y+h)), white, 2)
    return image, displayMask, FoundYellow, FoundPurple

def FindRectFillAmount(image,contours,x,y,w,h):
    # Seen vision targets (correct angle, adjacent to each other)
    #cargo = []

    if len(contours) > 0:
        
        cntsArea = 0
        for cnt in contours:
            cntx, cnty, cntw, cnth = cv2.boundingRect(cnt)
            
            #is it inside the desired box?
            if cntx < x+h and cnty < y+h and cntx+cntw > x and cnty+cnth > y and cv2.contourArea(cnt)/(cntw*cnth) >0.1:
            #and cntw<(w*2) and cnth<h*2:     WHAT????????
                cv2.drawContours(image, [cnt], 0, green, 2)
                #find and draw center of cnt
                M = cv2.moments(cnt)
                cntx = int(M["m10"] / M["m00"])
                cnty = int(M["m01"] / M["m00"])
                cv2.circle(image, (cntx, cnty), 7, (255, 255, 255), -1)

                # Calculate Contour area
                cntsArea += cv2.contourArea(cnt)
                print("Area2 of contour: " + str(cntsArea))
                print("here")
    
                #find the difference between center of game piece and desired placement
                p=[cntx,cnty]
                q=[200,350]
                print(math.dist(p,q))
                

                
        desiredRectArea = w*h
        #percentage of contours in desired rect
        desiredRectFilledArea = float(cntsArea/desiredRectArea)
        if desiredRectFilledArea > 0.1:
            Found = True
        else:
            Found = False
            
        return image, Found, desiredRectFilledArea
    return image, False, 0