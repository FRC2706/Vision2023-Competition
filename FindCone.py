import cv2
import numpy as np
import math
from VisionUtilities import * 
from VisionConstants import *
from DistanceFunctions import *

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
def findCone(frame, MergeVisionPipeLineTableName,CameraFOV):
    # Copies frame and stores it in image
    image = frame.copy()
    #Create a yellow mask
    MaskYellow = threshold_video(lower_yellow, upper_yellow, image)

    #Cannot show image on raspberry Pi, you can draw on the frame
    #cv2.imshow("Mask", MaskYellow)
    kernel = np.ones((10,3), np.uint8)
    # Using cv2.erode() method 
    MaskYellow = cv2.erode(MaskYellow, kernel)
    MaskYellow = cv2.erode(MaskYellow, kernel)
    MaskYellow = cv2.erode(MaskYellow, kernel)
    
    #Cannot show image on raspberry Pi, you can draw on the frame
    #cv2.imshow("Eroded", MaskYellow)

    #find the contours of the mask 
    if is_cv3():
        print("is_cv3")
        _, contours, _ = cv2.findContours(MaskYellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    else:
        contours, _ = cv2.findContours(MaskYellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    
    Yaw = 0
    # Processes the contours, takes in (contours, output_image, (centerOfImage)
    if len(contours) != 0:    
        # Sort contours by area size (biggest to smallest)
        cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)[:5]#what is this 5?
        image,Yaw = findCones(cntsSorted, image, CameraFOV,)
    # Shows the contours overlayed on the original video
        publishNumber(MergeVisionPipeLineTableName, "YawToCone", Yaw)
    return image, Yaw

def findCones(cntsSorted, image, CameraFOV):
    screenHeight, screenWidth, _ = image.shape
    # Gets center of width
    centerX = (screenWidth / 2) - .5

    cntHeight = 0
    biggestCone = []
    
    for cnt in cntsSorted:
        
        cv2.drawContours(image, [cnt], 0, green, 2)

        x, y, w, h = cv2.boundingRect(cnt)

        ##print("Area of bounding rec: " + str(boundingRectArea))
        # Calculate Contour area
        cntArea = cv2.contourArea(cnt)
        #print("Area of contour: " + str(cntArea))
        #calculate area of a cone standing up at that size
        expectedArea = (w*h/2)
        #print("expected area: " + str(expectedArea))
        #percentage of contour in area of a cone standing up at that size
        expectedAreaContArea = float(cntArea/expectedArea)
        #print("percentage of contour in area of a cone standing up at that size: " + str(expectedAreaContArea))

        #find the height of the bottom (y position of contour)
        # which is just the y value plus the height
        bottomHeight = y+h
        # Get moments of contour; mainly for centroid
        M = cv2.moments(cnt)

        # Filters contours based off of size
        if (checkCone(cntArea, expectedAreaContArea)):
            ### MOSTLY DRAWING CODE, BUT CALCULATES IMPORTANT INFO ###
            # Gets the centeroids of contour
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0
            if (len(biggestCone) < 3):

                ##### DRAWS CONTOUR######
                # Gets rotated bounding rectangle of contour
                #rect = cv2.minAreaRect(cnt)
                # Creates box around that rectangle
                #box = cv2.boxPoints(rect)
                # Covert boxpoints to integer
                #box = np.int0(box)
                   
                # Draws a vertical white line passing through center of contour
                cv2.line(image, (cx, screenHeight), (cx, 0), yellow,5)

                # Draws the contours
                #cv2.drawContours(image, [cnt], 0, green, 2)

                # Draws contour of bounding rectangle in red
                #cv2.rectangle(image, (x, y), (x + w, y + h), red, 1)
                   
                # Appends important info to array
                if [cx, cy, cnt, bottomHeight] not in biggestCone:
                    biggestCone.append([cx, cy, cnt, bottomHeight, cntHeight])
                    

        # Check if there are Cone seen
        if (len(biggestCone) > 0):
            # copy
            tallestCone = biggestCone

            # pushes that it sees Cone to network tables

            finalTarget = []
            # Sorts targets based on tallest height (bottom of contour to top of screen or y position)
            tallestCone.sort(key=lambda height: math.fabs(height[3]))


            #sorts closestCone - contains center-x, center-y, contour and contour height from the
            #bounding rectangle.  The closest one has the largest bottom point
            closestCone = min(tallestCone, key=lambda height: (math.fabs(height[3] - centerX)))

            # extreme points
            #topmost = tuple(closestCone[2][closestCone[2][:,:,1].argmin()][0])
            bottommost = tuple(closestCone[2][closestCone[2][:,:,1].argmax()][0])

            # draw extreme points
            # from https://www.pyimagesearch.com/2016/04/11/finding-extreme-points-in-contours-with-opencv/
            #cv2.circle(image, topmost, 6, white, -1)
            #cv2.circle(image, bottommost, 6, blue, -1)
            ##print('extreme points', leftmost,rightmost,topmost,bottommost)

            #print("topmost: " + str(topmost[0]))
            #print("bottommost: " + str(bottommost[0]))
           
            #print("bottommost[1]: " + str(bottommost[1]))
            #print("screenheight: " + str(screenHeight))

            # Contour that fills up bottom seems to reside on one less than 
            # screen height.  For example, screenHeight of 480 has bottom
            # pixel as 479, probably because 0-479 = 480 pixel rows
            if (int(bottommost[1]) >= screenHeight - 1):
                # This is handing over centoid X when bottommost is in bottom row
                xCoord = closestCone[0]
            else:
                # This is handing over X of bottommost point
                xCoord = bottommost[0]   

            # calculate yaw and store in finalTarget0
            H_FOCAL_LENGTH, V_FOCAL_LENGTH = calculateFocalLengthsFromInput(CameraFOV,screenWidth, screenHeight)
            finalTarget.append(calculateYaw(xCoord, centerX, H_FOCAL_LENGTH))
            # calculate dist and store in finalTarget1
            finalTarget.append(404)#calculateDistWPILib(closestCone[4],CONE_HEIGHT,KNOWN_CONE_PIXEL_HEIGHT,KNOWN_CONE_DISTANCE ))
            # calculate yaw from pure centroid and store in finalTarget2
            finalTarget.append(calculateYaw(closestCone[0], centerX, H_FOCAL_LENGTH))

            #print("Yaw: " + str(finalTarget[0]))
            # Puts the yaw on screen
            # Draws yaw of target + line where center of target is
            #finalYaw = round(finalTarget[1]*1000)/1000
            #cv2.putText(image, "Yaw: " + str(finalTarget[0]), (40, 200), cv2.FONT_HERSHEY_COMPLEX, .6, white)
            #cv2.line(image, (xCoord, screenHeight), (xCoord, 0), blue, 2)

            #cv2.putText(image, "cxYaw (Used): " + str(finalTarget[2]), (40, 225), cv2.FONT_HERSHEY_COMPLEX, .6, white)


        else:
            finalTarget = [0,0,0]

        cv2.line(image, (round(centerX), screenHeight), (round(centerX), 0), white, 5)

        return image, finalTarget[2]

# Checks if cone contours are worthy based off of contour area and (not currently) hull area
def checkCone(cntArea, expectedAreaContArea):
    goodCone = (expectedAreaContArea > 0.8)
    #print(str(goodCone))
    return goodCone