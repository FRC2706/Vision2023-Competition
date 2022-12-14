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

# Note that findCargo uses findBall which uses checkBall

# Draws on the image - > contours and finds center and yaw of nearest cargo
# Puts on network tables -> Yaw and Distance to nearest cargo ball
# frame is the original images, mask is a binary mask based on desired color
# centerX is center x coordinate of image
# centerY is center y coordinate of image
# MergeVisionPipeLineTableName is the Network Table destination for yaw and distance

# Finds the balls from the masked image and displays them on original stream + network tables
def findCargo(frame, cameraFOV, mask, MergeVisionPipeLineTableName):
    # Finds contours
    if is_cv3():
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    else:
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)

    # Take each frame
    # Gets the shape of video
    screenHeight, screenWidth, _ = frame.shape
    # Gets center of height and width
    centerX = (screenWidth / 2) - .5
    centerY = (screenHeight / 2) - .5
    # Copies frame and stores it in image
    image = frame.copy()
    # Processes the contours, takes in (contours, output_image, (centerOfImage)
    if len(contours) != 0:
        image = findBall(contours, image, centerX, centerY, MergeVisionPipeLineTableName, cameraFOV)
    # Shows the contours overlayed on the original video
    return image

def findBall(contours, image, centerX, centerY, MergeVisionPipeLineTableName,cameraFOV):
    screenHeight, screenWidth, channels = image.shape
    # Seen vision targets (correct angle, adjacent to each other)
    #cargo = []

    if len(contours) > 0:
        # Sort contours by area size (biggest to smallest)
        cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)[:5]
        cntHeight = 0
        biggestCargo = []
        
        for cnt in cntsSorted:
            x, y, w, h = cv2.boundingRect(cnt)

            ##print("Area of bounding rec: " + str(w*h))
            boundingRectArea = w*h

            # Calculate Contour area
            cntArea = cv2.contourArea(cnt)
            ##print("Area of contour: " + str(cntArea))

            #percentage of contour in bounding rect
            boundingRectContArea = float(cntArea/boundingRectArea)
            #print("Percentage contour area in bounding rect: " + str(boundingRectContArea))
            cntHeight = h
            #find the height of the bottom (y position of contour)
            # which is just the y value plus the height
            bottomHeight = y+h
            #aspect_ratio = float(w) / h
            # Get moments of contour; mainly for centroid
            M = cv2.moments(cnt)

            # Filters contours based off of size
            if (checkBall(cntArea, image_width, boundingRectContArea)):
                ### MOSTLY DRAWING CODE, BUT CALCULATES IMPORTANT INFO ###
                # Gets the centeroids of contour
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = 0, 0
                if (len(biggestCargo) < 3):

                    ##### DRAWS CONTOUR######
                    # Gets rotated bounding rectangle of contour
                    #rect = cv2.minAreaRect(cnt)
                    # Creates box around that rectangle
                    #box = cv2.boxPoints(rect)
                    # Covert boxpoints to integer
                    #box = np.int0(box)
                   
                    # Draws a vertical white line passing through center of contour
                    cv2.line(image, (cx, screenHeight), (cx, 0), white)
                    # Draws a white circle at center of contour
                    cv2.circle(image, (cx, cy), 6, white)

                    # Draws the contours
                    cv2.drawContours(image, [cnt], 0, green, 2)

                    # Draws contour of bounding rectangle in red
                    cv2.rectangle(image, (x, y), (x + w, y + h), red, 1)
                   
                    # Appends important info to array
                    if [cx, cy, cnt, bottomHeight] not in biggestCargo:
                        biggestCargo.append([cx, cy, cnt, bottomHeight, cntHeight])
                    

        # Check if there are Cargo seen
        if (len(biggestCargo) > 0):
            # copy
            tallestCargo = biggestCargo

            # pushes that it sees cargo to network tables

            finalTarget = []
            # Sorts targets based on tallest height (bottom of contour to top of screen or y position)
            tallestCargo.sort(key=lambda height: math.fabs(height[3]))


            #sorts closestCargo - contains center-x, center-y, contour and contour height from the
            #bounding rectangle.  The closest one has the largest bottom point
            closestCargo = min(tallestCargo, key=lambda height: (math.fabs(height[3] - centerX)))

            # extreme points
            #topmost = tuple(closestCargo[2][closestCargo[2][:,:,1].argmin()][0])
            bottommost = tuple(closestCargo[2][closestCargo[2][:,:,1].argmax()][0])

            # draw extreme points
            # from https://www.pyimagesearch.com/2016/04/11/finding-extreme-points-in-contours-with-opencv/
            #cv2.circle(image, topmost, 6, white, -1)
            cv2.circle(image, bottommost, 6, blue, -1)
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
                xCoord = closestCargo[0]
            else:
                # This is handing over X of bottommost point
                xCoord = bottommost[0]   

            # calculate yaw and store in finalTarget0
            H_FOCAL_LENGTH, V_FOCAL_LENGTH = calculateFocalLengthsFromInput(cameraFOV,screenWidth, screenHeight)
            finalTarget.append(calculateYaw(xCoord, centerX, H_FOCAL_LENGTH))
            # calculate dist and store in finalTarget1
            finalTarget.append(calculateDistWPILib(closestCargo[4],CARGO_BALL_HEIGHT,KNOWN_CARGO_PIXEL_HEIGHT,KNOWN_CARGO_DISTANCE ))
            # calculate yaw from pure centroid and store in finalTarget2
            finalTarget.append(calculateYaw(closestCargo[0], centerX, H_FOCAL_LENGTH))

            #print("Yaw: " + str(finalTarget[0]))
            # Puts the yaw on screen
            # Draws yaw of target + line where center of target is
            #finalYaw = round(finalTarget[1]*1000)/1000
            cv2.putText(image, "Yaw: " + str(finalTarget[0]), (40, 360), cv2.FONT_HERSHEY_COMPLEX, .6,
                        white)
            cv2.putText(image, "Dist: " + str(finalTarget[1]), (40, 400), cv2.FONT_HERSHEY_COMPLEX, .6,
                        white)
            cv2.line(image, (xCoord, screenHeight), (xCoord, 0), blue, 2)

            cv2.putText(image, "cxYaw: " + str(finalTarget[2]), (450, 360), cv2.FONT_HERSHEY_COMPLEX, .6,
                        white)

            # pushes cargo angle to network tables
            publishNumber(MergeVisionPipeLineTableName, "YawToCargo", finalTarget[0])
            publishNumber(MergeVisionPipeLineTableName, "DistanceToCargo", finalTarget[1])
            publishNumber(MergeVisionPipeLineTableName, "CargoCentroid1Yaw", finalTarget[2])

        cv2.line(image, (round(centerX), screenHeight), (round(centerX), 0), white, 2)

        return image

# Checks if ball contours are worthy based off of contour area and (not currently) hull area
def checkBall(cntArea, image_width,boundingRectContArea):
    #this checks that the area of the contour is greater than the image width divide by 2
    #It also checks the percentage of the area of the bounding rectangle is
    #greater than 69%.  
    #print("cntArea " + str(cntArea))
    return (cntArea > (image_width*2)) and (boundingRectContArea > 0.69)

if __name__ == "__main__":

    # the purpose of this code is to test the functions above
    # findCargo uses findBall which uses checkBall
    # this test does not use a real network table
    # TODO #2 get network tables working in test code

    # create empty bgr image for the test
    bgrTestImage = np.zeros(shape=[240, 320, 3], dtype=np.uint8)

    # draw a yellow rectangle on the test image
    bgrTestImage = cv2.circle(bgrTestImage,(100,100), 50, (0,255,255),-1)

    # display the test image to verify it visually
    cv2.imshow('This is the test', bgrTestImage)
    
    # convert image to hsv from bgr
    hsvTestImage = cv2.cvtColor(bgrTestImage, cv2.COLOR_BGR2HSV)

    # using inrange from opencv make mask
    mskBinary = cv2.inRange(hsvTestImage, (29,254,254), (31,255,255)) # (30, 255, 255)

    # display the mask to verify it visually
    cv2.imshow('This is the mask', mskBinary)

    # use a dummy network table for test code for now, real network tables not working
    MergeVisionPipeLineTableName = "DummyNetworkTableName"

    # use findCargo, which uses findBall, which uses checkBall to generate image
    bgrTestFoundBall = findCargo(bgrTestImage, mskBinary, MergeVisionPipeLineTableName)

    # display the visual output (nearest based on height) of findBall to verify it visually
    cv2.imshow('Test of 1 ball findBall output', bgrTestFoundBall)

    # wait for user input to close
    cv2.waitKey(0)

    # cleanup so we can do second test of two balls
    cv2.destroyAllWindows()

    # create empty bgr image for the test
    bgrTestImage = np.zeros(shape=[240, 320, 3], dtype=np.uint8)

    # draw two yellow circle on the test image
    bgrTestImage = cv2.circle(bgrTestImage,(100,100), 50, (0,255,255),-1)
    bgrTestImage = cv2.circle(bgrTestImage,(280,105), 45, (0,255,255),-1)

    # display the test image to verify it visually
    cv2.imshow('This is the test', bgrTestImage)
    
    # convert image to hsv from bgr
    hsvTestImage = cv2.cvtColor(bgrTestImage, cv2.COLOR_BGR2HSV)

    # using inrange from opencv make mask
    mskBinary = cv2.inRange(hsvTestImage, (29,254,254), (31,255,255)) # (30, 255, 255)
    
    # display the mask to verify it visually
    cv2.imshow('This is the mask', mskBinary)

    # use a dummy network table for test code for now, real network tables not working
    MergeVisionPipeLineTableName = "DummyNetworkTableName"

    # use findCargo, which uses findBall, which uses checkBall to generate image
    bgrTestFoundBall = findCargo(bgrTestImage, mskBinary, MergeVisionPipeLineTableName)

    # display the visual output (nearest based on height) of findBall to verify it visually
    cv2.imshow('Test of 2 ball findBall output', bgrTestFoundBall)

    # wait for user input to close
    cv2.waitKey(0)
    # cleanup and exit
    cv2.destroyAllWindows()