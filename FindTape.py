import math
import numpy as np

from VisionConstants import *
from VisionMasking import *
from VisionUtilities import *
from DistanceFunctions import *

#from CornersVisual4 import get_four

try:
    from PrintPublisher import *
except ImportError:
    from NetworkTablePublisher import *


# real world dimensions of the goal target
# These are the full dimensions around both strips
TARGET_STRIP_LENGTH = 2.0    # inches
TARGET_STRIP_WIDTH = 4.0        # inches


real_world_coordinates = np.array([
    [-TARGET_STRIP_LENGTH / 2.0, TARGET_STRIP_WIDTH / 2.0, 0.0], # for top left corner
    [TARGET_STRIP_LENGTH / 2.0, TARGET_STRIP_WIDTH / 2.0, 0.0], # for top right
    [TARGET_STRIP_LENGTH / 2.0, -TARGET_STRIP_WIDTH / 2.0, 0.0], # for bottom right
    [-TARGET_STRIP_LENGTH / 2.0, -TARGET_STRIP_WIDTH / 2.0, 0.0], # for bottom left
])



MAXIMUM_TARGET_AREA = 4400

# Finds the tape targets from the masked image and displays them on original stream + network tales
def findTargets(frame, cameraFOV, CameraTiltAngle, mask, MergeVisionPipeLineTableName, past_distances):

    
    # Taking a matrix of size 5 as the kernel 
    #kernel = np.ones((3,3), np.uint8) 
  
    # The first parameter is the original image, 
    # kernel is the matrix with which image is  
    # convolved and third parameter is the number  
    # of iterations, which will determine how much  
    # you want to erode/dilate a given image.  
    #img_erosion = cv2.erode(mask, kernel, iterations=1) 
    #mask = cv2.dilate(img_erosion, kernel, iterations=1) 
    #cv2.imshow("mask2", mask)

    # Finds contours
    # we are accomodating different versions of openCV and the different methods for corners
    if is_cv3():
       # if CornerMethod is 1 or CornerMethod is 2 or CornerMethod is 3:
       #     _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
       # elif CornerMethod is 4 or CornerMethod is 5:
       #     _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
       # elif CornerMethod is 6 or CornerMethod is 7:
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        #elif CornerMethod is 8 or CornerMethod is 9 or CornerMethod is 10:
        #    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
    else: #implies not cv3, either version 2 or 4
        #if CornerMethod is 1 or CornerMethod is 2 or CornerMethod is 3:
        #    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #elif CornerMethod is 4 or CornerMethod is 5:
        #    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
        #elif CornerMethod is 6 or CornerMethod is 7:
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        #elif CornerMethod is 8 or CornerMethod is 9 or CornerMethod is 10:
        #    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #else:
        #    pass

    contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

    # rotates frame 90 deg clockwise
    # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    # Take each frame
    # Gets the shape of video
    screenHeight, screenWidth, _ = frame.shape
    # Gets center of height and width
    centerX = (screenWidth / 2) - .5
    centerY = (screenHeight / 2) - .5
    # Copies frame and stores it in image
    image = frame.copy()
    # Processes the contours, takes in (contours, output_image, (centerOfImage)
    final_center = -99
    YawToTarget = -99
    distance = -1
    if len(contours) != 0:
        image, final_center, YawToTarget, distance = findTape(contours, image, centerX, centerY, mask, MergeVisionPipeLineTableName, past_distances, cameraFOV, CameraTiltAngle)
    else:
        past_distances.clear()
        YawToTarget = -99
        final_center = -99
        distance = -1
        publishNumber(MergeVisionPipeLineTableName, "YawToTarget", -99)
        publishNumber(MergeVisionPipeLineTableName, "DistanceToTarget", -1)  
        publishNumber(MergeVisionPipeLineTableName, "AverageDistance", -1)  
    # Shows the contours overlayed on the original video
    return image, final_center, YawToTarget, distance




# Simple method to order points from left to right
def order_points(pts):
    # initialize a list of coordinates that will be ordered
    # such that the first entry in the list is the top-left,
    # the second entry is the top-right, the third is the
    # bottom-right, and the fourth is the bottom-left
    rect = np.zeros((4, 2), dtype="float32")
 
    # the top-left point will have the smallest sum, whereas
    # the bottom-right point will have the largest sum
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
 
    # now, compute the difference between the points, the
    # top-right point will have the smallest difference,
    # whereas the bottom-left will have the largest difference
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
 
    # return the ordered coordinates
    return rect

 #3D Rotation estimation
 
def findTvecRvec(image, outer_corners, real_world_coordinates, H_FOCAL_LENGTH, V_FOCAL_LENGTH):
    # Read Image
    size = image.shape
 
    # Camera internals
 
    focal_length = size[1]
    center = (size[1]/2, size[0]/2)
    camera_matrix = np.array(
                          [[H_FOCAL_LENGTH, 0, center[0]],
                          [0, V_FOCAL_LENGTH, center[1]],
                          [0, 0, 1]], dtype = "double"
                          )

    dist_coeffs = np.array([[0.16171335604097975, -0.9962921370737408, -4.145368586842373e-05, 
                             0.0015152030328047668, 1.230483016701437]])

    #print(camera_matrix)
    # This is tested for a 640x480 resolution
    # Should be 
    #     Fx  0  Cx
    #     0  Fy  Cy
    #     0   0  1
    #  Cx and Cy are close to center of image,  Fx and Fy are focal lengths in pixel units
    #camera_matrix = np.array([[676.9254672222575, 0.0, 303.8922263320326], 
    #                          [0.0, 677.958895098853, 226.64055316186037], 
    #                          [0.0, 0.0, 1.0]], dtype = "double")

    #print("Camera Matrix :\n {0}".format(camera_matrix))                           
 
    #dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
    (success, rotation_vector, translation_vector) = cv2.solvePnP(real_world_coordinates, outer_corners, camera_matrix, dist_coeffs)
 
    #print ("Rotation Vector:\n {0}".format(rotation_vector))
    #print ("Translation Vector:\n {0}".format(translation_vector))
    return success, rotation_vector, translation_vector


#Computer the final output values, 
#angle 1 is the Yaw to the target
#distance is the distance to the target
#angle 2 is the Yaw of the Robot to the target
def compute_output_values(rvec, tvec, cameraTiltAngle):
    '''Compute the necessary output distance and angles'''

    # The tilt angle only affects the distance and angle1 calcs
    # This is a major impact on calculations
    tilt_angle = math.radians(cameraTiltAngle)

    x = tvec[0][0]
    z = math.sin(tilt_angle) * tvec[1][0] + math.cos(tilt_angle) * tvec[2][0]

    # distance in the horizontal plane between camera and target
    distance = math.sqrt(x**2 + z**2)

    # horizontal angle between camera center line and target
    angleInRad = math.atan2(x, z)
    angle1 = math.degrees(angleInRad)

    rot, _ = cv2.Rodrigues(rvec)
    rot_inv = rot.transpose()
    pzero_world = np.matmul(rot_inv, -tvec)
    angle2InRad = math.atan2(pzero_world[0][0], pzero_world[2][0])
    angle2 = math.degrees(angle2InRad)

    # distance = 

    return distance, angle1, angle2

#Simple function that displays 4 corners on an image
#A np.array() is expected as the input argument
def displaycorners(image, outer_corners):
    # draw extreme points
    # from https://www.pyimagesearch.com/2016/04/11/finding-extreme-points-in-contours-with-opencv/
    if len(outer_corners) == 4: #this is methods 1 to 4 
        cv2.circle(image, (int(outer_corners[0,0]),int(outer_corners[0,1])), 6, green, -1)
        cv2.circle(image, (int(outer_corners[1,0]),int(outer_corners[1,1])), 6, red, -1)
        cv2.circle(image, (int(outer_corners[2,0]),int(outer_corners[2,1])), 6, white,-1)
        cv2.circle(image, (int(outer_corners[3,0]),int(outer_corners[3,1])), 6, blue, -1)
        #print('extreme points', leftmost,rightmost,topmost,bottommost)
    else: # this assumes len is 5 and method 5
        cv2.circle(image, (int(outer_corners[0,0]),int(outer_corners[0,1])), 6, green, -1)
        cv2.circle(image, (int(outer_corners[1,0]),int(outer_corners[1,1])), 6, blue, -1)
        cv2.circle(image, (int(outer_corners[2,0]),int(outer_corners[2,1])), 6, purple, -1)
        cv2.circle(image, (int(outer_corners[3,0]),int(outer_corners[3,1])), 6, white,-1)
        cv2.circle(image, (int(outer_corners[4,0]),int(outer_corners[4,1])), 6, red, -1)

# Function that takes a list of 3 corners of a contour and gives you the closest to the center

def getAverageArray(array):
    values = 0
    final_value = 0
    
    if len(array) > 0:
        for i in range(len(array)):
            values += array[i]
        final_value = values/len(array)
        #except ValueError:
        #print("Something is wrong with one of these values:", array)
    
    return final_value 

# gets minimum contour from a two dimensional array to find closest corner to centerpoint out of a set of arrays
def minContour(center, contourCorners):
    xDiff = [] # list that stores the differences between number (final_center) and contourCorners
    minVar = 10000 # will change to the greatest value that shows up
    closestCorner = 0 # set closestCorner to the value of i (so 1-4 in most cases)

    for i in range(len(contourCorners)): # two for loops as contourCorners is a 2D array
        for j in contourCorners[i]:
            #xDiff.insert([center-(i[0][0]), center-(i[1][0]), center-(i[1][0]), center-(i[1][1])])
            xDiff.append(abs(center-(j[0][0]))) # append the differences between number (final_center) and j[0][0] and add to xDiff
            var = min(xDiff)
            if var < minVar : # if var the value (min(xDiff) which is an array with 3+ elements) is greater than current minVar, then set minVar to var, 
                minVar = var # set minVar to var, so minVar becomes the highest value
                closestCorner = i # closest corner will be the number of the element of contourCorners where it is closest to center
            #print("i[0][0] data: ", j[0][0])

    #print("xDiff: ", xDiff)
    #print("len of contourCorners: ", len(contourCorners[0]))

    #smallestDiff = min(xDiff)
    return closestCorner

# Draws Contours and finds center and yaw of vision targets
# centerX is center x coordinate of image
# centerY is center y coordinate of image
# Draws Contours and finds center and yaw of vision targets
# centerX is center x coordinate of image
# centerY is center y coordinate of image

def findTape(contours, image, centerX, centerY, mask, MergeVisionPipeLineTableName, past_distances, cameraFOV, CameraTiltAngle):
    global blingColour
    #global warped
    screenHeight, screenWidth, channels = image.shape
    # Seen vision targets (correct angle, adjacent to each other)
    # targets = []
    # Constant used as minimum area for fingerprinting is equal to 60% of screenWidth. (Using 
    # a value based on screenWidth scales properly if the resolution ever changes.)
    # minContourArea = 0.6 * screenWidth

    final_center = -99
    YawToTarget = -99
    distance = -1

    if len(contours) >= 1:
        # Sort contours by area size (biggest to smallest)
        cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)[:10]
       
        cntsFiltered = []

        # First contour has largest area, so only go further if that one meets minimum area criterion
        if cntsSorted:

            for (j, cnt) in enumerate(cntsSorted):

                # Calculate Contour area
                cntArea = cv2.contourArea(cnt)

                # rotated rectangle fingerprinting
               # bounding = cv2.boundingRect(cnt)
                rect = cv2.minAreaRect(cnt)

               #rect = cv2.boundingRect(cnt)
                (xr,yr),(wr,hr),ar = rect #x,y width, height, angle of rotation = rotated rect


                x, y, w, h = cv2.boundingRect(cnt)
                #cv2.rectangle(image,(x,y),(x+w,y+h),(0, 255, 255),1)
                # cv2.imshow("result",image)

                percentfill = (cntArea/(w*h))


                total_screen_pixels = screenHeight * screenWidth
                percent_contour_area = cntArea/total_screen_pixels *100

                #Filter based on too large contour 
                # if cntArea > 2500 or cntArea < 440: continue
 
                #Filter based on too small contour
                if (percent_contour_area < 0.05): continue

                # Filter based on percent fill
                if (percentfill < 0.5): continue 

                # Filter based on Angle of rotation 


                #to get rid of height and width switching
                if hr > wr: 
                    ar = ar + 90
                    wr, hr = [hr, wr]
                else:
                    ar = ar + 180
                if ar == 180:
                    ar = 0

                if hr == 0: continue
                cntMinAreaAR = float(wr)/hr
                cntBoundRectAR = float(w)/h


                # Filter based on aspect ratio (previous values: 2-3)
                #Tape is 13 cm wide by 5 cm high - that gives aspect ration of 2.6
                #To be flexible, lets accept (1.9 - 3.3) range 
                if (cntBoundRectAR < 0.2 or cntBoundRectAR > 0.8): continue 

                cv2.rectangle(image,(x,y),(x+w,y+h),(0, 0, 255),1)
                
                #minAextent = float(cntArea)/(wr*hr)

                # Hull
                #hull = cv2.convexHull(cnt)
                #hull_area = cv2.contourArea(hull)
                #solidity = float(cntArea)/hull_area

               
                # Filter based on minimum area extent (previous values: 0.16-0.26)
                #if (minAextent < 0.139 or minAextent > 1.1): continue
               
                # Filter based on solidity (previous values: 0.22-0.35)
                #if (solidity < 0.19 or solidity > 0.35): continue

                cntsFiltered.append([cnt, cntArea])
                #end fingerprinting

            # We will work on the filtered contour with the largest area which is the
            # first one in the list
            if (len(cntsFiltered) > 0):
                #Used to hold the 4 contour Corners
                contourCorners = []

                
                cv2.line(image, (round(centerX), screenHeight), (round(centerX), 0), white, 2)

                #Found candidate contours
                #now find the following:
                #1. center of all candiates
                #2. average of the area

                #loop through candiates
                final_center = 0
                average_area = 0
                foundCorners = False

                # print(cntsFiltered)
                for i in range(1):
                    
                    cnt = cntsFiltered[i][0]
                    cntArea = cntsFiltered[i][1]

                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                    else:
                        cx = 0
                        cy = 0

                  # limit contour to quadrilateral
                    peri = cv2.arcLength(cnt, True)
                    corners = cv2.approxPolyDP(cnt, 0.04 * peri, True)
                    #print(corners)

                    if (len(corners) == 4):
                        foundCorners = True 
                        #print("found 4")
                        #print(corners)
                        contourCorners.append(corners)

                   # draw quadrilateral on input image from detected corners
                    #print(box)

                    for i in corners:
                        cv2.circle(image,(i[0][0],i[0][1]), 3, (0,255,0), -1)
                     #   print("x value",  i[0][0])
                    #box = np.int0(box)
                    #boxes.append(box)

                    #cv2.drawContours(image,[box],0,(36,255,12),2)
                    #cv2.fillPoly(mask, [box], (255,255,255))

                    # Find corners on the mask
                     #mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
                    #corners = cv2.goodFeaturesToTrack(mask, maxCorners=4, qualityLevel=0.5, minDistance=50)
                    
                    #displaycorners(image, corners)

                   # cv2.circle(image, extLeft, 8, (0, 0, 255), -1)
                   # cv2.circle(image, extRight, 8, (0, 255, 0), -1)
                   # cv2.circle(image, extTop, 8, (255, 0, 0), -1)
                   # cv2.circle(image, extBot, 8, (255, 255, 0), -1)

                    #Calculate the Center of each Contour                    
                    
                    final_center += cx

                    #find average Area
                    average_area += cntArea
                
                # final_center = round(final_center / len(cntsFiltered))
                average_area = average_area / len(cntsFiltered)

                #Add Array to go through 4 corners, find nearest contour to final_center

                #print("contourCorners:", len(contourCorners))
                #print("Average_AREA: ", average_area)

                H_FOCAL_LENGTH, V_FOCAL_LENGTH = calculateFocalLengthsFromInput(cameraFOV, screenWidth, screenHeight)

                YawToTarget = calculateYaw(final_center, centerX, H_FOCAL_LENGTH)

                #testing for overlay
                #fincen = getTargetCenterFromYaw(YawToTarget, centerX, H_FOCAL_LENGTH)
                #print("fincenter", final_center)
                #print("calcyAW: ", fincen)

                success = False

                if (foundCorners):
                     #displaycorners(image, outer_corners)

                     if len(contourCorners) > 2:
                        closestCorner = minContour(final_center, contourCorners)
                        #print("Closest Corner:", closestCorner)
                        pnpCorners = contourCorners[closestCorner]
                     else:
                        pnpCorners = contourCorners[0]
                
                     #print("pnpCorners:", pnpCorners[0][0])
                     # pnpCorners[0] = tuple(cnt[cnt[:,:,0].argmin()][0])
                
                    # unpack corners

                     corner = []

                     for i in pnpCorners:
                        corner.append((i[0][0],i[0][1]))
                
                     outer_corners = np.array([corner[0], corner[1], corner[2], corner[3]])
                    
                     #print("outer1: ", outer_corners)

                     outer_corners = order_points(outer_corners)
                     #print("outer order: ", outer_corners)

                    # outer_corners = np.array((pnpCorners[0][0],pnpCorners[0][1]), (pnpCorners[1][0],pnpCorners[1][1]), corner3, corner4)
                
                     #print("outer_corners", outer_corners)
                     #print("real_world_cordinates", real_world_coordinates)

                     #print("Final_Center: ", final_center)
                     #print("Average_AREA: ", average_area)

                     success, rvec,tvec  = findTvecRvec(image, outer_corners, real_world_coordinates,H_FOCAL_LENGTH,V_FOCAL_LENGTH) 
                    
                cv2.putText(image, "TargetYaw: " + str(YawToTarget), (20, 100), cv2.FONT_HERSHEY_COMPLEX, 0.5,white)
                #cv2.putText(image, "Average Area: " + str(average_area), (20, 240), cv2.FONT_HERSHEY_COMPLEX, 1.0,white)

                # If success then print values to screen                               
                if success:
                    distance, angle1, angle2 = compute_output_values(rvec, tvec, CameraTiltAngle)
                    poseX = tvec[0][0]
                    poseY = tvec[1][0]
                    poseZ = tvec[2][0]
                    
                    past_distances.append(distance)
                    if len(past_distances) > 5:
                        past_distances.pop(0)
                    
                    #print("past distances", past_distances)
                    average_distance = getAverageArray(past_distances)
                    #calculate RobotYawToTarget based on Robot offset (subtract 180 degrees)
                    RobotYawToTarget = 180-abs(angle2)
          
                    # cv2.putText(image, "RobotYawToTarget: " + str(round(RobotYawToTarget,2)), (20, 140), cv2.FONT_HERSHEY_COMPLEX, .6,white)
                    # cv2.putText(image, "SolvePnPTargetYawToCenter: " + str(round(angle1,2)), (20, 170), cv2.FONT_HERSHEY_COMPLEX, .6,white)
                    cv2.putText(image, "Distance: " + str(round((distance/12),2)), (20, 200), cv2.FONT_HERSHEY_COMPLEX, 0.5,white)
                    # cv2.putText(image, "Average Distance: " + str(round((average_distance/12),2)), (20, 230), cv2.FONT_HERSHEY_COMPLEX, 1.0,white)
                    # cv2.putText(image, "PoseX: " + str(round((poseX),2)), (20, 260), cv2.FONT_HERSHEY_COMPLEX, 1.0,white)
                    # cv2.putText(image, "PoseY: " + str(round((poseY),2)), (20, 290), cv2.FONT_HERSHEY_COMPLEX, 1.0,white)
                    # cv2.putText(image, "PoseZ: " + str(round((poseZ),2)), (20, 320), cv2.FONT_HERSHEY_COMPLEX, 1.0,white)                   

                #start with a non-existing colour
                
                # color 0 is red
                # color 1 is yellow
                # color 2 is green
                if (YawToTarget >= -2 and YawToTarget <= 2):
                    colour = green
                    #Use Bling
                    #Set Green colour
                   # if (blingColour != 2):
                   #     publishNumber("blingTable", "green",255)
                   #     publishNumber("blingTable", "blue", 0)
                   #     publishNumber("blingTable", "red", 0)
                   #     publishNumber("blingTable", "wait_ms",0)
                   #     publishString("blingTable","command","solid")
                   #     blingColour = 2
                if ((YawToTarget >= -5 and YawToTarget < -2) or (YawToTarget > 2 and YawToTarget <= 5)):  
                    colour = yellow
                    
                   # if (blingColour != 1):
                   #     publishNumber("blingTable", "red",255)
                   #     publishNumber("blingTable", "green",255)
                   #     publishNumber("blingTable", "blue",0)
                   #     publishNumber("blingTable", "wait_ms",0)
                   #     publishString("blingTable","command","solid")
                   #     blingColour = 1
                if ((YawToTarget < -5 or YawToTarget > 5)):  
                    colour = red
                   # if (blingColour != 0):
                   #     publishNumber("blingTable", "red",255)
                   #     publishNumber("blingTable", "blue",0)
                   #     publishNumber("blingTable", "green",0)
                   #     publishNumber("blingTable", "wait_ms",0)
                   #     publishString("blingTable","command","solid")
                   #     blingColour = 0

                cv2.line(image, (round(centerX), screenHeight), (round(centerX), 0), white, 2)
                cv2.line(image, (final_center, screenHeight), (final_center, 0), colour, 2)
                #print("Final_Center:",final_center)

                #publishResults(name,value)
                publishNumber(MergeVisionPipeLineTableName, "YawToTarget", YawToTarget)

                if success:
                    publishNumber(MergeVisionPipeLineTableName, "DistanceToTarget", round(distance/12,2))
                    publishNumber(MergeVisionPipeLineTableName, "AverageDistance", round(average_distance/12,2))
                    publishNumber(MergeVisionPipeLineTableName, "RobotYawToTarget", round(RobotYawToTarget,2))
                    publishNumber(MergeVisionPipeLineTableName, "TargetPixelFromCenter", round(final_center-centerX,2))
                    publishNumber(MergeVisionPipeLineTableName, "PoseX", round(poseX,2))
                    publishNumber(MergeVisionPipeLineTableName, "PoseY", round(poseY,2))
                    publishNumber(MergeVisionPipeLineTableName, "PoseZ", round(poseZ,2))


            else:
                #If Nothing is found, publish -99 and -1 to Network table
                publishNumber(MergeVisionPipeLineTableName, "YawToTarget", -99)
                publishNumber(MergeVisionPipeLineTableName, "DistanceToTarget", -1)  
                publishNumber(MergeVisionPipeLineTableName, "AverageDistance", -1)  
                publishNumber(MergeVisionPipeLineTableName, "RobotYawToTarget", -99)
                publishNumber(MergeVisionPipeLineTableName, "TargetPixelFromCenter", -99)
                publishNumber(MergeVisionPipeLineTableName, "PoseX", -99)
                publishNumber(MergeVisionPipeLineTableName, "PoseY", -99)
                publishNumber(MergeVisionPipeLineTableName, "PoseZ", -99)
                publishString("blingTable","command","clear")
                past_distances.clear()
                #print("past_distances are gone")

    else:
        #If Nothing is found, publish -99 and -1 to Network table
        publishNumber(MergeVisionPipeLineTableName, "YawToTarget", -99)
        publishNumber(MergeVisionPipeLineTableName, "DistanceToTarget", -1) 
        publishNumber(MergeVisionPipeLineTableName, "AverageDistance", -1)  
        publishNumber(MergeVisionPipeLineTableName, "RobotYawToTarget", -99) 
        publishNumber(MergeVisionPipeLineTableName, "TargetPixelFromCenter", -99)
        publishNumber(MergeVisionPipeLineTableName, "PoseX", -99)
        publishNumber(MergeVisionPipeLineTableName, "PoseY", -99)
        publishNumber(MergeVisionPipeLineTableName, "PoseZ", -99)
        
        publishString("blingTable","command","clear") 
        past_distances.clear()
        #print("past_distances are gone")
    #     # pushes vision target angle to network table
    return image, (final_center-centerX), YawToTarget, distance


# Checks if the target contours are worthy 
def checkTargetSize(cntArea, cntAspectRatio):
    #print("cntArea: " + str(cntArea))
    #print("aspect ratio: " + str(cntAspectRatio))
    #return (cntArea > image_width/3 and cntArea < MAXIMUM_TARGET_AREA and cntAspectRatio > 1.0)
    return (cntArea > image_width/3 and cntAspectRatio > 1.0)

import math
import numpy as np

from VisionConstants import *
from VisionMasking import *
from VisionUtilities import *
from DistanceFunctions import *

#from CornersVisual4 import get_four

try:
    from PrintPublisher import *
except ImportError:
    from NetworkTablePublisher import *


# real world dimensions of the goal target
# These are the full dimensions around both strips
TARGET_STRIP_LENGTH = 2.0    # inches
TARGET_STRIP_WIDTH = 4.0        # inches


real_world_coordinates = np.array([
    [-TARGET_STRIP_LENGTH / 2.0, TARGET_STRIP_WIDTH / 2.0, 0.0], # for top left corner
    [TARGET_STRIP_LENGTH / 2.0, TARGET_STRIP_WIDTH / 2.0, 0.0], # for top right
    [TARGET_STRIP_LENGTH / 2.0, -TARGET_STRIP_WIDTH / 2.0, 0.0], # for bottom right
    [-TARGET_STRIP_LENGTH / 2.0, -TARGET_STRIP_WIDTH / 2.0, 0.0], # for bottom left
])



MAXIMUM_TARGET_AREA = 4400

# Finds the tape targets from the masked image and displays them on original stream + network tales
def findTape(frame, cameraFOV, CameraTiltAngle, mask, MergeVisionPipeLineTableName, past_distances):

    
    # Taking a matrix of size 5 as the kernel 
    #kernel = np.ones((3,3), np.uint8) 
  
    # The first parameter is the original image, 
    # kernel is the matrix with which image is  
    # convolved and third parameter is the number  
    # of iterations, which will determine how much  
    # you want to erode/dilate a given image.  
    #img_erosion = cv2.erode(mask, kernel, iterations=1) 
    #mask = cv2.dilate(img_erosion, kernel, iterations=1) 
    #cv2.imshow("mask2", mask)

    # Finds contours
    # we are accomodating different versions of openCV and the different methods for corners
    if is_cv3():
       # if CornerMethod is 1 or CornerMethod is 2 or CornerMethod is 3:
       #     _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
       # elif CornerMethod is 4 or CornerMethod is 5:
       #     _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
       # elif CornerMethod is 6 or CornerMethod is 7:
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        #elif CornerMethod is 8 or CornerMethod is 9 or CornerMethod is 10:
        #    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
    else: #implies not cv3, either version 2 or 4
        #if CornerMethod is 1 or CornerMethod is 2 or CornerMethod is 3:
        #    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #elif CornerMethod is 4 or CornerMethod is 5:
        #    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
        #elif CornerMethod is 6 or CornerMethod is 7:
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        #elif CornerMethod is 8 or CornerMethod is 9 or CornerMethod is 10:
        #    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #else:
        #    pass

    contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

    # rotates frame 90 deg clockwise
    # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    # Take each frame
    # Gets the shape of video
    screenHeight, screenWidth, _ = frame.shape
    # Gets center of height and width
    centerX = (screenWidth / 2) - .5
    centerY = (screenHeight / 2) - .5
    # Copies frame and stores it in image
    image = frame.copy()
    # Processes the contours, takes in (contours, output_image, (centerOfImage)
    final_center = -99
    YawToTarget = -99
    distance = -1
    if len(contours) != 0:
        image, final_center, YawToTarget, distance = findTargets(contours, image, centerX, centerY, mask, MergeVisionPipeLineTableName, past_distances, cameraFOV, CameraTiltAngle)
    else:
        past_distances.clear()
        YawToTarget = -99
        final_center = -99
        distance = -1
        publishNumber(MergeVisionPipeLineTableName, "YawToTarget", -99)
        publishNumber(MergeVisionPipeLineTableName, "DistanceToTarget", -1)  
        publishNumber(MergeVisionPipeLineTableName, "AverageDistance", -1)  
    # Shows the contours overlayed on the original video
    return image, final_center, YawToTarget, distance




# Simple method to order points from left to right
def order_points(pts):
    # initialize a list of coordinates that will be ordered
    # such that the first entry in the list is the top-left,
    # the second entry is the top-right, the third is the
    # bottom-right, and the fourth is the bottom-left
    rect = np.zeros((4, 2), dtype="float32")
 
    # the top-left point will have the smallest sum, whereas
    # the bottom-right point will have the largest sum
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
 
    # now, compute the difference between the points, the
    # top-right point will have the smallest difference,
    # whereas the bottom-left will have the largest difference
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
 
    # return the ordered coordinates
    return rect

 #3D Rotation estimation
 
def findTvecRvec(image, outer_corners, real_world_coordinates, H_FOCAL_LENGTH, V_FOCAL_LENGTH):
    # Read Image
    size = image.shape
 
    # Camera internals
 
    focal_length = size[1]
    center = (size[1]/2, size[0]/2)
    camera_matrix = np.array(
                          [[H_FOCAL_LENGTH, 0, center[0]],
                          [0, V_FOCAL_LENGTH, center[1]],
                          [0, 0, 1]], dtype = "double"
                          )

    dist_coeffs = np.array([[0.16171335604097975, -0.9962921370737408, -4.145368586842373e-05, 
                             0.0015152030328047668, 1.230483016701437]])

    #print(camera_matrix)
    # This is tested for a 640x480 resolution
    # Should be 
    #     Fx  0  Cx
    #     0  Fy  Cy
    #     0   0  1
    #  Cx and Cy are close to center of image,  Fx and Fy are focal lengths in pixel units
    #camera_matrix = np.array([[676.9254672222575, 0.0, 303.8922263320326], 
    #                          [0.0, 677.958895098853, 226.64055316186037], 
    #                          [0.0, 0.0, 1.0]], dtype = "double")

    #print("Camera Matrix :\n {0}".format(camera_matrix))                           
 
    #dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
    (success, rotation_vector, translation_vector) = cv2.solvePnP(real_world_coordinates, outer_corners, camera_matrix, dist_coeffs)
 
    #print ("Rotation Vector:\n {0}".format(rotation_vector))
    #print ("Translation Vector:\n {0}".format(translation_vector))
    return success, rotation_vector, translation_vector


#Computer the final output values, 
#angle 1 is the Yaw to the target
#distance is the distance to the target
#angle 2 is the Yaw of the Robot to the target
def compute_output_values(rvec, tvec, cameraTiltAngle):
    '''Compute the necessary output distance and angles'''

    # The tilt angle only affects the distance and angle1 calcs
    # This is a major impact on calculations
    tilt_angle = math.radians(cameraTiltAngle)

    x = tvec[0][0]
    z = math.sin(tilt_angle) * tvec[1][0] + math.cos(tilt_angle) * tvec[2][0]

    # distance in the horizontal plane between camera and target
    distance = math.sqrt(x**2 + z**2)

    # horizontal angle between camera center line and target
    angleInRad = math.atan2(x, z)
    angle1 = math.degrees(angleInRad)

    rot, _ = cv2.Rodrigues(rvec)
    rot_inv = rot.transpose()
    pzero_world = np.matmul(rot_inv, -tvec)
    angle2InRad = math.atan2(pzero_world[0][0], pzero_world[2][0])
    angle2 = math.degrees(angle2InRad)

    # distance = 

    return distance, angle1, angle2

#Simple function that displays 4 corners on an image
#A np.array() is expected as the input argument
def displaycorners(image, outer_corners):
    # draw extreme points
    # from https://www.pyimagesearch.com/2016/04/11/finding-extreme-points-in-contours-with-opencv/
    if len(outer_corners) == 4: #this is methods 1 to 4 
        cv2.circle(image, (int(outer_corners[0,0]),int(outer_corners[0,1])), 6, green, -1)
        cv2.circle(image, (int(outer_corners[1,0]),int(outer_corners[1,1])), 6, red, -1)
        cv2.circle(image, (int(outer_corners[2,0]),int(outer_corners[2,1])), 6, white,-1)
        cv2.circle(image, (int(outer_corners[3,0]),int(outer_corners[3,1])), 6, blue, -1)
        #print('extreme points', leftmost,rightmost,topmost,bottommost)
    else: # this assumes len is 5 and method 5
        cv2.circle(image, (int(outer_corners[0,0]),int(outer_corners[0,1])), 6, green, -1)
        cv2.circle(image, (int(outer_corners[1,0]),int(outer_corners[1,1])), 6, blue, -1)
        cv2.circle(image, (int(outer_corners[2,0]),int(outer_corners[2,1])), 6, purple, -1)
        cv2.circle(image, (int(outer_corners[3,0]),int(outer_corners[3,1])), 6, white,-1)
        cv2.circle(image, (int(outer_corners[4,0]),int(outer_corners[4,1])), 6, red, -1)

# Function that takes a list of 3 corners of a contour and gives you the closest to the center

def getAverageArray(array):
    values = 0
    final_value = 0
    
    if len(array) > 0:
        for i in range(len(array)):
            values += array[i]
        final_value = values/len(array)
        #except ValueError:
        #print("Something is wrong with one of these values:", array)
    
    return final_value 

# gets minimum contour from a two dimensional array to find closest corner to centerpoint out of a set of arrays
def minContour(center, contourCorners):
    xDiff = [] # list that stores the differences between number (final_center) and contourCorners
    minVar = 10000 # will change to the greatest value that shows up
    closestCorner = 0 # set closestCorner to the value of i (so 1-4 in most cases)

    for i in range(len(contourCorners)): # two for loops as contourCorners is a 2D array
        for j in contourCorners[i]:
            #xDiff.insert([center-(i[0][0]), center-(i[1][0]), center-(i[1][0]), center-(i[1][1])])
            xDiff.append(abs(center-(j[0][0]))) # append the differences between number (final_center) and j[0][0] and add to xDiff
            var = min(xDiff)
            if var < minVar : # if var the value (min(xDiff) which is an array with 3+ elements) is greater than current minVar, then set minVar to var, 
                minVar = var # set minVar to var, so minVar becomes the highest value
                closestCorner = i # closest corner will be the number of the element of contourCorners where it is closest to center
            #print("i[0][0] data: ", j[0][0])

    #print("xDiff: ", xDiff)
    #print("len of contourCorners: ", len(contourCorners[0]))

    #smallestDiff = min(xDiff)
    return closestCorner

# Draws Contours and finds center and yaw of vision targets
# centerX is center x coordinate of image
# centerY is center y coordinate of image
# Draws Contours and finds center and yaw of vision targets
# centerX is center x coordinate of image
# centerY is center y coordinate of image

def findTape(contours, image, centerX, centerY, mask, MergeVisionPipeLineTableName, past_distances, cameraFOV, CameraTiltAngle):
    global blingColour
    #global warped
    screenHeight, screenWidth, channels = image.shape
    # Seen vision targets (correct angle, adjacent to each other)
    # targets = []
    # Constant used as minimum area for fingerprinting is equal to 60% of screenWidth. (Using 
    # a value based on screenWidth scales properly if the resolution ever changes.)
    # minContourArea = 0.6 * screenWidth

    final_center = -99
    YawToTarget = -99
    distance = -1

    if len(contours) >= 1:
        # Sort contours by area size (biggest to smallest)
        cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)[:10]
       
        cntsFiltered = []

        # First contour has largest area, so only go further if that one meets minimum area criterion
        if cntsSorted:

            for (j, cnt) in enumerate(cntsSorted):

                # Calculate Contour area
                cntArea = cv2.contourArea(cnt)

                # rotated rectangle fingerprinting
               # bounding = cv2.boundingRect(cnt)
                rect = cv2.minAreaRect(cnt)

               #rect = cv2.boundingRect(cnt)
                (xr,yr),(wr,hr),ar = rect #x,y width, height, angle of rotation = rotated rect

               # print("Area: " + ar)

                x, y, w, h = cv2.boundingRect(cnt)
                #cv2.rectangle(image,(x,y),(x+w,y+h),(0, 255, 255),1)
                # cv2.imshow("result",image)

                percentfill = (cntArea/(w*h))

                print("cntArea: " , cntArea)
                print("percent fill: ",(cntArea/(w*h)) )

                total_screen_pixels = screenHeight * screenWidth
                percent_contour_area = cntArea/total_screen_pixels *100

                #Filter based on too large contour 
                # if cntArea > 2500 or cntArea < 440: continue
                print('percent contour:', percent_contour_area)
 
                #Filter based on too small contour
                if (percent_contour_area < 0.05): continue

                print("percentfill:", percentfill)
                # Filter based on percent fill
                if (percentfill < 0.5): continue 

                # Filter based on Angle of rotation 

                #print("AR:" , ar)
                #print("X: " , x)
                #print("Y: " , y)
                #print("W: " , w)
                #print("H: " , h)

                #to get rid of height and width switching
                if hr > wr: 
                    ar = ar + 90
                    wr, hr = [hr, wr]
                else:
                    ar = ar + 180
                if ar == 180:
                    ar = 0

                if hr == 0: continue
                cntMinAreaAR = float(wr)/hr
                cntBoundRectAR = float(w)/h

                print ("cntBoundRectAR: " , cntBoundRectAR)
                print ("cntMinAreaAR: " , cntMinAreaAR)

                # Filter based on aspect ratio (previous values: 2-3)
                #Tape is 13 cm wide by 5 cm high - that gives aspect ration of 2.6
                #To be flexible, lets accept (1.9 - 3.3) range 
                print("aspect ratio:", cntBoundRectAR)
                if (cntBoundRectAR < 0.2 or cntBoundRectAR > 0.8): continue 

                cv2.rectangle(image,(x,y),(x+w,y+h),(0, 0, 255),1)
                
                #minAextent = float(cntArea)/(wr*hr)

                # Hull
                #hull = cv2.convexHull(cnt)
                #hull_area = cv2.contourArea(hull)
                #solidity = float(cntArea)/hull_area

               
                # Filter based on minimum area extent (previous values: 0.16-0.26)
                #if (minAextent < 0.139 or minAextent > 1.1): continue
               
                # Filter based on solidity (previous values: 0.22-0.35)
                #if (solidity < 0.19 or solidity > 0.35): continue

                cntsFiltered.append([cnt, cntArea])
                #end fingerprinting

            # We will work on the filtered contour with the largest area which is the
            # first one in the list
            if (len(cntsFiltered) > 0):
                print("hi")
                #Used to hold the 4 contour Corners
                contourCorners = []

                
                cv2.line(image, (round(centerX), screenHeight), (round(centerX), 0), white, 2)

                #Found candidate contours
                #now find the following:
                #1. center of all candiates
                #2. average of the area

                #loop through candiates
                final_center = 0
                average_area = 0
                foundCorners = False

                # print(cntsFiltered)
                for i in range(1):
                    
                    cnt = cntsFiltered[i][0]
                    cntArea = cntsFiltered[i][1]

                    print("first target: ", cnt)
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                    else:
                        cx = 0
                        cy = 0
                    print(cx)

                  # limit contour to quadrilateral
                    peri = cv2.arcLength(cnt, True)
                    corners = cv2.approxPolyDP(cnt, 0.04 * peri, True)
                    #print(corners)

                    if (len(corners) == 4):
                        foundCorners = True 
                        #print("found 4")
                        #print(corners)
                        contourCorners.append(corners)

                   # draw quadrilateral on input image from detected corners
                    #print(box)

                    for i in corners:
                        cv2.circle(image,(i[0][0],i[0][1]), 3, (0,255,0), -1)
                     #   print("x value",  i[0][0])
                    #box = np.int0(box)
                    #boxes.append(box)

                    #cv2.drawContours(image,[box],0,(36,255,12),2)
                    #cv2.fillPoly(mask, [box], (255,255,255))

                    # Find corners on the mask
                     #mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
                    #corners = cv2.goodFeaturesToTrack(mask, maxCorners=4, qualityLevel=0.5, minDistance=50)
                    
                    #displaycorners(image, corners)

                   # cv2.circle(image, extLeft, 8, (0, 0, 255), -1)
                   # cv2.circle(image, extRight, 8, (0, 255, 0), -1)
                   # cv2.circle(image, extTop, 8, (255, 0, 0), -1)
                   # cv2.circle(image, extBot, 8, (255, 255, 0), -1)

                    #Calculate the Center of each Contour                    
                    
                    final_center += cx
                    print(final_center)

                    #find average Area
                    average_area += cntArea
                
                # final_center = round(final_center / len(cntsFiltered))
                average_area = average_area / len(cntsFiltered)

                #Add Array to go through 4 corners, find nearest contour to final_center

                #print("contourCorners:", len(contourCorners))
                #print("Average_AREA: ", average_area)

                H_FOCAL_LENGTH, V_FOCAL_LENGTH = calculateFocalLengthsFromInput(cameraFOV, screenWidth, screenHeight)

                YawToTarget = calculateYaw(final_center, centerX, H_FOCAL_LENGTH)

                #testing for overlay
                #fincen = getTargetCenterFromYaw(YawToTarget, centerX, H_FOCAL_LENGTH)
                #print("fincenter", final_center)
                #print("calcyAW: ", fincen)

                success = False

                if (foundCorners):
                     #displaycorners(image, outer_corners)

                     if len(contourCorners) > 2:
                        closestCorner = minContour(final_center, contourCorners)
                        #print("Closest Corner:", closestCorner)
                        pnpCorners = contourCorners[closestCorner]
                     else:
                        pnpCorners = contourCorners[0]
                
                     #print("pnpCorners:", pnpCorners[0][0])
                     # pnpCorners[0] = tuple(cnt[cnt[:,:,0].argmin()][0])
                
                    # unpack corners

                     corner = []

                     for i in pnpCorners:
                        corner.append((i[0][0],i[0][1]))
                
                     outer_corners = np.array([corner[0], corner[1], corner[2], corner[3]])
                    
                     #print("outer1: ", outer_corners)

                     outer_corners = order_points(outer_corners)
                     #print("outer order: ", outer_corners)

                    # outer_corners = np.array((pnpCorners[0][0],pnpCorners[0][1]), (pnpCorners[1][0],pnpCorners[1][1]), corner3, corner4)
                
                     #print("outer_corners", outer_corners)
                     #print("real_world_cordinates", real_world_coordinates)

                     #print("Final_Center: ", final_center)
                     #print("Average_AREA: ", average_area)

                     success, rvec, tvec = findTvecRvec(image, outer_corners, real_world_coordinates,H_FOCAL_LENGTH,V_FOCAL_LENGTH) 
                    
                cv2.putText(image, "TargetYaw: " + str(YawToTarget), (20, 100), cv2.FONT_HERSHEY_COMPLEX, 1.0,white)
                #cv2.putText(image, "Average Area: " + str(average_area), (20, 240), cv2.FONT_HERSHEY_COMPLEX, 1.0,white)

                # If success then print values to screen                               
                if success:
                    distance, angle1, angle2 = compute_output_values(rvec, tvec, CameraTiltAngle)
                    
                    past_distances.append(distance)
                    if len(past_distances) > 5:
                        past_distances.pop(0)
                    
                    #print("past distances", past_distances)
                    average_distance = getAverageArray(past_distances)
                    #calculate RobotYawToTarget based on Robot offset (subtract 180 degrees)
                    RobotYawToTarget = 180-abs(angle2)
          
                    cv2.putText(image, "RobotYawToTarget: " + str(round(RobotYawToTarget,2)), (20, 140), cv2.FONT_HERSHEY_COMPLEX, .6,white)
                    cv2.putText(image, "SolvePnPTargetYawToCenter: " + str(round(angle1,2)), (20, 170), cv2.FONT_HERSHEY_COMPLEX, .6,white)
                    cv2.putText(image, "Distance: " + str(round((distance/12),2)), (20, 200), cv2.FONT_HERSHEY_COMPLEX, 1.0,white)
                    cv2.putText(image, "Average Distance: " + str(round((average_distance/12),2)), (20, 230), cv2.FONT_HERSHEY_COMPLEX, 1.0,white)
                   

                #start with a non-existing colour
                
                # color 0 is red
                # color 1 is yellow
                # color 2 is green
                if (YawToTarget >= -2 and YawToTarget <= 2):
                    colour = green
                    #Use Bling
                    #Set Green colour
                   # if (blingColour != 2):
                   #     publishNumber("blingTable", "green",255)
                   #     publishNumber("blingTable", "blue", 0)
                   #     publishNumber("blingTable", "red", 0)
                   #     publishNumber("blingTable", "wait_ms",0)
                   #     publishString("blingTable","command","solid")
                   #     blingColour = 2
                if ((YawToTarget >= -5 and YawToTarget < -2) or (YawToTarget > 2 and YawToTarget <= 5)):  
                    colour = yellow
                    
                   # if (blingColour != 1):
                   #     publishNumber("blingTable", "red",255)
                   #     publishNumber("blingTable", "green",255)
                   #     publishNumber("blingTable", "blue",0)
                   #     publishNumber("blingTable", "wait_ms",0)
                   #     publishString("blingTable","command","solid")
                   #     blingColour = 1
                if ((YawToTarget < -5 or YawToTarget > 5)):  
                    colour = red
                   # if (blingColour != 0):
                   #     publishNumber("blingTable", "red",255)
                   #     publishNumber("blingTable", "blue",0)
                   #     publishNumber("blingTable", "green",0)
                   #     publishNumber("blingTable", "wait_ms",0)
                   #     publishString("blingTable","command","solid")
                   #     blingColour = 0

                cv2.line(image, (round(centerX), screenHeight), (round(centerX), 0), white, 2)
                cv2.line(image, (final_center, screenHeight), (final_center, 0), colour, 2)
                #print("Final_Center:",final_center)

                #publishResults(name,value)
                publishNumber(MergeVisionPipeLineTableName, "YawToTarget", YawToTarget)

                if success:
                    publishNumber(MergeVisionPipeLineTableName, "DistanceToTarget", round(distance/12,2))
                    publishNumber(MergeVisionPipeLineTableName, "AverageDistance", round(average_distance/12,2))
                    publishNumber(MergeVisionPipeLineTableName, "RobotYawToTarget", round(RobotYawToTarget,2))
                    publishNumber(MergeVisionPipeLineTableName, "TargetPixelFromCenter", round(final_center-centerX,2))

            else:
                #If Nothing is found, publish -99 and -1 to Network table
                publishNumber(MergeVisionPipeLineTableName, "YawToTarget", -99)
                publishNumber(MergeVisionPipeLineTableName, "DistanceToTarget", -1)  
                publishNumber(MergeVisionPipeLineTableName, "AverageDistance", -1)  
                publishNumber(MergeVisionPipeLineTableName, "RobotYawToTarget", -99)
                publishNumber(MergeVisionPipeLineTableName, "TargetPixelFromCenter", -99)
                publishString("blingTable","command","clear")
                past_distances.clear()
                #print("past_distances are gone")

    else:
        #If Nothing is found, publish -99 and -1 to Network table
        publishNumber(MergeVisionPipeLineTableName, "YawToTarget", -99)
        publishNumber(MergeVisionPipeLineTableName, "DistanceToTarget", -1) 
        publishNumber(MergeVisionPipeLineTableName, "AverageDistance", -1)  
        publishNumber(MergeVisionPipeLineTableName, "RobotYawToTarget", -99) 
        publishNumber(MergeVisionPipeLineTableName, "TargetPixelFromCenter", -99)
        publishString("blingTable","command","clear") 
        past_distances.clear()
        #print("past_distances are gone")
    #     # pushes vision target angle to network table
    return image, (final_center-centerX), YawToTarget, distance


# Checks if the target contours are worthy 
def checkTargetSize(cntArea, cntAspectRatio):
    #print("cntArea: " + str(cntArea))
    #print("aspect ratio: " + str(cntAspectRatio))
    #return (cntArea > image_width/3 and cntArea < MAXIMUM_TARGET_AREA and cntAspectRatio > 1.0)
    return (cntArea > image_width/3 and cntAspectRatio > 1.0)
