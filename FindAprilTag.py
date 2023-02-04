#use findtape to find distance

from FindTarget import*
from pupil_apriltags import Detector
import cv2
import numpy as np
from math import degrees

try:
    from PrintPublisher import *
except ImportError:
    from NetworkTablePublisher import *

# Marker size and object
marker_size = 5 + 15/16.0   # FRC targets might be a different size

# Data about the marker for solvePnP's SOLVEPNP_IPPE_SQUARE stuff
object_points = []
object_points.append( [float(-marker_size / 2),float(marker_size / 2), 0])
object_points.append( [float(marker_size / 2),float(marker_size / 2), 0])
object_points.append(  [float(marker_size / 2),float(-marker_size / 2), 0])
object_points.append(  [float(-marker_size / 2),float(-marker_size / 2), 0])
object_points = np.array(object_points)

detector = Detector(
   families="tag16h5",
   nthreads=1,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.5,
)
#main function 
def findAprilTagCorner(image, cameraFOV, CameraTiltAngle,MergeVisionPipeLineTableName):
    apriltag_outer_corner(image, cameraFOV, CameraTiltAngle,MergeVisionPipeLineTableName)
    #print("length: ",np.size(corners))
    return image

def calc_distance(image, cameraFOV, corners, CameraTiltAngle):
        size = image.shape
        focal_length = size[1]
        center = (size[1]/2, size[0]/2)
        screenHeight, screenWidth, _ = image.shape


        H_FOCAL_LENGTH, V_FOCAL_LENGTH = calculateFocalLengthsFromInput(cameraFOV, screenWidth, screenHeight)

        #define camera matrix
        camera_matrix = np.array(
                            [[H_FOCAL_LENGTH, 0, center[0]],
                            [0, V_FOCAL_LENGTH, center[1]],
                            [0, 0, 1]], dtype = "double"
                            )

        #define camera distortion
        dist_coeffs = np.array([[0.16171335604097975, -0.9962921370737408, -4.145368586842373e-05, 
                                0.0015152030328047668, 1.230483016701437]])

        pnpsuccess, rvec, tvec = cv2.solvePnP(object_points, corners, camera_matrix, dist_coeffs, flags = cv2.SOLVEPNP_IPPE_SQUARE)
        s, rvec, tvec = findTvecRvec(image, corners, real_world_coordinates, H_FOCAL_LENGTH, V_FOCAL_LENGTH)
        distance, angle1, angle2 = compute_output_values(rvec, tvec, CameraTiltAngle)
        return distance, angle1


def apriltag_outer_corner(image, cameraFOV, CameraTiltAngle,MergeVisionPipeLineTableName):

    detector = Detector(
        families="tag16h5",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.5,
    )
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    results = detector.detect(gray)

    
    corners = np.empty
    #print("before results")
    min_distance = -1

    for r in results:

        if (r.hamming == 0):
            print("-------------------------------------")
            print(r)
            print("-------------------------------------")
            (ptA, ptB, ptC, ptD) = r.corners
        
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))

        # draw the bounding box of the AprilTag detection
            cv2.line(image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(image, ptD, ptA, (0, 255, 0), 2)
            cv2.line(image, ptD, ptA, (0, 255, 0), 2)
            
        # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        
        # ERIK: Tag family should contain a string, "36h11" which is the family of the apriltag. 
        # FIRST says we should only have 36h11 tags so might be a good check to see if the target is good.
        
            tagFamily = r.tag_family.decode("utf-8")
        
        # Put the text for the id of the tag
            cv2.putText(image, f"id: {r.tag_id}", (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            corners = np.array(r.corners)
            if np.size(corners) >= 4:
                distance, yaw = calc_distance(image, cameraFOV, corners, CameraTiltAngle)
                if min_distance == -1:
                    min_distance = distance
                else:
                    if distance < min_distance:
                        min_distance = distance

                cv2.putText(image, f"ft: {round(distance/12, 2)}", (ptA[0], ptA[1] + 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                cv2.putText(image, f"yaw: {round(yaw, 2)}", (ptA[0], ptA[1] + 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                  
                #cv2.putText(image, f"d: " {distance}, (ptA[0], ptA[1] + 15),
                #cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Get the corners of the target in a numpy array for solvePnP

            else:
                print("hello world")
                distance = -1 
    print("Ft:", round(min_distance/12, 2))
    # publishNumber(MergeVisionPipeLineTableName, "DistanceToTarget", round(distance/12,2))
                
                
        