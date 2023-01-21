from FindTarget import*
from pupil_apriltags import Detector
import cv2
import numpy as np
from math import degrees

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
#father function 
def findAprilTagCorner(image, cameraFOV):
 gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
 
 results = detector.detect(gray)
 screenHeight, screenWidth, _ = image.shape
 _FOCAL_LENGTH, V_FOCAL_LENGTH = calculateFocalLengthsFromInput(cameraFOV, screenWidth, screenHeight)
 for r in results:
    
    if (r.hamming == 0):
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
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Get the corners of the target in a numpy array for solvePnP
        corners = np.array(r.corners)

        pnpsucsess, rvec, tvec = cv2.solvePnP(object_points, corners, camera_matrix, camera_distortion, flags = cv2.SOLVEPNP_IPPE_SQUARE)

 s, rvec, tvec = findTvecRvec(image, outer_corners, real_world_coordinates, H_FOCAL_LENGTH, V_FOCAL_LENGTH)
 