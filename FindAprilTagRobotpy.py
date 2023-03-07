#use findtape to find distance

#from FindTape import*
#from apriltag import apriltag 
#from pupil_apriltags import Detector
import robotpy_apriltag
from wpimath.geometry import Transform3d
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

# This is the main function initiated from MergeViewer and Merge2023Pipeline
def findAprilTag(frame, MergeVisionPipeLineTableName):
     #screenHeight, screenWidth, _ = frame.shape
     detector, estimator = get_apriltag_detector_and_estimator(frame.shape)
     frame, tag_id, tvec, center = detect_and_process_apriltag(frame, detector, estimator, MergeVisionPipeLineTableName)

     return frame, tag_id, tvec, center

# This function is called once to initialize the apriltag detector and the pose estimator
def get_apriltag_detector_and_estimator(frame_size):
    detector = robotpy_apriltag.AprilTagDetector()
    # FRC 2023 uses tag16h5 (game manual 5.9.2)
    assert detector.addFamily("tag16h5")
    estimator = robotpy_apriltag.AprilTagPoseEstimator(
    robotpy_apriltag.AprilTagPoseEstimator.Config(
            0.2, 500, 500, frame_size[1] / 2.0, frame_size[0] / 2.0
        )
    )
    return detector, estimator

# This function is called for every detected tag. It uses the `estimator` to 
# return information about the tag, including its centerpoint. (The corners are 
# also available.)
def process_apriltag(estimator, tag):
    tag_id = tag.getId()
    center = tag.getCenter()
    hamming = tag.getHamming()
    decision_margin = tag.getDecisionMargin()
    # print("Hamming for {} is {} with decision margin {}".format(tag_id, hamming, decision_margin))

    est = estimator.estimateOrthogonalIteration(tag, 50)
    pose = est.pose1
    t = pose.translation()
    tvec = [t.x, t.y, t.z]
    # x is left/right
    # y is up/down
    # z is front/back

    return tag_id, tvec, center, hamming

# This simply outputs some information about the results returned by `process_apriltag`.
# It prints some info to the console and draws a circle around the detected center of the tag
def draw_tag(frame, result):
    assert frame is not None
    assert result is not None
    tag_id, pose, center = result
    # print(center)
    cv2.circle(frame, (int(center.x), int(center.y)), 50, (255, 0, 255), 3)
    msg = f"Tag ID: {tag_id} Pose: {pose}"
    cv2.putText(frame, msg, (100, 50 * 1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    return frame


# This function is called once for every frame captured by the Webcam. For testing, it can simply
# be passed a frame capture loaded from a file. (See commented-out alternative `if __name__ == main:` at bottom of file)
def detect_and_process_apriltag(frame, detector, estimator, MergeVisionPipeLineTableName):
    assert frame is not None
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect apriltag
    tag_info = detector.detect(gray)
    DETECTION_MARGIN_THRESHOLD = 100
    filter_tags = [tag for tag in tag_info if tag.getDecisionMargin() > DETECTION_MARGIN_THRESHOLD and tag.getHamming() == 0]

    if len(filter_tags) > 0:
        z_to_tag = {}

        for tag in filter_tags:
            
            result = process_apriltag(estimator, tag)
            # print(result)

            tag_id = result[0]
            tvec = result[1]
            center = result[2]

            z = abs(tvec[2])
            z_to_tag[z] = [tag_id, tvec, center]
            

            # lowest abs of z

            # Draw a frame around the tag:
            col_box = (0,0,255)
            col_txt = (0,255,255)
            corner0 = (int(tag.getCorner(0).x), int(tag.getCorner(0).y))
            corner1 = (int(tag.getCorner(1).x), int(tag.getCorner(1).y))
            corner2 = (int(tag.getCorner(2).x), int(tag.getCorner(2).y))
            corner3 = (int(tag.getCorner(3).x), int(tag.getCorner(3).y))
            cv2.line(frame, corner0, corner1, color = col_box, thickness = 2)
            cv2.line(frame, corner1, corner2, color = col_box, thickness = 2)
            cv2.line(frame, corner2, corner3, color = col_box, thickness = 2)
            cv2.line(frame, corner3, corner0, color = col_box, thickness = 2)

            # Label the tag with the ID:
            cv2.putText(frame, f"{tag_id}", (int(center.x), int(center.y)), cv2.FONT_HERSHEY_SIMPLEX, 1, col_txt, 2)

        min_z = min(z_to_tag.keys())
        closest_tag = z_to_tag[min_z]

        tag_id, tvec, center = closest_tag

        publishNumber(MergeVisionPipeLineTableName, "TagId", tag_id)
        publishNumber(MergeVisionPipeLineTableName, "PoseX", round(tvec[0],4))
        publishNumber(MergeVisionPipeLineTableName, "PoseY", round(tvec[1],4))
        publishNumber(MergeVisionPipeLineTableName, "PoseZ", round(tvec[2],4))

    
    else:
        publishNumber(MergeVisionPipeLineTableName, "TagId", -1)
        publishNumber(MergeVisionPipeLineTableName, "PoseX", -99)
        publishNumber(MergeVisionPipeLineTableName, "PoseY", -99)
        publishNumber(MergeVisionPipeLineTableName, "PoseZ", -99)

    return frame, -1, -1, -1
    

    # publish values to network table
    """
    publishNumber(MergeVisionPipeLineTableName, "DistanceToAprilTag", distance)
    publishNumber(MergeVisionPipeLineTableName, "YawToAprilTag", yaw)
    publishNumber(MergeVisionPipeLineTableName, "TagID", tag_id) 
    """
