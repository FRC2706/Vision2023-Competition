# ----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.

# My 2020 license: use it as much as you want. Crediting is recommended because it lets me know 
# that I am being useful.
# Some parts of pipeline are based on 2019 code created by the Screaming Chickens 3997

# This is meant to be used in conjuction with WPILib Raspberry Pi image: https://github.com/wpilibsuite/FRCVision-pi-gen
# ----------------------------------------------------------------------------

# import the necessary packages
import datetime
import json
import time
import sys
import random
import cv2
import math
import os
import sys

import numpy as np
from imutils.video import WebcamVideoStream

from threading import Thread

# Imports EVERYTHING from these files


from VisionConstants import *
from VisionUtilities import *
from VisionMasking import *
from DistanceFunctions import *
from DriverOverlay import *
from DetectIntakeItem import *
from FindCube import *
from FindCone import *
from FindTape import *
from FindAprilTagRobotpy import *

print()
print("--- Merge Viewer Starting ---")
print()
print("Using python version {0}".format(sys.version))
print()
print('OpenCV Version = ', cv2.__version__)
print()

###################### PROCESSING OPENCV ################################

# CHOOSE VIDEO OR FILES HERE!!!!
# boolean for video input, if true does video, if false images
useVideo = False
# integer for usb camera to use, boolean for live webcam
useWebCam = False
webCamNumber = 1

# ADJUST DESIRED TARGET BASED ON VIDEO OR FILES ABOVE !!!
Tape = False
Cone = False
Cube = False
Intake = True
AprilTag = False
CameraFOV = 68.5
CameraTiltAngle = 30
OverlayScaleFactor = 1

# counts frames for writing images
frameStop = 0
ImageCounter = 0
showAverageFPS = False

#Code to load images from a folder
def load_images_from_folder(folder):
    images = []
    imagename = []
    for filename in sorted(os.listdir(folder)):
        #print(filename)
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
            images.append(img)
            imagename.append(filename)
    return images, imagename

def draw_circle(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN:
        green = np.uint8([[[frame[y, x, 0], frame[y, x, 1], frame[y, x, 2]]]])
        #print(frame[y, x, 2], frame[y, x, 1], frame[y, x, 0], cv2.cvtColor(green,cv2.COLOR_BGR2HSV))  

# choose video to process -> Outer Target Videos
#videoname = './OuterTargetVideos/ThirdScale-01.mp4'
videoname = './OuterTargetVideos/FullScale-02.mp4'

if useVideo: # test against video
    showAverageFPS = True

elif useWebCam: #test against live camera
    showAverageFPS = True

else:  # implies images are to be read

    # Outer Target Images
    #images, imagename = load_images_from_folder("./2023VisionSampleImages/RetroTape")
    #images, imagename = load_images_from_folder("/Users/johngray/FRC/Vision2023-Competition/2023VisionSampleImages/RetroTape")
    images, imagename = load_images_from_folder("./2023VisionSampleImages/AprilTags")
    #images, imagename = load_images_from_folder("./HubImgSketchup")
    print(imagename)


    # finds height/width of camera frame (eg. 640 width, 480 height)
    image_height, image_width = images[0].shape[:2]
    #print(image_height, image_width)

team = 2706
server = True
MergeVisionPipeLineTableName = "DummyNetworkTableName"
cameraConfigs = []

if useVideo and not useWebCam:
    cap = cv2.VideoCapture(videoname)

elif useWebCam:
    # src defines which camera, assume 2nd camera or src=1
    vs = WebcamVideoStream(src=webCamNumber).start()

else:
    currentImg = 0
    imgLength = len(images)

print("Hello Vision Team!")

stayInLoop = True

#Setup variables for average framecount
frameCount = 0
averageTotal = 0
averageFPS = 0

framePSGroups = 50
displayFPS = 3.14159265

# start
#fps = FPS().start()
begin = milliSince1970()
start = begin
prev_update = start

# past_distances test
past_distances = []

while stayInLoop or cap.isOpened():

    if useVideo and not useWebCam:
        (ret, frame) = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame, likely end of file, Exiting ...")
            stayInLoop = False
            break

    elif useWebCam:
        frame = vs.read()

    else:
        frame = images[currentImg]
        filename = imagename[currentImg]
        print("Filename is: ", filename )

    processed = frame

    if Tape:
        threshold = threshold_video(lower_green, upper_green, frame)
        processed, TargetPixelFromCenter, YawToTarget, distance = findTargets(frame, CameraFOV, CameraTiltAngle, threshold, MergeVisionPipeLineTableName, past_distances)
    
    if Intake:
        processed = DetectIntakeItem(frame, MergeVisionPipeLineTableName)
        #processed = findCone(frame, MergeVisionPipeLineTableName,CameraFOV)
        #processed, yaw = findCube(frame, MergeVisionPipeLineTableName,CameraFOV)

    if Cone:
        processed = findCone(frame, MergeVisionPipeLineTableName,CameraFOV)

    if Cube: 
        processed, yaw = findCube(frame, MergeVisionPipeLineTableName,CameraFOV)

    if AprilTag:
        processed = FindAprilTagRobotpy(frame, MergeVisionPipeLineTableName )

       # if AprilTag:
        #    findAprilTag(frame, CameF)

           

    # end of cycle so update counter
    #fps.update()
    # in merge view also end of time we want to measure so stop FPS
    #fps.stop()
    frameCount = frameCount+1
    update = milliSince1970()

    processedMilli = (update-prev_update)
    averageTotal = averageTotal+(processedMilli)
    prev_update = update

    if ((frameCount%30)==0.0):
        averageFPS = (1000/((update-begin)/frameCount))

    if frameCount%framePSGroups == 0.0:
        # also end of time we want to measure so stop FPS
        stop = milliSince1970()  
        displayFPS = (stop-start)/framePSGroups
        start = milliSince1970()

    # because we are timing in this file, have to add the fps to image processed 
    #cv2.putText(processed, 'elapsed time: {:.2f}'.format(fps.elapsed()), (40, 40), cv2.FONT_HERSHEY_COMPLEX, 0.6 ,white)
    #cv2.putText(processed, 'FPS: {:.7f}'.format(3.14159265), (40, 80), cv2.FONT_HERSHEY_COMPLEX, 0.6 ,white)
    
    #cv2.putText(processed, "frame time: " + str(int(processedMilli)) + " ms", (40, 40), cv2.FONT_HERSHEY_COMPLEX, 0.6 ,white)
    #cv2.putText(processed, 'Instant FPS: {:.2f}'.format(1000/(processedMilli)), (40, 80), cv2.FONT_HERSHEY_COMPLEX, 0.6 ,white)
    
    if (showAverageFPS): 
        cv2.putText(processed, 'Grouped FPS: {:.2f}'.format(1000/(displayFPS)), (20, 20), cv2.FONT_HERSHEY_COMPLEX, 0.6 ,white)
        cv2.putText(processed, 'Average FPS: {:.2f}'.format(averageFPS), (20, 50), cv2.FONT_HERSHEY_COMPLEX, 0.6 ,white)
    #else:
        #cv2.putText(processed, 'Grouped FPS: {:.2f}'.format(1000/(displayFPS)), (20, 20), cv2.FONT_HERSHEY_COMPLEX, 0.6 ,white)

    cv2.imshow("raw", frame)
    cv2.setMouseCallback('raw', draw_circle)

    if useVideo or useWebCam:
        cv2.imshow('videoname', processed)

        key = cv2.waitKey(1)
        if key == 27:
            break

    else:
        cv2.imshow(filename, processed)

        # wait for user input to move or close
        key = cv2.waitKeyEx(0)

        print('you pressed this code->', key)

        if key == 113 or key == 27: # this is the escape key
            stayInLoop = True
            break
        if key == 105 or key == 2490368: # this is the up arrow, and key 'i'
            currentImg = currentImg - 1
            if currentImg < 0: 
                currentImg = imgLength - 1
        if key == 109 or key == 2621440: # this is the down arrow, and key 'm'
            currentImg = currentImg + 1
            if currentImg > imgLength - 1:
                currentImg = 0

        #destroy old window
        cv2.destroyWindow(filename)
        filename = imagename[currentImg]

    # end while
# end if

if useVideo and not useWebCam:
    cap.release()
elif useWebCam:
    vs.stop()
else:
    pass

cv2.destroyAllWindows()