#!/usr/bin/env python3

# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import json
import time
import sys
from threading import Thread
import random

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from ntcore import NetworkTableInstance, EventFlags
import cv2
import numpy as np
import math
import datetime

from FindTape import *
from FindAprilTag import *
from FindAprilTagRobotpy import *
from FindCube import *
from FindCone import *
from DetectIntakeItem import *
from VisionConstants import *
from VisionUtilities import *
from VisionMasking import *
from DistanceFunctions import *
from DriverOverlay import *

print("Using python version {0}".format(sys.version))
print()
print('OpenCV version is', cv2.__version__)
print()

# class that runs separate thread for showing video,
class VideoShow:
    """
    Class that continuously shows a frame using a dedicated thread.
    """

    def __init__(self, imgWidth, imgHeight, cameraServer, frame=None):
        self.outputStream = cameraServer.putVideo(OutputStream, imgWidth, imgHeight)
        #self.outputStream = cameraServer.putVideo("2706_out", imgWidth, imgHeight)
        #OutputStream
        self.frame = frame
        self.stopped = False

    def start(self):
        Thread(target=self.show, args=()).start()
        return self

    def show(self):
        while not self.stopped:
            self.outputStream.putFrame(self.frame)

    def stop(self):
        self.stopped = True

    def notifyError(self, error):
        self.outputStream.notifyError(error)


# Class that runs a separate thread for reading  camera server also controlling exposure.
class WebcamVideoStream:
    def __init__(self, camera, cameraServer, frameWidth, frameHeight, name="WebcamVideoStream"):
        # initialize the video camera stream and read the first frame
        # from the stream

        # Automatically sets exposure to 0 to track tape
        self.webcam = camera
     
        # Some booleans so that we don't keep setting exposure over and over to the same value
        self.autoExpose = True
        self.prevValue = True

        self.switchBall = False
        self.switchTape = False
        
        # Make a blank image to write on
        self.img = np.zeros(shape=(frameWidth, frameHeight, 3), dtype=np.uint8)
        # Gets the video
        self.stream = cameraServer.getVideo()
        (self.timestamp, self.img) = self.stream.grabFrame(self.img)

        # initialize the thread name
        self.name = name

        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target=self.update, name=self.name, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            global switch
            if self.stopped:
                return

            if switch == 1: #driver mode
                self.autoExpose = True
                ##print("Driver mode")
                if self.autoExpose != self.prevValue:
                    self.webcam.setExposureManual(60)
                    self.webcam.setExposureManual(39)
                    self.webcam.setExposureAuto()
                    ##print("Driver mode")
                    self.prevValue = self.autoExpose
             
            elif switch == 2: #Tape Target Mode - set manual exposure to 20
                #self.autoExpose = False
                #self.switchTape = True
                #if self.autoExpose != self.prevValue:
                if self.switchTape != True:
                    self.webcam.setExposureManual(60)
                    self.webcam.setExposureManual(ExposureTape)
                    self.switchTape = True
                    self.switchBall = False
                    #self.prevValue = self.autoExpose

            elif switch == 3: #Cargo Mode - set exposure to 39
                #self.autoExpose = False
                #if self.autoExpose != self.prevValue:
                if self.switchBall != True:
                    self.webcam.setExposureManual(ExposureBall)
                    self.webcam.setExposureManual(39)
                    self.webcam.setExposureAuto()
                    self.switchBall = True
                    self.switchTape = False
                    #self.prevValue = self.autoExpose

            # gets the image and timestamp from cameraserver
            (self.timestamp, self.img) = self.stream.grabFrame(self.img)

    def read(self):
        # return the frame most recently read
        return self.timestamp, self.img

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

    def getError(self):
        return self.stream.getError()



#   JSON format:
#   {
#       "team": <team number>,
#       "ntmode": <"client" or "server", "client" if unspecified>
#       "cameras": [
#           {
#               "name": <camera name>
#               "path": <path, e.g. "/dev/video0">
#               "pixel format": <"MJPEG", "YUYV", etc>   // optional
#               "width": <video mode width>              // optional
#               "height": <video mode height>            // optional
#               "fps": <video mode fps>                  // optional
#               "brightness": <percentage brightness>    // optional
#               "white balance": <"auto", "hold", value> // optional
#               "exposure": <"auto", "hold", value>      // optional
#               "properties": [                          // optional
#                   {
#                       "name": <property name>
#                       "value": <property value>
#                   }
#               ],
#               "stream": {                              // optional
#                   "properties": [
#                       {
#                           "name": <stream property name>
#                           "value": <stream property value>
#                       }
#                   ]
#               }
#           }
#       ]
#       "switched cameras": [
#           {
#               "name": <virtual camera name>
#               "key": <network table key used for selection>
#               // if NT value is a string, it's treated as a name
#               // if NT value is a double, it's treated as an integer index
#           }
#       ]
#   }

###################### PROCESSING OPENCV ################################

# counts frames for writing images
frameStop = 0
ImageCounter = 0

# Set Default to find the Tape target
#switch = 2

# Masks the video based on a range of hsv colors
# Takes in a frame, range of color, and a blurred frame, returns a masked frame
def threshold_video(lower_color, upper_color, blur):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    h = threshold_range(h, lower_color[0], upper_color[0])
    s = threshold_range(s, lower_color[1], upper_color[1])
    v = threshold_range(v, lower_color[2], upper_color[2])
    combined_mask = cv2.bitwise_and(h, cv2.bitwise_and(s,v))
    
    # hold the HSV image to get only red colors
    #mask = cv2.inRange(combined, lower_color, upper_color)

    # Returns the masked imageBlurs video to smooth out image
    global frameStop
    if frameStop == 1:
        global ImageCounter, matchNumber, matchNumberDefault
        matchNumber = networkTableMatch.getNumber("MatchNumber", 0)
        if matchNumber == 0:
            matchNumber = matchNumberDefault
        cv2.imwrite('/mnt/VisionImages/visionImg-' + str(matchNumber) + "-" + str(ImageCounter) + '_mask.png',
                    combined_mask)
    return combined_mask

#################### FRC VISION PI Image Specific #############
configFile = "/boot/frc.json"

pipelineConfig = "pipelineConfig.json"

with open(pipelineConfig) as json_file:
    data = json.load(json_file)

MergeVisionPipeLineTableName = data["networkTableName"]
MergeVisionReadPipeLineTableName = data["networkTableReadName"]
TapeEnabled = data["Tape"]
Intake = data["Intake"]
AprilTagsEnabled = data["AprlTag"]
OutputStream = data["OutputStream"]
ExposureTape = data["ExposureTape"]
CameraFOV = data["CameraFOV"]
CameraTiltAngle = data["CameraTiltAngle"]
OverlayScaleFactor = data["OverlayScaleFactor"]
OutputStreamWidth = data["OutputStreamWidth"]
OutputStreamHeight = data["OutputStreamHeight"]

if Intake:
    switch = 1

elif TapeEnabled:
    switch = 2

elif AprilTagsEnabled:
    switch = 3


class CameraConfig: pass

team = 2706
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

def readSwitchedCameraConfig(config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False

    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    camera = UsbCamera(config.name, config.path)
    server = CameraServer.startAutomaticCapture(camera=camera)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera

def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.addSwitchedCamera(config.name)

    def listener(event):
        data = event.data
        if data is not None:
            value = data.value.value()
            if isinstance(value, int):
                if value >= 0 and value < len(cameras):
                    server.setSource(cameras[value])
            elif isinstance(value, float):
                i = int(value)
                if i >= 0 and i < len(cameras):
                    server.setSource(cameras[i])
            elif isinstance(value, str):
                for i in range(len(cameraConfigs)):
                    if value == cameraConfigs[i].name:
                        server.setSource(cameras[i])
                        break

    NetworkTableInstance.getDefault().addListener(
        NetworkTableInstance.getDefault().getEntry(config.key),
        EventFlags.kImmediate | EventFlags.kValueAll,
        listener)

    return server

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

        # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
       # Name of network table - this is how it communicates with robot. IMPORTANT
   
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("wpilibpi")
        ntinst.setServerTeam(team)
        ntinst.startDSClient()
   
   
    networkTableMatch = ntinst.getTable("FMSInfo")
    networkTableTime = ntinst.getTable("SmartDashboard")
    networkTableMatchVariables = ntinst.getTable("VisionControl")

    #Used to control MergeVisionPipeLineSettings
    networkTableVisionPipeline = ntinst.getTable(MergeVisionPipeLineTableName)

    #NetworkTable to read from if needed
    networkTableVisionReadPipeline = ntinst.getTable(MergeVisionReadPipeLineTableName)
    networkTableMatchVariables.putBoolean("StartUp",False)
    networkTableMatchVariables.putBoolean("ShutDown",False)

    #PipeLine Table Values, Unique for Each PipeLine

    networkTableVisionPipeline.putBoolean("Tape", TapeEnabled)
    networkTableVisionPipeline.putBoolean("AprilTag", AprilTagsEnabled)
    networkTableVisionPipeline.putBoolean("Intake", Intake)
    #networkTable.putBoolean("ControlPanel", False)
    networkTableVisionPipeline.putBoolean("WriteImages", False)
    networkTableVisionPipeline.putBoolean("SendMask", False)
    #networkTable.putBoolean("Aligned", False)
    networkTableVisionPipeline.putValue("OverlayScaleFactor",OverlayScaleFactor)

    matchNumberDefault = random.randint(1, 1000)
    processed = 0

    #Setup variables for average framecount
    frameCount = 0
    averageTotal = 0
    averageFPS = 0

    framePSGroups = 50
    displayFPS = 3.14159265

    # start frames per second outside loop, will stop and restart every framePSGroups
    #fps = FPS().start()
    begin = milliSince1970()
    start = begin
    prev_update = start

    #Make sure Start and Stop images only publish network table values once
    startedImageWrite = False
    stoppedImageWrite = False


    #initialize the past distances
    past_distances = []

    # start cameras
    for config in cameraConfigs:
        cameras.append(startCamera(config))

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)

    currentCam = 0

    webcam = cameras[currentCam]
    cameraServer = streams[currentCam]
    # Start thread reading camera    

    cap = WebcamVideoStream(webcam, cameraServer, image_width, image_height).start()    

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(image_height, image_width, 3), dtype=np.uint8)
    # Start thread outputing stream
    streamViewer = VideoShow(image_width, image_height, cameraServer, frame=img).start()


    # loop forever
    while True:
         if (startedImageWrite == False and networkTableMatchVariables.getBoolean("StartUp",False)):
            startedImageWrite = True
            networkTableVisionPipeline.putBoolean("WriteImages", True)

         if (stoppedImageWrite == False and networkTableMatchVariables.getBoolean("ShutDown",False)):
            stoppedImageWrite = True
            networkTableVisionPipeline.putBoolean("WriteImages", False)     


        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
         timestamp, img = cap.read()
         if frameStop == 0:
            matchNumber = networkTableMatch.getNumber("MatchNumber", 0)
            if matchNumber == 0:
                matchNumber = matchNumberDefault
            cv2.imwrite('/mnt/VisionImages/visionImg-' + str(matchNumber) + "-" + str(ImageCounter) + '_Raw.png',
                        img)
        # Uncomment if camera is mounted upside down
         if networkTableVisionPipeline.getBoolean("TopCamera", False):
            frame = flipImage(img)
         else:
            frame = img
        # Comment out if camera is mounted upside down
    
        
         if timestamp == 0:
            # Send the output the error.
            streamViewer.notifyError(cap.getError())
            # skip the rest of the current iteration
            continue
        # Checks if you just want camera for driver (No processing), False by default

        #switch = 2

        

        #Check if Network Table value Tape is True
         if (networkTableVisionPipeline.getBoolean("Tape", True)):
            switch = 2
            #Method = int(networkTableVisionPipeline.getNumber("Method", 7))
            threshold = threshold_video(lower_green, upper_green, frame)
            if (networkTableVisionPipeline.getBoolean("SendMask", False)):
                processed = threshold
            else:    
                processed, final_center, YawToTarget, distance = findTargets(frame, CameraFOV, CameraTiltAngle, threshold, MergeVisionPipeLineTableName, past_distances)

                #Read RPM From Network Table
                rpm = networkTableVisionPipeline.getNumber("RPM", 0)
                if rpm != 0:
                    cv2.putText(processed, "RPM: " + str(round(rpm,2)), (20, 340), cv2.FONT_HERSHEY_COMPLEX, 1.0,white)

         if (networkTableVisionPipeline.getBoolean("Driver", True)):
            switch = 1
            
            TargetPixelFromCenter = networkTableVisionReadPipeline.getNumber("TargetPixelFromCenter", -99)
            yaw = networkTableVisionReadPipeline.getNumber("YawToTarget", -99)
            distance = networkTableVisionReadPipeline.getNumber("DistanceToTarget", -1)
            NTOverlayScaleFactor = networkTableVisionReadPipeline.getValue("OverlayScaleFactor",OverlayScaleFactor)
            
            processed = DriverOverlay(frame, CameraFOV, NTOverlayScaleFactor, TargetPixelFromCenter, yaw, distance)
           

      
         if (networkTableVisionPipeline.getBoolean("Cargo", True)):
            # Checks if you just want to look for Cargo
            switch = 3
#               boxBlur = blurImg(frame, yellow_blur)
#               threshold = threshold_video(lower_yellow, upper_yellow, boxBlur)
            if (networkTableVisionPipeline.getBoolean("Red", True)):
                boxBlur = blurImg(frame, red_blur)
                threshold = threshold_video(lower_red, upper_red, boxBlur)
                processed = findCargo(frame, CameraFOV, threshold, MergeVisionPipeLineTableName)
            elif (networkTableVisionPipeline.getBoolean("Blue", True)):
                boxBlur = blurImg(frame, blue_blur)
                threshold = threshold_video(lower_blue, upper_blue, boxBlur)

            if (networkTableVisionPipeline.getBoolean("SendMask", False)):
                processed = threshold
            else:   
                processed = findCargo(frame, CameraFOV, threshold, MergeVisionPipeLineTableName)

           
                          

        # Puts timestamp of camera on network tables
         networkTableVisionPipeline.putNumber("VideoTimestamp", timestamp)

         if (networkTableVisionPipeline.getBoolean("WriteImages", True)):
            frameStop = frameStop + 1
            if frameStop == 15 :
                matchNumber = networkTableMatch.getNumber("MatchNumber", 0)
                if matchNumber == 0:
                    matchNumber = matchNumberDefault
                cv2.imwrite('/mnt/VisionImages/visionImg-' +str(matchNumber)+"-"+ str(ImageCounter) + '_Proc.png', processed)
                frameStop = 0
                ImageCounter = ImageCounter+1
                if (ImageCounter==10000):
                    ImageCounter=0

        # end of cycle so update counter
        #fps.update()
         frameCount = frameCount+1
         update = milliSince1970()        
 
         processedMilli = (update-prev_update)
         averageTotal = averageTotal+(processedMilli)
         prev_update = update

         if ((frameCount%30)==0.0):
            averageFPS = (1000/((update-begin)/frameCount))

        # only update FPS in groups according to framePSGroups
         if frameCount%framePSGroups == 0.0:
             # also end of time we want to measure so stop FPS
            stop = milliSince1970()  
            displayFPS = (stop-start)/framePSGroups
            start = milliSince1970()

        # because we are timing in this file, have to add the fps to image processed 
        #if (displayFPS != 0):
            #print(displayFPS)
            #cv2.putText(processed, 'Grouped FPS: {:.2f}'.format(1000/displayFPS), (20, 20), cv2.FONT_HERSHEY_COMPLEX, 0.6 ,white)

        #if (averageFPS != 0):    
            #cv2.putText(processed, 'Average FPS: {:.2f}'.format(averageFPS), (20, 50), cv2.FONT_HERSHEY_COMPLEX, 0.6 ,white)

        # Resize stream based on the type of stream
         if (OutputStreamWidth != 0):
            processed = cv2.resize(processed,(OutputStreamWidth,OutputStreamHeight),fx=0,fy=0,interpolation=cv2.INTER_CUBIC)
            streamViewer.frame = processed

        # Flushes camera values to reduce latency
         ntinst.flush()

    # end of while true
# end of main
# end of file

