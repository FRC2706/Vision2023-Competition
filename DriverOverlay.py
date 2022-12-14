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


# A simple calculation that scales an X position value based on the X point of the
# current camera and an alternate Camera
def calculateResolutionDifferential(final_center, driverCamWidth, AltCamWidth):
    #calculate final_center for simple resolution difference
    scaledCenter = (final_center * (driverCamWidth/AltCamWidth))
    return scaledCenter

# A simple Driver Overlay.  Requires 
#   frame:  The frame (image) which will be overlayed
#   cameraFOV:  The Field of view of the current camera type
#   final center which is the center of the object being targeted which may have been
#                calcualted using a different field of view.
#   Yaw to Target, the Yaw value to the target calculated using the alternate camera with
#           an alternate field of view
#   The Distance to the target (for display purposes)

def DriverOverlay(frame, cameraFOV, OverlayScaleFactor, TargetPixelFromCenter,YawToTarget, distance):
  
    # Take each frame
    # Gets the shape of video
    screenHeight, screenWidth, _ = frame.shape
    # Gets center of height and width
    centerX = (screenWidth / 2) - .5
    centerY = (screenHeight / 2) - .5
    # Copies frame and stores it in image
    image = frame.copy()

    #final_center = -99

    #print("harcoded focalLength:",H_FOCAL_LENGTH)

    #How to calculate scaling factor:
    #It takes into affect the FOV of the two cameras as well as resolution difference
    # We are given TargetPixelFromCenter which is the difference between the center of the 
    # screen and the target x position from the original camera
    # The OverlayScaleFactor accounts for the difference in camera resolution and Field of View
    # It is calculated as follows:
    # H_Camera1_FOV      H_Camera2_RES (eg 640) 
    # -------------  X   ---------------------- 
    # H_Camera2_FOV      H_Camera1_RES (eg 1280)   
    # We multiply the TargetPixelFromCenter by the OverlayScaleFactor, which will give us the
    # TargetPixelFromCenter on the new Camera.   Now all that is need is to add the center X
    # of the current camera.

    if (YawToTarget != -99):
        final_center = (TargetPixelFromCenter*OverlayScaleFactor)+centerX
        #print("TargetPixelFromCenter: ",TargetPixelFromCenter) 
        #print("CenterX: ",centerX)
        #print("final_center: ",final_center) 
  
    #Now read Distance and Yaw from Network tables
    #if final_center != -99:
        cv2.line(image, (round(centerX), screenHeight), (round(centerX), 0), white, 2)
        if (YawToTarget >= -2 and YawToTarget <= 2):
           colour = green
        if ((YawToTarget >= -6 and YawToTarget < -2) or (YawToTarget > 2 and YawToTarget <= 6)):  
           colour = yellow
        if ((YawToTarget < -6 or YawToTarget > 6)):  
           colour = red
        cv2.line(image, (round(final_center), screenHeight), (round(final_center), 0), colour, 2)

    #if YawToTarget != -99:        
        cv2.putText(image, "Yaw: " + str(YawToTarget), (20, 100), cv2.FONT_HERSHEY_COMPLEX, 0.8,white)

    if distance != -1:    
        cv2.putText(image, "Distance: " + str(round((distance),2)), (20, 200), cv2.FONT_HERSHEY_COMPLEX, 0.8,white)

  
    # Shows the contours overlayed on the original video
    return image

