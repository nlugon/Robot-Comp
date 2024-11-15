
import numpy as np
import cv2 as cv
import cv2
import sys
import matplotlib.pyplot as plt
from pupil_apriltags import Detector
import time
from utils import *


at_detector = Detector(
   families="tag36h11",
   nthreads=1,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0
)


def detectQR(img):
    
   if(len(img.shape) > 2):
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
   start = time.time()
   a = at_detector.detect(img)
#    print(time.time()-start)

   if a is None or len(a) == 0 or a[0] is None or a[0].corners is None:
      return False, None

   corners = np.array(a[0].corners)
   #show the corner on the image
   resolution = img.shape

   # print(f"Corner 1: {corners[0]}, Corner 2: {corners[1]}, Corner 3: {corners[2]}, Corner 4: {corners[3]}")

   for i in range(len(corners)):
      cv2.circle(img, (int(corners[i][0]), int(corners[i][1])), 5, (0, 0, 255), -1)

   X = np.mean(corners[:,0])
   Y = np.mean(corners[:,1])
   W = np.max(corners[:,0]) - np.min(corners[:,0])
   H = np.max(corners[:,1]) - np.min(corners[:,1])

   buttonObjective = {
         "X": -X,
         "Y": Y,
         "WB": W,
         "HB": H,
         "W": resolution[0], 
         "H": resolution[1], 
         "name" : "aligningButton",
   }

   buttonObjective = translateXYToAngleDistance(buttonObjective)

   print(f"Button: {buttonObjective}")

   return True,buttonObjective
   