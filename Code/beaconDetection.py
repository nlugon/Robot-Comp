import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import time
import os
import random

from utils import *


# The goal is to detect the beacons, there are 4 beacons in each corners of the arena, 
# the camera is using a 360 degree kogeto lens, so the camera can see the whole arena
# the beacons are neons, there are pink, red, green and blue 
# on the kogeto lens, we see them as lines pointing to the center of the arena
# the goal is to get the angle of the line


def getAngleFrom3Points(A,B,C):
    """
    calcules the angle between 3 points, using the middle point as the reference (called B)

    --------------------------------------------------------------
    Input:
        A (float,float), [m]
        B (float,float), [m]
        C (float,float), [m]

    --------------------------------------------------------------
    Output: 
        angle (float), [rad]
    
    """
    
    # B is the middle point
    line1 = [A[0] - B[0], A[1] - B[1]]
    line2 = [C[0] - B[0], C[1] - B[1]]

    # calculate the angle with arctan
    angle = - np.arctan2(line1[1], -line1[0]) + np.arctan2(line2[1], -line2[0])
    # angle = np.arccos(np.dot(line1, line2) / (np.linalg.norm(line1) * np.linalg.norm(line2))) #2nd method, using alkashif formula

    return angle

def getLine(currentImg, middle):
    """
    detects the longest line pointing to the middle of the image

    --------------------------------------------------------------
    Input:
        currentImg (np.ndarray)
        middle (float,float), [m]

    --------------------------------------------------------------
    Output: 
        maxLine (float,float,float,float), [m]
        currentImg (np.ndarray)
    
    """

    # get the edges of the image
    edges = cv.Canny(currentImg, 50, 150, apertureSize=3)

    # use houghlines to detect the lines
    lines = cv.HoughLinesP(edges, 2, np.pi/360, 10, minLineLength=10, maxLineGap=10)

    maxLine = None

    # draw the lines on the image
    if lines is not None:
        # take the longest line
        maxLine = None
        for line in lines:
            for x1, y1, x2, y2 in line:

                

                angle = getAngleFrom3Points([x2,y2],[x1,y1],middle)
                angle = angle22pi(angle)

                threshold = np.pi / 4

                # only keep the line pointing to the middle(wihtin a 10 degree range)
                if abs(angle) < threshold or abs(angle - np.pi) < threshold or abs(angle - 2 * np.pi) < threshold: 
                    # cv.putText(currentImg, str(round(angle * rad2deg, 2)), (x1, y1), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)
                    # cv.line(currentImg, (x1, y1), (x2, y2), (255, 0, 255), 2)

                    # if the line is longer than the maxLine, replace it
                    if maxLine is None:
                        maxLine = line
                    elif np.linalg.norm([x2-x1,y2-y1]) > np.linalg.norm([maxLine[0][2]-maxLine[0][0],maxLine[0][3]-maxLine[0][1]]):
                        maxLine = line
                else:
                    # cv.circle(currentImg, (x1, y1), 5, (0, 0, 255), 1)
                    # cv.line(currentImg, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    # cv.putText(currentImg, str(round(angle * rad2deg, 2)), (x1, y1), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv.LINE_AA)
                    pass
                
        # draw the longest line
        if maxLine is not None:
            for x1, y1, x2, y2 in maxLine:
                pass
                # cv.line(currentImg, (x1, y1), (x2, y2), (255, 0, 0), 2)
                # line1 = [x2 - x1, y2 - y1]
                # line2 = [middle[0] - x1, middle[1] - y1]

                # angle = np.arccos(np.dot(line1, line2) / (np.linalg.norm(line1) * np.linalg.norm(line2)))

                # cv.putText(currentImg, str(round(angle, 2)), (x1, y1), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)

    return maxLine,currentImg


def getAngleFromLine(line,middle):
    """
    Get the angle to the middle from the line data

    --------------------------------------------------------------
    Input:
        line (float,float,float,float), [m]
        middle (float,float), [m]

    --------------------------------------------------------------
    Output: 
        angle (float), [rad]
    
    """

    X = Y = 0

    for x1, y1, x2, y2 in line:
        X = x1 if (abs(x1-middle[0]) > abs(x2-middle[0])) else x2
        Y = y1 if (abs(y1-middle[1]) > abs(y2-middle[1])) else y2

    # calculate the angle with the middle

    angle = getAngleFrom3Points([X,Y],middle,[0,middle[1]])

    return angle

def getRange(Channel, lower, upper, maxVal = 180,mask = None):
    """
    Make the mask for the hue layer, with the given range, range can be from 0 to 360
    Range can be overlapping (lower > upper) like for red (0-30 and 330-360)

    --------------------------------------------------------------
    Input:
        Channel (np.ndarray)
        lower (int), [0-360]
        upper (int), [0-360]
        maxVal (int), [0-360]

    --------------------------------------------------------------
    Output: 
        variable (type) [unit]
    
    """

    if mask is None:
        mask = np.ones(Channel.shape, dtype=np.uint8)*255

    if lower < upper:
        return cv.bitwise_and(cv.inRange(Channel, lower, upper),mask)
    else:
        return cv.bitwise_and(cv.bitwise_or(cv.inRange(Channel, lower, maxVal),cv.inRange(Channel, 0, upper)),mask) 


def calcAnglesFromImage(img, debug = False, showPlot = False, putPointsOnImage = True, sendImage = False):
    """
    Main function that takes an image and returns the angles of the lines, and depending on the parameters, shows the image with the lines and the angles
    What the function does in order is :

        o Colors:
            - crop the image to the circle, the inside and outside circles prevent any lights from the ceiling and reflections from the floor to be detected
            - convert the image to HSV, and split the channels
            - make the mask for saturation and brightness, absolute and or mask
            - make the mask for the red, green and blue colors
            - combine all the masks 

        o Lines:
            - find the contours in the image with the canny edge detector
            - find the lines in the contours with the Hough transform
            - find the longest line that points to the middle of the image
            - calculate the angle from the longest line to the middle of the image

        o Plot:
            - plot the image with the lines and the angles if needed

    --------------------------------------------------------------
    Input:
        img (np.ndarray)
        debug (bool)
        showPlot (bool)
        putPointsOnImage (bool)
        sendImage (bool)

    --------------------------------------------------------------
    Output: 
        img (np.ndarray)
        angles (float,float,float,float), [rad]
    
    """
    
    time1 = time.time()
    
    middle = [1010,550] #x,y
    
    outsideRadius = 280
    insideRadius = outsideRadius - 60

    #points to crop the image
    p1 = [middle[0] - outsideRadius, middle[1] - outsideRadius]#[680,322]
    p2 = [middle[0] + outsideRadius, middle[1] + outsideRadius]#[1240,869]

    #crop the image if the intervals are not null
    if p2[0] < img.shape[0] or p2[1] < img.shape[1]:
        img = img[p1[1]:p2[1],p1[0]:p2[0]]

        middle = [outsideRadius,outsideRadius]
    
    else:
        
        size = min(img.shape[0:1])

        img = img[0:size,0:size]
        middle = [int(size/2),int(size/2)]
    
    imgCenter = middle
    imgRGB = img
    imgRGB = cv.cvtColor(imgRGB, cv.COLOR_BGR2RGB)

    coef170 = 170/360

    #pink thresholds for Hue 
    pinkLower = 270 * coef170
    pinkUpper = 320 * coef170

    #red thresholds for Hue
    redLower = 320 * coef170
    redUpper = 25 * coef170

    #green thresholds for Hue
    greenLower = 66 * coef170
    greenUpper = 190 * coef170

    #blue thresholds for Hue
    blueLower = 199 * coef170
    blueUpper = 259 * coef170

    absMinSaturation = 20
    absMinLuminosity = 100


    orMinSaturation = 150
    orMinLuminosity = 200

    
    # put the image in HSL color space
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    SChannel = hsv[:,:,1]
    LChannel = hsv[:,:,2]
    HChannel = hsv[:,:,0]

    #make a mask for the circle, all the pixels not in the circle are not in the mask
    radiusMask = np.zeros(img.shape[:2], dtype=np.uint8)
    cv.circle(radiusMask, tuple(imgCenter), outsideRadius, 255, -1)
    cv.circle(radiusMask, tuple(imgCenter), insideRadius, 0, -1)
    
    # put a threshold on the image, the image needs to keep only the colored parts of the image
    # the threshold is based on the saturation luminosity and the or mask

    # Absolute minimum variables
    absSaturationMask = getRange(SChannel, absMinSaturation, 255)
    absLuminosityMask = getRange(LChannel, absMinLuminosity, 255)

    # Or mask variables
    orSaturationMask = getRange(SChannel, orMinSaturation, 255)
    orLuminosityMask = getRange(LChannel, orMinLuminosity, 255)

    # Final mask, Absolute minimum and Or mask, see report for more information
    finalMask = cv.bitwise_and(radiusMask, cv.bitwise_or(cv.bitwise_and(absSaturationMask, orLuminosityMask), cv.bitwise_and(absLuminosityMask, orSaturationMask)))

    #put the L and H at the max value
    LChannel2 = np.ones(LChannel.shape)*255
    SChannel2 = np.ones(SChannel.shape)*255

    hsv2 = hsv.copy()
    hsv2[:,:,1] = SChannel2
    hsv2[:,:,2] = LChannel2

    #show the saturated image
    hueImg = cv.cvtColor(hsv2, cv.COLOR_HSV2RGB)
    valueImg = cv.cvtColor(LChannel, cv.COLOR_GRAY2RGB)
    saturationImg = cv.cvtColor(SChannel, cv.COLOR_GRAY2RGB)

    #make a mask for each color
    pinkMask = getRange(HChannel, pinkLower, pinkUpper, mask = finalMask)
    redMask = getRange(HChannel, redLower, redUpper, mask = finalMask)
    greenMask = getRange(HChannel, greenLower, greenUpper, mask = finalMask)
    blueMask = getRange(HChannel, blueLower, blueUpper, mask = finalMask)

    # take the image and put the masks on it
    pinkImg = cv.bitwise_and(imgRGB, imgRGB, mask=pinkMask)
    redImg = cv.bitwise_and(imgRGB, imgRGB, mask=redMask)
    greenImg = cv.bitwise_and(imgRGB, imgRGB, mask=greenMask)
    blueImg = cv.bitwise_and(imgRGB, imgRGB, mask=blueMask)
    
    time2 = time.time()

    pinkLine,pinkImg = getLine(pinkImg, middle=middle)
    redLine,redImg = getLine(redImg, middle=middle)
    greenLine,greenImg = getLine(greenImg, middle=middle)
    blueLine,blueImg = getLine(blueImg, middle=middle)

    time3 = time.time()

    #calculate the angles for each colors
    if pinkLine is not None:
        pinkAngle = getAngleFromLine(pinkLine, middle)
        if debug:
            cv.putText(pinkImg, str(pinkAngle * rad2deg), (pinkLine[0][0],pinkLine[0][1]), cv.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 2, cv.LINE_AA)
    else:
        pinkAngle = None

    if redLine is not None:
        redAngle = getAngleFromLine(redLine, middle)
        if debug:
            cv.putText(redImg, str(redAngle * rad2deg), (redLine[0][0],redLine[0][1]), cv.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv.LINE_AA)
    else:
        redAngle = None

    if greenLine is not None:
        greenAngle = getAngleFromLine(greenLine, middle)
        if debug:
            cv.putText(greenImg, str(greenAngle * rad2deg), (greenLine[0][0],greenLine[0][1]), cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2, cv.LINE_AA)
    else:
        greenAngle = None

    if blueLine is not None:
        blueAngle = getAngleFromLine(blueLine, middle)
        if debug:
            cv.putText(blueImg, str(blueAngle * rad2deg), (blueLine[0][0],blueLine[0][1]), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv.LINE_AA)
    else:
        blueAngle = None

    # real algo finished, now we just need to show the images / plot them -----------------------------------------------------------------------------

    #show all the masks in a subplot
    if showPlot:
        plt.subplot(3,3,1);plt.imshow(pinkImg, cmap = 'gray')
        plt.title('Pink Mask'); plt.xticks([]); plt.yticks([])
        plt.subplot(3,3,2);plt.imshow(greenImg, cmap = 'gray')
        plt.title('Green Mask'); plt.xticks([]); plt.yticks([])
        plt.subplot(3,3,3);plt.imshow(blueImg, cmap = 'gray')
        plt.title('Blue Mask'); plt.xticks([]); plt.yticks([])
        plt.subplot(3,3,4);plt.imshow(redImg, cmap = 'gray')
        plt.title('Red Mask'); plt.xticks([]); plt.yticks([])
        plt.subplot(3,3,5);plt.imshow(hsv, cmap = 'gray')
        plt.title('HSV'); plt.xticks([]); plt.yticks([])
        plt.subplot(3,3,6);plt.imshow(imgRGB, cmap = 'gray')
        plt.title('Original Image'); plt.xticks([]); plt.yticks([])
        plt.subplot(3,3,7);plt.imshow(hueImg, cmap = 'gray')
        plt.title('Hue Image'); plt.xticks([]); plt.yticks([])
        plt.subplot(3,3,8);plt.imshow(saturationImg, cmap = 'gray')
        plt.title('Saturation Image'); plt.xticks([]); plt.yticks([])
        plt.subplot(3,3,9);plt.imshow(valueImg, cmap = 'gray')
        plt.title('Value Image'); plt.xticks([]); plt.yticks([])

        # pause the program until the window is closed
        plt.show(block=True)
        plt.pause(0.01)

    time4 = time.time()

    if putPointsOnImage: 
        #put all the lines on the img
        img = cv.cvtColor(img, cv.COLOR_RGB2BGR)
        
        # put a circle for the inside and outside radius
        img = cv.circle(img, tuple(imgCenter), outsideRadius, (255,255,0), 2)
        img = cv.circle(img, tuple(imgCenter), insideRadius, (255,255,0), 2)

        # put a small point at the center of the image
        img = cv.circle(img, tuple(imgCenter), 2, (0,255,0), 2)
        
        textSize = 0.8
        textOffset = (-20,20)
        
        if pinkLine is not None:
            for x1,y1,x2,y2 in pinkLine:
                # cv.line(img, (x1,y1), (x2,y2), (255,0,0), 4)
                cv.circle(img, (x1,y1), 4, (255,0,255), 4)
            cv.putText(img, f"{pinkAngle * rad2deg:3.0f}", (pinkLine[0][0]+textOffset[0],pinkLine[0][1]+textOffset[1]), cv.FONT_HERSHEY_SIMPLEX, textSize, (255,0,255), 2, cv.LINE_AA) 
        if greenLine is not None:
            for x1,y1,x2,y2 in greenLine:
                # cv.line(img, (x1,y1), (x2,y2), (255,0,0), 4)
                cv.circle(img, (x1,y1), 4, (0,255,0), 4)
            cv.putText(img, f"{greenAngle * rad2deg:3.0f}", (greenLine[0][0]+textOffset[0],greenLine[0][1]+textOffset[1]), cv.FONT_HERSHEY_SIMPLEX, textSize, (0,255,0), 2, cv.LINE_AA) 

        if blueLine is not None:
            for x1,y1,x2,y2 in blueLine:
                # cv.line(img, (x1,y1), (x2,y2), (255,0,0), 4)
                cv.circle(img, (x1,y1), 4, (0,0,255), 4)
            cv.putText(img, f"{blueAngle * rad2deg:3.0f}", (blueLine[0][0]+textOffset[0],blueLine[0][1]+textOffset[1]), cv.FONT_HERSHEY_SIMPLEX, textSize, (0,0,255), 2, cv.LINE_AA) 
        if redLine is not None:
            for x1,y1,x2,y2 in redLine:
                # cv.line(img, (x1,y1), (x2,y2), (255,0,0), 4)
                cv.circle(img, (x1,y1), 4, (255,0,0), 4)
            cv.putText(img, f"{redAngle * rad2deg:3.0f}", (redLine[0][0]+textOffset[0],redLine[0][1]+textOffset[1]), cv.FONT_HERSHEY_SIMPLEX, textSize, (255,0,0), 2, cv.LINE_AA) 
    if not sendImage:
        img = None
        
    time5 = time.time()
    
    # print(f"{time2-time1},{time3-time2},{time4-time3},{time5-time4}") # Measure time it takes to run each section

    return img, [angle22pi(pinkAngle),angle22pi(greenAngle),angle22pi(blueAngle),angle22pi(redAngle)]
    


def saturateToBounds(value, Bound):
    """
    Limit the value to the bounds at the end value with be between -Bound and Bound or equal to the bounds

    --------------------------------------------------------------
    Input:
        value (float)
        Bound (float)
        
    --------------------------------------------------------------
    Output: 
        value (float)
    
    """
    if value < -Bound:
        return Bound
    elif value > Bound:
        return Bound
    else:
        return value

def triangulate(angle1, angle2, angle3, beacon1, beacon2, beacon3):
    """
    Second main function, takes in the angles and positions of the beacons and calculates the position of the robot as well as the orientation of the robot
    Steps of this function :
        - Sort the angles and beacons by angle
        - Calculate the position of the robot
        - Calculate the orientation of the robot

    --------------------------------------------------------------
    Input:
        angle1 (float) [rad]
        angle2 (float) [rad]
        angle3 (float) [rad]
        beacon1 (float, float) [m]
        beacon2 (float, float) [m]
        beacon3 (float, float) [m]

    --------------------------------------------------------------
    Output: 
        position (float, float) [m]
        orientation (float) [rad]
    
    """

    # check if any of the angles are None
    if angle1 is None or angle2 is None or angle3 is None:
        return None,None

    # sort the angles and beacons by angle
    indices = np.argsort([angle1, angle2, angle3])
    newAngle1 = [angle1, angle2, angle3][indices[0]]
    newAngle2 = [angle1, angle2, angle3][indices[1]]
    newAngle3 = [angle1, angle2, angle3][indices[2]]

    newBeacon1 = [beacon1, beacon2, beacon3][indices[0]]
    newBeacon2 = [beacon1, beacon2, beacon3][indices[1]]
    newBeacon3 = [beacon1, beacon2, beacon3][indices[2]]

    # implement the ToTal algorithm

    COTMAX = 1000000

    Xd1 = newBeacon1[0] - newBeacon2[0]
    Yd1 = newBeacon1[1] - newBeacon2[1]

    Xd3 = newBeacon3[0] - newBeacon2[0]
    Yd3 = newBeacon3[1] - newBeacon2[1]

    T12 = 1 / np.tan(newAngle2 - newAngle1)
    T23 = 1 / np.tan(newAngle3 - newAngle2)
    T31 = (1-T12*T23)/(T12+T23)

    # saturate the values, if they are too big, they are not valid, if they are too small, they are not valid
    T12 = saturateToBounds(T12, COTMAX)
    T23 = saturateToBounds(T23, COTMAX)
    T31 = saturateToBounds(T31, COTMAX)

    X12 = Xd1 + T12*Yd1
    Y12 = Yd1 - T12*Xd1
    X23 = Xd3 - T23*Yd3
    Y23 = Yd3 + T23*Xd3
    X31 = (Xd3 + Xd1) + T31*(Yd3 - Yd1)
    Y31 = (Yd3 + Yd1) - T31*(Xd3 - Xd1)

    k31 = (Xd3 * Xd1) + (Yd3 * Yd1) + T31 * (Xd1 * Yd3 - Xd3 * Yd1)
    
    D = (X12 - X23) * (Y23 - Y31) - (X23 - X31) * (Y12 - Y23)
    
    coef = k31 / D

    if D == 0 :
        print("error")
        return [0,0]

    robotX = newBeacon2[0] + coef * (Y12 - Y23)
    robotY = newBeacon2[1] + coef * (X23 - X12)

    # for each beacon calculate the angle of the robot compared to the horizontal line
    beaconsList = [newBeacon1, newBeacon2, newBeacon3]
    anglesList = [newAngle1, newAngle2, newAngle3]
    orientation = 0
    for i in range(3):
        #using the position calculate the arctan and then add the angle of the beacon, to have the angle of the robot compared to the horizontal line
        current_angle = np.arctan2(robotY - beaconsList[i][1], robotX - beaconsList[i][0]) - anglesList[i]
        current_angle = angle22pi(-current_angle)
        # print(f"current angle {i} : {current_angle}")
        orientation += current_angle
        
    orientation /= 3

    return [robotX,robotY],orientation
    

def calcPosFromAngle(Angles, beaconsPos = [[0,0],[0,1],[1,1],[1,0]]):
    """
    Third main function, takes in the angles and positions of the beacons and calculates the position of the robot as well as the orientation of the robot
    Steps of this function :
        - Make a list of all the possible triplets of beacons
        - For each triplet :
            - Test if the angles are not too close and if the beacons are in order
            - For each triplet calculate the position of the robot with the triangulate function
        - Calculate the average position and orientation of the robot with each valid triplet

    --------------------------------------------------------------
    Input:
        Angles (float, float, float, float) [rad]
        beaconsPos Array((float, float)) [m]

    --------------------------------------------------------------
    Output: 
        position (float, float) [m]
        orientation (float) [rad]
        numberOfValidTriplets (int)
    
    """

    # calculate the position of the robot from the angles, the angles are in radians, the angles are with the corners of the arena
    # the beacons are in order : blue pink red green
    # triangulate the position for each of the triplets of beacons

    maxX = max(beaconsPos, key=lambda x: x[0])[0]
    maxY = max(beaconsPos, key=lambda x: x[1])[1]
    minX = min(beaconsPos, key=lambda x: x[0])[0]
    minY = min(beaconsPos, key=lambda x: x[1])[1]

    triplets = [[0,1,2],[0,1,3],[0,2,3],[1,2,3]]

    positions = []
    orientations = []

    tolTooClose = 0.20

    for triplet in triplets:
        
        if isNone(Angles[triplet[0]],Angles[triplet[1]],Angles[triplet[2]]):
            continue
        
        isTooClose = False

        # check if two angle are too close use a for loop to check all the combinations
        for i in range(3):
            for j in range(i+1,3):
                if abs(Angles[triplet[i]] - Angles[triplet[j]]) < tolTooClose:
                    isTooClose = True
        if isTooClose:
            continue
        
        #check if it's a valid triplet, if the order is respected
        testAngle = isInBetween(Angles[triplet[0]],Angles[triplet[1]],Angles[triplet[2]])
        if not testAngle:
            continue

        position, orientation = triangulate(Angles[triplet[0]],Angles[triplet[1]],Angles[triplet[2]], beaconsPos[triplet[0]], beaconsPos[triplet[1]], beaconsPos[triplet[2]])
        if position is not None and position[0] < maxX and position[0] > minX and position[1] < maxY and position[1] > minY: 
            positions.append(position)
            orientations.append(orientation)
                  
    #calculate the average position
    if len(positions) == 0:
        return None,None,None
    
    averagePos = np.mean(positions, axis=0)
    averageOrientation = np.mean(orientations)

    return averagePos, averageOrientation, len(positions)

def isInBetween(A,B,C):
    """
    Checking the order of the angles, if the angles are in order

    --------------------------------------------------------------
    Input:
        A (float) [rad]
        B (float) [rad]
        C (float) [rad]

    --------------------------------------------------------------
    Output: 
        testValid (bool)
    
    """
    # it's like A <= B and B <= C, but for angles which means there are modulo 
    # B is the angle we want to check is in between A and C

    if A <= C:
        return (B > A or B < C)
    else:
        return (B < A and B > C)


def calculateExampleAngles(beaconsPos):
    """
    Testing function to test the algorithm and make artificial angles

    --------------------------------------------------------------
    Input:
        beaconsPos Array((float, float)) [m]

    --------------------------------------------------------------
    Output: 
        angles (float, float, float, float) [rad]
        pos (float, float) [m]
    
    """

    #calculate the angles for a given position
    angles = []
    
    pos = [0.1,0.4]#[random.random() * 8,random.random() * 8]

    for beaconPos in beaconsPos:
        angle = np.arctan2(beaconPos[1]-pos[1],beaconPos[0]-pos[0])
        angles.append(angle)
    
    randomOrigin = random.random()*2*np.pi

    for i in range(len(angles)):
        angles[i] = [i] + randomOrigin

    return angles, pos

def calcPosFromImage(img, beaconPos,sendImage = False, debug = False, showPlot = False):
    """
    Use the different function (calcAnglesFromImage and calcPosFromAngle) to calculate the position of the robot from an image

    --------------------------------------------------------------
    Input:
        img (np.array)
        beaconPos Array((float, float)) [m]
        sendImage (bool)
        debug (bool)
        showPlot (bool)

    --------------------------------------------------------------
    Output: 
        img (np.array)
        pos (float, float) [m]
        orientation (float) [rad]
        numberOfpoints (int)
        angles (float, float, float, float) [rad]
    
    """

    
    time1 = time.time()
    #use the function to calculate the angles from the image
    img,angles = calcAnglesFromImage( img , debug = debug,sendImage=sendImage, showPlot = showPlot)

    time2 = time.time()
    #use the function to calculate the position from the angles
    pos, orientation, numberOfpoints = calcPosFromAngle(angles, beaconPos)

    time3 = time.time()
    # print(f"time for angle : {time2-time1}, time for position : {time3-time2}")
    
    return img, pos, orientation, numberOfpoints, angles

def calcPosFromCapture(cap, beaconPos, sendImage = False):
    """
    Capture an image from the webcam and calculate the position of the robot from the image

    --------------------------------------------------------------
    Input:
        cap (cv2.VideoCapture)
        beaconPos Array((float, float)) [m]
        sendImage (bool)

    --------------------------------------------------------------
    Output: 
        Same as calcPosFromImage:
        img (np.array)
        pos (float, float) [m]
        orientation (float) [rad]
        numberOfpoints (int)
        angles (float, float, float, float) [rad]
    
    """
    time1 = time.time()
    img = getWebcamImage(cap)
    # print(f"time for capture {time.time() - time1}")
    return calcPosFromImage(img, beaconPos, sendImage = sendImage)

def multipleTest(folder):
    """
    Testing many same images at a time for tuning the algorithm

    --------------------------------------------------------------
    Input:
        folder (string)

    --------------------------------------------------------------
    Output: 
        None
    
    """

    images = os.listdir(folder)
    images.sort()
    print(images)

    for image in images:
        
        img = cv.imread(folder+"/"+image)
        print(f"image : {image}, path : {folder+image}")
        img, angles = calcAnglesFromImage( img , debug = True, showPlot=False, sendImage=True, putPointsOnImage=True)
        pos,orientation, numPoints = calcPosFromAngle(angles)
        plt.subplot(1, len(images), images.index(image)+1)
        # change the color form BGR to RGB
        # img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        plt.imshow(img)  
    
    plt.show(block=False)

    plt.figure()


if __name__ == "__main__":

    capture = initWebcam(camera_id = 1, width = 1920, height = 1080) #,listCameras=True
    
    #     multipleTest("C:/Users/clare/Documents/Work documents/Cours/EPFL/Ma 2/LeRobotEnY/Code/Utils/TestImages")
    # 
    #     # for all images in the folder, calculate the position and show the image with the position
    #     folder = "C:/Users/clare/Documents/Work documents/Cours/EPFL/Ma 2/LeRobotEnY/Code/Utils/TestImages"
    #     images = os.listdir(folder)
    # 
    #     for image in images:
    #         img = cv.imread(folder+"/"+image)
    # 
    # 
    #         pos,orientation = calcPosFromImage(img, [[0,0],[0,8],[8,8],[8,0]])
        # print(f"position : {pos}, orientation : {orientation * rad2deg}, time : {end-start}")
        

        #showImage(img, "image",isBlocking = True)
    # img = getWebcamImage(cap=capture)
    # saveImage(img, "image.png")

    while True:
        
        img = getWebcamImage(cap=capture)
    
        #angles = calcAnglesFromImage( img , debug = True, showPlot = True)
        beaconsPos = [[0,0],[0,8],[8,8],[8,0]]

        # angles, pos = calculateExampleAngles(beaconsPos = beaconsPos)

        # print(f"Real position : {pos}")
        # pos = calcPosFromAngle(angles, beaconsPos)
        # print("showing image")

        pos,orientation,_,_,_ = calcPosFromImage(img, beaconsPos, showPlot = True, debug = True) 
        if pos is not None and orientation is not None:
            print(pos, orientation * 180 / np.pi)
        else:
            print(None)
        
        #showImage(img, "image",isBlocking = True)