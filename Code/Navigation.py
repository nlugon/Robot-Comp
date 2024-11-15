import numpy as np

from classes import *
from utils import *


def calcGlobalObjective(field,duploObjective,angles,frontCapture):
    """
    Calculates the global objective of the robot, this is using a finite state machine
    Here are the different states possible:
    - alone : robot does everything by itself, only with local navigation
    - aloneAfterStuck : robot is stuck, it will try to get out of the situation only with local navigation
    - home : robot goes back to home, doesn't follow duplos
    - unloading : robot is unloading the duplo
    - aligning : robot is aligning to the duplo
    - aligningButton : robot is aligning to the button
    - pushingButton : robot is pushing the button
    - go : robot is going to the next waypoint, following the duplos if it sees one
    - search : robot is searching for a duplo, goes towards it if it sees one
    - button : robot is going to the button

    --------------------------------------------------------------
    Input:
        field (Field), contains the field with the robot inside
        duploObjective (dict), contains the objective of the duplo
        angles (float, float, float, float) [rad], contains the angles of the duplos
        frontCapture (cv2.VideoCapture), contains the video capture of the front camera

    --------------------------------------------------------------
    Output: 
        objective (dict), contains the objective of the robot
    
    """

    robot = field.robot
    currentState = robot.currentState

    home = (0.25,0.25)
    buttonPos = (4.05,6.5)
    missions = [    
                    [(-1,-1),(-2,-2),(2.99,7.5)], # pushin button mission        
                    [(7,1),(7,4)], # carpet mission
                    [(3,2), (5,4), (6,2)], # zone 1 mission (just in case)
               ]


    # + = tourner a gauche - - tourner a droite

    # different threshold for different states
    thresholdDistanceFar = 1
    thresholdDistanceClose = 0.20
    thresholdDistanceUnload = 0.5
    thresholdTimeIsStuck = 10
    thresholdRadiusIsStuck = 0.25

    # different times for different states
    searchTime = 20
    unloadTime = 6
    aloneTime = 60
    buttonAloneTime = 15
    buttonPushingTime = 3
    totalTime = 600
    combackTime = 70

    objective = aloneObjective()
    done = False

    infoString = ""
    
    if robot.missionNumber > len(missions) - 1:
        currentState = "alone"
        robot.missionNumber = 0
        robot.subMissionNumber = 0

    if currentState not in ["alone","aloneAfterStuck", "home", "unloading","aligning"] and totalTime - (time.time() - robot.timeSinceStart) < combackTime:
        currentState = "home"

    robot.updatePositions(thresholdTimeIsStuck,thresholdRadiusIsStuck)
    isRobotStuck = robot.isStuck(thresholdTimeIsStuck,thresholdRadiusIsStuck)

    if isRobotStuck and robot.currentState not in ["aloneAfterStuck","unloading","aligning","aligningButton","pushingButton"]:
        currentState = "aloneAfterStuck"
        robot.stateToReturnTo = robot.currentState

    while done == False:
        done = True
        
        if currentState == "unloading":
            if time.time() - robot.timeSinceUnloadStarted > unloadTime:
                currentState = "go"
                done = False
            else:
                _,objective,_ = alignForUnloading(angles)
                objective["name"] = "unloading" 
            infoString = f"Unloading remaining : {unloadTime - (time.time() - robot.timeSinceUnloadStarted):2.1f}s"

        elif currentState == "aligning":
            isAligned, objective,WMAngle = alignForUnloading(angles)
            if isAligned:
                currentState = "unloading"
                robot.timeSinceUnloadStarted = time.time()
            
            infoString = f"Unloading angle : {WMAngle * rad2deg:3.0f}s"

        elif currentState == "home":
            objective = positionToObjective(field.robot,home)
            if isCloseEnough(field.robot, home, thresholdDistanceUnload):
                currentState = "aligning"
                done = False
            
        elif currentState == "go":

            currentMission = robot.missionNumber
            currentSubMission = robot.subMissionNumber
            
            currentObjective = missions[currentMission][currentSubMission]
            thresholdDistance = thresholdDistanceFar
            
            if currentObjective == buttonPos:
                thresholdDistance = thresholdDistanceClose

            if currentObjective == (-1,-1):
                #push button
                currentState = "aligningButton"
                done = False
            
            elif currentObjective == (-2,-2):
                #push button
                currentState = "aloneAfterButton"
                robot.timeSinceAlone = time.time()
                done = False

            elif isCloseEnough(field.robot, currentObjective, thresholdDistance):
                currentState = "search"
                robot.timeSinceSearchStarted = time.time()
                done = False

            elif duploObjective is not None:
                objective = duploObjective

            else:
                objective = positionToObjective(field.robot,currentObjective)

        elif currentState == "search":

            currentMission = robot.missionNumber
            currentSubMission = robot.subMissionNumber

            currentObjective = missions[currentMission][currentSubMission]

            if time.time() - robot.timeSinceSearchStarted > searchTime:
                if currentSubMission < len(missions[currentMission]) - 1:
                    robot.subMissionNumber += 1
                    currentState = "go"
                    done = False
                else:
                    robot.missionNumber += 1
                    robot.subMissionNumber = 0
                    currentState = "home"
                    done = False

            elif duploObjective is not None:
                objective = duploObjective

            else:
                objective = searchingObjective(field.robot)

            infoString = f"Remaining searching time : {searchTime - (time.time() - robot.timeSinceSearchStarted):2.1f}s"
                
        elif currentState == "aloneAfterButton":

            if time.time() - robot.timeSinceAlone > buttonAloneTime:
                robot.subMissionNumber += 1
                currentState = "go"
                done = False
            elif duploObjective is not None:
                objective = duploObjective
            else:
                objective = aloneObjective()

            infoString = f"Remaining alone time : {buttonAloneTime - (time.time() - robot.timeSinceAlone):2.1f}s"

        elif currentState == "aligningButton":
            
            if not isCloseEnough(field.robot, buttonPos, thresholdDistanceClose, numberOfFrames = 3) and field.robot.isNextToButton != True:
                objective = positionToObjective(field.robot,buttonPos)
            else:
                
                field.robot.isNextToButton = True
                isAligned, objective, WMAngle = aligningForButtonPushing(angles,frontCapture)

                if isAligned:
                    currentState = "pushingButton"
                    robot.timeSincePushingButton = time.time()
            
                infoString = f"Reached waypoint : {bool2str(field.robot.isNextToButton)}, error angle : {WMAngle:3.0f}s"

        elif currentState == "pushingButton":
            objective = pushingButtonObjective()
            
            if time.time() - robot.timeSincePushingButton > buttonPushingTime:
                currentState = "go"
                robot.subMissionNumber += 1
                robot.hasPushedButton = True
                done = False

            infoString = f"Remaining pushing time : {buttonPushingTime - (time.time() - robot.timeSincePushingButton):2.1f}s"
                
        elif currentState == "start":
            currentState = "alone"
            
            robot.timeSinceAlone = time.time()
            done = False
            
        elif currentState == "alone":
            
            if time.time() - robot.timeSinceAlone > aloneTime:
                currentState = "home"
                done = False
            else:
                objective = searchingObjective(robot)#aloneObjective()
            
            infoString = f"Je me baladais sur l avenue, le coeur ouvert a l imprevu, remaing : {aloneTime - (time.time() - robot.timeSinceAlone):2.1f}s"

        elif currentState == "aloneAfterStuck":
            if isRobotStuck == False:
                currentState = robot.stateToReturnTo
                done = False
            
            infoString = f"La j suis pas bien"
        
        else:
            print(f"Error: unknown state : {currentState}")
            objective = aloneObjective()
            infoString = f"Error state : {currentState}"

    robot.currentState = currentState
    objective["duploObjective"] = duploObjective  
    
    if objective["name"] != "position":
        objective["infoString"] = infoString  


    return objective

def isCloseEnough(robot:Robot,objective,threshold, numberOfFrames = 1):

    if numberOfFrames == 1:
        position = robot.position
    
        distSqr = (objective[1] - position[1])**2 + (objective[0] - position[0])**2
    #     print(distSqr)
        return distSqr <= threshold * threshold
    else:
        #here is how I do list of position self.coordsList.append((self.position[0],self.position[1], self.orientation, time.time()))
        #take only the last number and check for the last numberOfFrames distance, it need to be every time under the threshold
        
        listOfPosition = robot.coordsList[-numberOfFrames:]
        for position in listOfPosition:
            distSqr = (objective[1] - position[1])**2 + (objective[0] - position[0])**2
            if distSqr > threshold * threshold:
                return False
        return True

def searchingObjective(robot):
    objective = {
        "angle": 0, #np.pi/4
        "distance": 6, #0.7
        "name": "searching"
    }

    return objective

def aloneObjective():
    objective = {
        "angle": 0,
        "distance": 100,
        "name": "alone"
    }

    return objective

def pushingButtonObjective():
    objective = {
        "angle": 0,
        "distance": 10,
        "name": "pushingButton",
    }

    return objective

def aligningForButtonPushing(angles,frontCapture):
    """
    Align the robot with the button using the beacons

    --------------------------------------------------------------
    Input:
        Angles (float, float, float, float) [rad]: angles of the beacons

    --------------------------------------------------------------
    Output: 
        success (bool): True if the robot is aligned with the button
        objective (dict): objective to give to the robot
        weightedMeanAngle (float) [rad]: weighted mean angle of the beacons
    
    """
    #  success, objective = detectQR(img) # using QR code to align with button, works not that well
    
    thresholdAngleButton = np.pi/48
    angleObjectives = [-0.52,-1.91,1.91,0.52] #1.65806

    weightedMeanAngle, angleDiff = alignWithBeacons(angles, angleObjectives, [0,3,3,0])
    
    objective = {
        "angle": - weightedMeanAngle,
        "distance": 0,
        "name": "aligningButton"
    }
    
    if abs(weightedMeanAngle) <  thresholdAngleButton:
        return True, objective, weightedMeanAngle
    else:
        return False, objective, weightedMeanAngle

def positionToObjective(robot,objectivePos):
    """
    With an objective and the robot object, calculate the path to go to another objective, and return the objective to give to the robot
    If the robot is in the button area, there is a certain path to get out of the area

    --------------------------------------------------------------
    Input:
        robot (Robot): robot object
        objectivePos (float, float) [m]: position of the objective

    --------------------------------------------------------------
    Output: 
        objective (dict): objective to give to the robot
    
    """

    orientation = robot.orientation
    robotPos = robot.position

    boxButton = [(0,5),(3,8)]
    boxSas = [(2,7),(3.5,8)]
    objectivePosSas = (4.5,7.5)
    objectivePosButton = (2.75,7.5)

    if not isInBox(objectivePos, boxButton):
        if isInBox(robotPos, boxSas):
            objectivePos = objectivePosSas
        elif isInBox(robotPos, boxButton):
            objectivePos = objectivePosButton

    angleToObject = np.arctan2(objectivePos[1] - robotPos[1], objectivePos[0] - robotPos[0])
    angle = (angleToObject + orientation)
    distance = np.sqrt((objectivePos[1] - robotPos[1])**2 + (objectivePos[0] - robotPos[0])**2)
    infoString = f"distance: {distance:3.2f}" 

    objective = {
        "angle": angle2pipi(angle),
        "distance": distance,
        "X": objectivePos[0],
        "Y": objectivePos[1],
        "name": "position",
        "infoString": infoString,
    }

    return objective

def alignWithBeacons(angles, angleObjectives, weights):
    """
    This is a function used to align for the button pushing, it uses the beacons to align the robot and for the unloading
    There are objective angles that it tries to aim

    --------------------------------------------------------------
    Input:
        Angles (float, float, float, float) [rad]: angles of the beacons
        angleObjectives (float, float, float, float) [rad]: angles of the beacons to aim
        weights (float, float, float, float): weights of the beacons

    --------------------------------------------------------------
    Output: 
        weightedMeanAngle (float) [rad]: weighted mean angle of the beacons
        angleDiffs (float, float, float, float) [rad]: difference between the angles and the objective angles
    
    """

    angleDiffs = [0]*4
    
    for i in range(len(angles)):
        if angles[i] is None:
            angles[i] = angleObjectives[i]
        angleDiffs[i] = angle2pipi(angleObjectives[i] - angles[i])

    weightedMeanAngle = np.average(angleDiffs, weights=weights)

    return weightedMeanAngle, angleDiffs

def alignForUnloading(angles):
    """
    Align the robot to the beacons for the unloading

    --------------------------------------------------------------
    Input:
        angles (float, float, float, float) [rad]: angles of the beacons

    --------------------------------------------------------------
    Output: 
        success (bool): True if the robot is aligned with the beacons
        objective (dict): objective to give to the robot
        weightedMeanAngle (float) [rad]: weighted mean angle of the beacons
    
    """

    if angles is None:
        objective = {
            "angle": 0,
            "distance": 0,
            "name": "aligning"
        }
        return False, objective, 0

    thresholdAngle = np.pi/12

    angleObjectives = [0,np.pi/6,0,-np.pi/6]
    
    weightedMeanAngle, angleDiffs = alignWithBeacons(angles, angleObjectives, [3,0,0,0])

    objective = {
        "angle": - weightedMeanAngle,
        "distance": 0,
        "name": "aligning"
    }
    
    closeEnough = abs(weightedMeanAngle) < thresholdAngle and np.sum(angleDiffs) != 0
    
    if closeEnough:
        objective = {
            "angle": - weightedMeanAngle,
            "distance": 0,
            "name": "unloading"
        }
    
    return closeEnough, objective, weightedMeanAngle

def isInBox(position, box):
    """
    Test if a position is in a box

    --------------------------------------------------------------
    Input:
        position (float, float) [m]: position of the robot

    --------------------------------------------------------------
    Output: 
        isInBox (bool): True if the robot is in the box
    
    """

    return position[0] > box[0][0] and position[0] < box[1][0] and position[1] > box[0][1] and position[1] < box[1][1]
