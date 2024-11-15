from Mouvements import *
from Navigation import *
from raspberry import *
from beaconDetection import *
from KalmanFilter import *




def main():
    """
    Main function of the program. This is where the magic happens. Here is what happend in this function:
        - Initialize the variables
        - Initialize the camera
        - Initialize the model
        - Initialize the arduino

        - Loop :
            - Calculate the position of the robot with the beacons
            - Find the duplos
            - Use the Kalman filter to smooth the position of the robot
            - Calculate the objectives of the robot
            - Send the objectives to the arduino

    --------------------------------------------------------------
    Input:
        None

    --------------------------------------------------------------
    Output: 
        None
    
    """

    # seems like the 2 camera always use the same 2 ids, but randomly, fast way of chaning it
    if False:
        frontCamId = 2
        topCamId = 0
    else:
        frontCamId = 0
        topCamId = 1

    showResume = True
    
    lastArduinoSend = time.time()
    maxArduinoSendInterval = 0.4 # [s]
    timeSinceLastLoop = time.time()

    pos = None
    orientation = None
    resultListening = None
    angles = [None,None,None,None]
    
    connectToArduino = True
    
    beaconsPos = [[0,0],[0,8],[8,8],[8,0]]

    beaconsData = {
        "beaconsPos": beaconsPos,
        "beaconsColor": [[255,0,255],[0,255,0],[255,0,0],[0,0,255]],
        "beaconsName": ["Pink","green","blue","yellow"],
    }

    # covariance is the 
    covariance = np.identity(3)
    lastKalmanTime = time.time()
    lastDuploPos = (0,0)
    lastNumberOfSucessTriangulations = 0
    
    lastMotorInput = np.array([20,20])

    field = Field(robot=Robot(1,1))
    currentState = "go"
    field.robot.missionNumber = 0
    field.robot.subMissionNumber = 0
    field.robot.timeSinceSearchStarted = time.time()
    field.robot.timeSinceStart = time.time()
    field.robot.currentState = currentState

    frontCam = initWebcam(frontCamId)
    topCam = initWebcam(topCamId, width = 1920, height = 1080, exposure = 100) 
    
    if not isNone(frontCam, topCam):
        print(f"Initialized cameras : {frontCam}, {topCam}")
    
    frontCamImg = None
    topCamImg = None

    vitesse = 0
    deltaV = 0
    vGauche = 0
    vDroite = 0
    
    readInstructionsExample = {
        "flipVertical": False,
        "flipHorizontal": False,
        "convertColor": False,
    }

    print("Initializing Model")
    model = initModel("BigBossTraining.tflite")
    
    if connectToArduino:
        print("Initializing Serial")
        arduinoSerial = initArduinoSerial()
    else:
        arduinoSerial = None

    if arduinoSerial is not None:
        print(f"Arduino opened with serial : {arduinoSerial}")
    
    while True:
        time1 = time.time()
        
        #print("Calculating beacons")
        if topCam is not None: 
            # print("Calculating Position")
            topCamImg,pos,orientation,numberOfSucessTriangulations,angles = calcPosFromCapture(topCam,beaconsPos,sendImage=showResume)
            # field.robot.position = pos
            # field.robot.orientation = orientation
            # if not (pos is None or pos[0] == None or pos[1] == None or orientation == None):
            #     print(f"position {pos[0]},{pos[1]}, orientation {orientation}")
            if not isNone(pos, orientation):
                field.robot.lastNavX = pos[0]
                field.robot.lastNavY = pos[1]
                field.robot.lastNavOrientation = orientation
                lastNumberOfSucessTriangulations = numberOfSucessTriangulations

        time2 = time.time()
        
        if frontCam is not None:
            # print("Calculating vision")
            frontCamImg,box,resolution = calcVision(model,frontCam, sendImage=showResume) 
        else:
            frontCamImg,box,resolution = None,None,None

        time3 = time.time()

        # print("Calculating Kalman")
        dt = time.time() - lastKalmanTime
        lastKalmanTime = time.time()
        state_prev = np.array([field.robot.position[0],field.robot.position[1],field.robot.orientation])

        if isNone(pos, orientation):
            camera_measure = np.array([None,None,None])
        else:
            camera_measure = np.array([pos[0],pos[1],orientation])

        # calculate the Kalman filter variables
        motor_input = lastMotorInput
        flag_pos = topCam is None or pos is None or pos[0] == None or pos[1] == None or orientation == None
        newState,covariance = calcKalmanFilter(state_prev, covariance, camera_measure, motor_input, flag_pos, dt)
        field.robot.position = [newState[0],newState[1]]
        field.robot.orientation = newState[2]
        
        time4 = time.time()

        # print("Calculating objective")   
        duploObjective = None

        if box is not None: 
            bbox = box.bounding_box 
            # print(bbox)
            duploObjective = {
                "X": bbox.origin_x + bbox.width / 2,
                "Y": bbox.origin_y + bbox.height / 2,
                "W": resolution[0], 
                "H": resolution[1], 
                "name" : "duplo",
            }
            
            lastDuploPos = (bbox.origin_x + bbox.width / 2,bbox.origin_y + bbox.height / 2)
            
            duploObjective = translateXYToAngleDistance(duploObjective)

        # print("Calculating global objective")
        globalObjective = calcGlobalObjective(field,duploObjective,angles,frontCam)

        # print("Making movement")
        commands = makeMovement(objective = globalObjective)
            
        time5 = time.time()
        
        deltaV = commands["deltaV"]
        vitesse = commands["vitesse"]
        
        # send commands to the robot
        if arduinoSerial is not None:
            if time.time() - lastArduinoSend > maxArduinoSendInterval and connectToArduino:
                        
                resultMovement,(vGauche,vDroite) = sendMovement(arduinoSerial,commands,debug = False)
                
                if (resultMovement) :
                    print("movement failed " + str(resultMovement)) 
                else: # successfully sent
                    lastMotorInput = np.array([vGauche,vDroite])
                    lastArduinoSend = time.time()
            
        time6 = time.time()
   
        stringTime = f"Vision {(time2 - time1):6.3f}s, Position {(time3 - time2):6.3f}s, Kalman {1000*(time4 - time3):6.3f}ms, Objective {1000*(time5 - time4):6.3f}ms, Movement {1000*(time6 - time5):6.3f}ms, total {time.time() - timeSinceLastLoop:6.3f}s"
        timeSinceLastLoop = time.time()

        infoString = commands["infoString"]

        # information to be displayed on the console (on the image and python console)
        stringConsole = (
            f"-----------------------------------------------------------------------------------------------------------\n"
            f"|Position: {field.robot.position[0]:6.3f},{field.robot.position[1]:6.3f}, "
            f"|Orientation: {angle22pi(field.robot.orientation) * rad2deg:3.0f}, "
            f"|State: {field.robot.currentState}, Mission: {field.robot.missionNumber}, Waypoint: {field.robot.subMissionNumber}\n"
            f"|Aduino Connected : {bool2str(not isNone(arduinoSerial))}, TopCam Connected : {bool2str(not isNone(topCam))}, FrontCam Connected : {bool2str(not isNone(frontCam))}, "
            f"|Time since start: {time.time() - field.robot.timeSinceStart:3.0f}s,\n"
            f"|Informations: {infoString},\n"
            f"|Duplo: {bool2str(not isNone(box))}, "
            f"|Last Duplo pos: {lastDuploPos[0]},{lastDuploPos[1]}, "
            f"|Localisation: {lastNumberOfSucessTriangulations:1}, "
            f"|Last Robot pos: {field.robot.lastNavX:6.3f},{field.robot.lastNavY:6.3f}, "
            f"|Orientation : {angle22pi(field.robot.lastNavOrientation) * rad2deg:3.0f},\n"
            f"|Commands dV: {deltaV:+6.3f}, Vf: {vitesse:6.3f}, "
            f"|Last movement sent vG: {lastMotorInput[1]}, vD: {lastMotorInput[0]},\n"
            f"|{stringTime}\n"
            f"-----------------------------------------------------------------------------------------------------------\n"
        )
        
        stringConsole = "".join([stringConsole])
        print(stringConsole)
        stringConsole = stringConsole.split("\n")

        if showResume:
            makeResume(topCamImg=topCamImg,frontCamImg=frontCamImg,beaconsData=beaconsData,commands=commands,field=field, stringConsole=stringConsole)
            
        if cv.waitKey(1) & 0xFF == ord('q'):
            # destroy all windows
            cv.destroyAllWindows()
            break
        
if __name__ == '__main__':
    main()
  