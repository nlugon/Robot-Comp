import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import os
import time
import sys

rad2deg = 180 / np.pi

def angle22pi(angle):
    """
    Convert an angle to the range [0, 2pi]

    --------------------------------------------------------------
    Input:
        angle (float) [rad]

    --------------------------------------------------------------
    Output: 
        angle (float) [rad] : angle in the range [0, 2pi]
    
    """
    if angle is None:
        return None
    while angle > 2 * np.pi:
        angle =  angle - 2 * np.pi
    while angle < 0:
        angle = angle + 2 * np.pi
    
    return angle

def angle2pipi(angle):
    """
    Convert an angle to the range [-pi, pi]

    --------------------------------------------------------------
    Input:
        angle (float) [rad]

    --------------------------------------------------------------
    Output: 
        angle (float) [rad] : angle in the range [-pi, pi]
    
    """

    if angle is None:
        return None
    while angle > np.pi:
        angle =  angle - 2 * np.pi
    while angle < -np.pi:
        angle = angle + 2 * np.pi
    
    return angle


def getWebcamImage(cap = None, readInstructions = None, numberOfFrames = 1):
    """
    Get an image from the webcam, there is number of frames to skip, if there is a buffer

    --------------------------------------------------------------
    Input:
        cap (cv.VideoCapture) : the webcam
        readInstructions (dict) : instructions to read the image
        numberOfFrames (int) : number of frames to skip

    --------------------------------------------------------------
    Output: 
        frame (np.array) : the image from the webcam
    
    """

    if cap == None:
        cap = initWebcam()
        print("Initialisation of webcam recommended")
    
    success, frame = cap.read() 
    
    for i in range(numberOfFrames - 1):
        success, frame = cap.read() 

    if not success:
        print("Cannot read frame from webcam")

    if readInstructions is not None:
        if readInstructions["flipVertical"]:
            frame = cv.flip(frame, 0)
        if readInstructions["flipHorizontal"]:
            frame = cv.flip(frame, 1)
        if readInstructions["convertColor"]:
            frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

    return frame
    
def showImage(img, title = "Image", isBlocking = False, waitTime = 0.0001, waitKey = 0):
    """
    Show an image with openCV

    --------------------------------------------------------------
    Input:
        img (np.ndarray) : the image to show
        title (str) : the title of the window
        isBlocking (bool) : if the window is blocking
        waitTime (float) : the time to wait before closing the window
        waitKey (int) : the time to wait before closing the window

    --------------------------------------------------------------
    Output: 
        None
    
    """

    cv.imshow(title, img)

    # cv.waitKey(waitKey)

    if isBlocking:
        cv.waitKey(waitKey)

    # #clear the plot 
    # plt.clf()
    # plt.cla()

    # plt.imshow(img)
    # plt.title(title)

    # plt.show(block = isBlocking)
    # # the plot doesn't show
    # plt.pause(waitTime)
        
    
    
def saveImage(img, path = "image.png"):
    """
    Save an image with openCV

    --------------------------------------------------------------
    Input:
        img (np.ndarray) : the image to save
        path (str) : the path to save the image

    --------------------------------------------------------------
    Output: 
        None
    
    """

    cv.imwrite(path, img)
    
def initWebcam(camera_id: int = -1, width: int = -1, height: int = -1, listCameras: bool = False, exposure = False):
    """
    Initialise the webcam with camera_id, width and height, you can list the first 10 cameras available, with the argument listCameras, you can change the exposure

    --------------------------------------------------------------
    Input:
        camera_id (int) : the id of the camera
        width (int) : the width of the image
        height (int) : the height of the image
        listCameras (bool) : if you want to list the first 10 cameras available
        exposure (bool) : if you want to change the exposure

    --------------------------------------------------------------
    Output: 
        cam (cv.VideoCapture) : the webcam
    
    """

    if listCameras:
        # print all available cameras
        openIds = []
        for i in range(10):
            try:
                cap = cv.VideoCapture(i)
            except:
                pass
            
            if cap.isOpened():
                openIds.append(i)
                print("Camera", i, "is available, nameL:", cap.getBackendName() ,", resolution:", cap.get(3), "x", cap.get(4))
            cap.release()

        # show all the opened cameras in a single window
        plt.figure("Cameras")
        for i in openIds:
            cap = cv.VideoCapture(i)
            _, img = cap.read()
            
            plt.subplot(2, 5, i+1)
            plt.title("Camera " + str(i))
            plt.imshow(img)
            plt.axis("off")

            cap.release()

        plt.show()

        # prompt user to select camera
        if camera_id == -1:
            camID = int(input("Select camera: "))
        else:
            camID = camera_id

        cam = cv.VideoCapture(camID)
    else:
        cam = cv.VideoCapture(camera_id)

    count = 0
    maxIter = 100

    while not cam.isOpened() and count < maxIter:
        count += 1
        pass
    
    if count >= maxIter:
        print("Cannot open camera")
        return None
    
    if width != -1:
        cam.set(cv.CAP_PROP_FRAME_WIDTH, width)
    
    if height != -1:
        cam.set(cv.CAP_PROP_FRAME_HEIGHT, height)

    # set the frameBuffer to 1 to prevent delays
    cam.set(cv.CAP_PROP_BUFFERSIZE, 1)
    
    # set the exposure to very dark
    if exposure != False:
        cam.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)
        cam.set(cv.CAP_PROP_EXPOSURE, exposure)

    #show the image 
    result, img = cam.read()
        
    time.sleep(0.1)
    
    if not cam.isOpened():
        print("Cannot open camera")
        exit()

    return cam

def saveImagesInARow(cam, path = "/home/leroboteny/Desktop/LeRobotEnY/Code/Utils/AutoSavedImages"):
    """
    Save images to a folder in a row, it is used for sampling data, like for a dataset, or tuning the beacons detection

    --------------------------------------------------------------
    Input:
        cam (cv.VideoCapture) : the webcam
        path (str) : the path to save the images

    --------------------------------------------------------------
    Output: 
        None
    
    """

    # check if the folder exists
    if not os.path.exists(path):
        os.makedirs(path)

    # read the folder and determine the next number to save
    files = os.listdir(path)
    if len(files) == 0:
        nextNumber = 0
    else:
        nextNumber = int(files[-1].split(".")[0]) + 1
    
    while True:
        img = getWebcamImage(cam)
        img = cv.flip(img, 0)
        saveImage(img, path + f"/{nextNumber}.png")
        img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        print(f"saved {path}/{nextNumber}.png")
        nextNumber += 1
        plt.imshow(img)
        plt.title("Image")
        # fait for next keypress for the next loop
        plt.waitforbuttonpress()
        plt.close()

def bool2str(boolean):
    """
    Convert a boolean to a nice string

    --------------------------------------------------------------
    Input:
        boolean (bool) : the boolean to convert

    --------------------------------------------------------------
    Output: 
        str (str) : the string
    
    """
    # put a checkmark if the bool is true, a cross if it is false
    if boolean:
        return "Y"
    else:
        return "N"

def draw_triangle(image, center, theta, size, color = (0, 0, 255), thickness = 1):
    """
    Draw a triangle on an image representing the position of the robot by the Kalman filter and the beacons detection

    --------------------------------------------------------------
    Input:
        image (np.ndarray) : the image to draw on
        center (tuple) : the center of the triangle
        theta (float) : the angle of the triangle
        size (tuple) : the size of the triangle
        color (tuple) : the color of the triangle
        thickness (int) : the thickness of the triangle

    --------------------------------------------------------------
    Output: 
        None
    
    """

    # Calculate the coordinates of the triangle vertices
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    x, y = center

    half_size_x = size[0] / 2
    half_size_y = size[1] / 2

    vertex1 = (int(x - cos_theta * half_size_x + sin_theta * half_size_y),
               int(y - sin_theta * half_size_x - cos_theta * half_size_y))
    vertex2 = (int(x - cos_theta * half_size_x - sin_theta * half_size_y),
               int(y - sin_theta * half_size_x + cos_theta * half_size_y))
    vertex3 = (int(x),
               int(y))

    # Draw the triangle on the image, it needs to be filled
    cv.drawContours(image, [np.array([vertex1, vertex2, vertex3])], 0, color, thickness = thickness, lineType = cv.LINE_AA)

def makeResume(topCamImg, frontCamImg, beaconsData, commands, field, stringConsole):
    """
    Make the resume of the current loop, it display the result of the detection, the results of the beacons detection,
    the map with the different position of the robot, sent by the beacons and the kalman filter,
    It also display the string console, which is the string that has all the information about the current loop

    --------------------------------------------------------------
    Input:
        topCamImg (np.ndarray) : the image from the top camera
        frontCamImg (np.ndarray) : the image from the front camera
        beaconsData (dict) : the data of the beacons
        commands (dict) : the commands of the robot
        field (np.ndarray) : the field
        stringConsole (str) : the string console

    --------------------------------------------------------------
    Output: 
        None
    
    """

    # if one of the images is None, return the other one
    if topCamImg is None and frontCamImg is None:
        return
    elif topCamImg is None:
        frontCamImg = cv.cvtColor(frontCamImg, cv.COLOR_BGR2RGB)
        showImage(frontCamImg, "Front Cam")
        return
    elif frontCamImg is None:
        topCamImg = cv.cvtColor(topCamImg, cv.COLOR_BGR2RGB)
        showImage(topCamImg, "Top Cam")
        return

    # tansform the images to rgb
    topCamImg = cv.cvtColor(topCamImg, cv.COLOR_BGR2RGB)
    frontCamImg = cv.cvtColor(frontCamImg, cv.COLOR_BGR2RGB)

    #the top cam has a higher resolution so we need to resize the front cam image
    # frontCamImg = cv.resize(frontCamImg, (frontCamImg.shape[1], topCamImg.shape[0]))
    if frontCamImg.shape[0] > topCamImg.shape[0]:
        frontCamImg = cv.resize(frontCamImg, (frontCamImg.shape[1], topCamImg.shape[0]))
    else:
        topCamImg = cv.resize(topCamImg, (topCamImg.shape[1], frontCamImg.shape[0]))

    #create the resume image put them side by side by concatenating them
    resumeImg = np.concatenate((topCamImg, frontCamImg), axis=1)

    #make a map on the image
    mapVerticalSize = 300

    mapImg = np.zeros((mapVerticalSize, mapVerticalSize, 3), dtype=np.uint8)
    consoleImg = np.zeros((mapVerticalSize, resumeImg.shape[1] - mapVerticalSize, 3), dtype=np.uint8)

    margin = 10

    #make a map of the field
    cv.rectangle(mapImg, (margin, margin), (mapVerticalSize - margin, mapVerticalSize - margin), (255, 255, 255), 2)
    beaconsPos = beaconsData["beaconsPos"]
    beaconsColor = beaconsData["beaconsColor"]

    maxX = max(beaconsPos, key=lambda x: x[0])[0]
    maxY = min(beaconsPos, key=lambda x: x[1])[1]
    minX = min(beaconsPos, key=lambda x: x[0])[0]
    minY = max(beaconsPos, key=lambda x: x[1])[1]

    for i in range(len(beaconsPos)):
        # print(beaconsPos[i])
        x = map(beaconsPos[i][0], minX, maxX, margin, mapVerticalSize - margin)
        y = map(beaconsPos[i][1], minY, maxY, margin, mapVerticalSize - margin)
        # print(x, y)
        cv.circle(mapImg, (int(x), int(y)), 5, beaconsColor[i], -1)

    robot = field.robot

    kalmanX = robot.x
    kalmanY = robot.y
    kalmanX = map(kalmanX, minX, maxX, margin, mapVerticalSize - margin)
    kalmanY = map(kalmanY, minY, maxY, margin, mapVerticalSize - margin)
    kalmanOrientation = robot.orientation

    navX = robot.lastNavX
    navY = robot.lastNavY
    navX = map(navX, minX, maxX, margin, mapVerticalSize - margin)
    navY = map(navY, minY, maxY, margin, mapVerticalSize - margin)
    navOrientation = robot.lastNavOrientation

    # put triangles on the map
    kalmanColor = (0, 255, 0)
    navColor = (0, 0, 255)
    triangleSizeX = 20
    triangleSizeY = 5

    # kalman triangle
    draw_triangle(mapImg, (kalmanX, kalmanY), kalmanOrientation, (triangleSizeX, triangleSizeY), kalmanColor, 2)
    # nav triangle
    draw_triangle(mapImg, (navX, navY), navOrientation, (triangleSizeX, triangleSizeY), navColor, 2)

    # put the text in the console
    # consoleImg = cv.putText(consoleImg, stringConsole, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv.LINE_AA)
    offset = 0
    for i in stringConsole:
        consoleImg = cv.putText(consoleImg, i, (10, 30 + offset), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv.LINE_AA)
        offset += 30

    if commands is not None and commands["name"]=="position":
        if commands["X"] is not None and commands["Y"] is not None:
            x = commands["X"]
            y = commands["Y"]
            x = map(x, minX, maxX, margin, mapVerticalSize - margin)
            y = map(y, minY, maxY, margin, mapVerticalSize - margin)
            cv.circle(mapImg, (int(x), int(y)), 5, (255, 0, 0), -1)

    #concatenate the map and the console
    mapConsoleImg = np.concatenate((mapImg, consoleImg), axis=1)
    # print(f"resumeImg shape: {resumeImg.shape}, mapImg shape: {mapImg.shape}, consoleImg shape: {consoleImg.shape}, mapConsoleShape: {mapConsoleImg.shape}")
    # Concatenate the resume image and the console
    resumeImg = np.concatenate((resumeImg, mapConsoleImg), axis=0)

    # show the image
    showImage(resumeImg, "Resume", isBlocking=False)
    

def translateXYToAngleDistance(objective):
    """
    Translates the X and Y coordinates of the objective to an angle and a distance

    --------------------------------------------------------------
    Input:
        objective (dict)

    --------------------------------------------------------------
    Output: 
        objective (dict) : added angle and distance to the objective
    
    """
    X = objective["X"]
    Y = objective["Y"]

    W = objective["W"]
    H = objective["H"]
    

    angle = - (X - W/2) / W * 4 * (Y / H) #np.pi / 3
    
    distance = map(Y,0,H,0.1,8) # should be an arctan

    objective["angle"] = angle
    objective["distance"] = distance

    return objective

def map(x, in_min, in_max, out_min, out_max):
    """
    Maps a value from one range to another

    --------------------------------------------------------------
    Input:
        x (float) : value to map
        in_min (float) : minimum value of the input range
        in_max (float) : maximum value of the input range
        out_min (float) : minimum value of the output range
        out_max (float) : maximum value of the output range

    --------------------------------------------------------------
    Output: 
        x (float) : mapped value     
    
    """

    # maps a value from one range to another
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def isNone(*args):
    """
    Check if any of the arguments is None, it's purely used to make the code more readable and have a reliable code
    It is recursive, it checks every element of an array, and it's elements, and so on

    --------------------------------------------------------------
    Input:
        *args (list) : list of arguments to check

    --------------------------------------------------------------
    Output: 
        isNone (bool) : True if any of the arguments is None, False otherwise
    
    """

    for arg in args:
        if arg is None:
            return True
        # if it's an array, check if any of the elements is None
        elif type(arg) == np.ndarray or type(arg) == list:
            for element in arg:
                if element is None:
                    return True
                
    return False

if __name__ == "__main__":

    # 1 = top
    # 0 = front

    capture = initWebcam(camera_id = 1, width = 1920, height = 1080, exposure = 100,listCameras=True) #,listCameras=True
    
    # saveImagesInARow(capture) 
    # img = getWebcamImage(cap=capture)
    # saveImage(img)

    while True:
        
        start = time.time()
        img = getWebcamImage(cap=capture)

        middle = time.time()
        # showImage(img, "image",isBlocking = False)
        # Display the resulting frame
        cv.imshow('frame', img)
        
        end = time.time()
        # print(f"Time to get image: {middle - start}, time to show image: {end - middle}")
        # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            # destroy all windows
            cv.destroyAllWindows()
            break
        
        