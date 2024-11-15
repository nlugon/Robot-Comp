import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import time

class Robot:
    def __init__(self,x = 0,y = 0):
        """
        Initialize the robot object, with its position and orientation

        --------------------------------------------------------------
        Input:
            x (float), x position of the robot [m]
            y (float), y position of the robot [m]

        --------------------------------------------------------------
        Output: 
            None
        
        """
        self.x = x
        self.y = y
        self.orientation = 0
        self.speed = 0
        self.angularSpeed = 0
        
        self.lastNavX = 0
        self.lastNavY = 0
        self.lastNavOrientation = 0
        
        self.timeSinceUnloadStarted = 0
        self.timeSinceSearchStarted = 0
        self.timeSincePushingButton = 0
        self.timeSinceAlone = 0
        self.timeSinceStart = 0

        self.stateToReturnTo = "home"

        self.missionNumber = 0
        self.subMissionNumber = 0
        self.isNextToButton = False
        self.hasPushedButton = False

        self.currentState = "0"

        self.coordsList = []

    @property
    def position(self):
        return (self.x,self.y)
    
    @position.setter
    def position(self,position):
        if position is None or position[0] is None or position[1] is None:
            return
        
        self.x = position[0]
        self.y = position[1]

    @property
    def kalmanCoords(self):
        """
        Returns the coordinates of the robot in the form of a column vector, it is used for the kalman filter

        --------------------------------------------------------------
        Input:
            None

        --------------------------------------------------------------
        Output: 
            coords (np.array), coordinates of the robot in the form of a column vector
        
        """

        coords = np.array([self.x,self.y,self.orientation])

        # put it in column vector form for kalman filter
        coords = np.transpose(coords)

        return coords
    
    @kalmanCoords.setter
    def kalmanCoords(self,coords):
        if coords is None or coords[0] is None or coords[1] is None or coords[2] is None:
            return
        
        self.x = coords[0]
        self.y = coords[1]
        self.orientation = coords[2]
        
    def updatePositions(self,thresholdTime, thresholdRadius):
        """
        Make the list of the last positions of the robot, and deletes the ones that are too old

        --------------------------------------------------------------
        Input:
            thresholdTime (float), time in seconds, if the position is older than this, it is deleted
            thresholdRadius (float), unused

        --------------------------------------------------------------
        Output: 
            None
        
        """
        # Adds the current position to the list
        self.coordsList.append((self.position[0],self.position[1], self.orientation, time.time()))

        # Deletes the positions that are too old
        self.coordsList = [i for i in self.coordsList if i[3] > time.time() - thresholdTime]

    def isStuck(self,thresholdTime, thresholdRadius):
        """
        Checks if the robot is stuck, by checking if all the points are in a radius of thresholdRadius

        --------------------------------------------------------------
        Input:
            thresholdTime (float), unused here
            thresholdRadius (float), radius in meters, if all the points are in this radius, the robot is considered stuck

        --------------------------------------------------------------
        Output: 
            isStuck (bool), True if the robot is stuck, False otherwise
        
        """
        # Calculates if all the points are in a radius of thresholdRadius
        if len(self.coordsList) > 0:
            meanX = np.mean([i[0] for i in self.coordsList])
            meanY = np.mean([i[1] for i in self.coordsList])

            for i in self.coordsList:
                if (i[0] - meanX)**2 + (i[1] - meanY)**2 > thresholdRadius**2:
                    return False

            return True
        else:
            print(f"Coords list is empty")
            return False

class Field:
    def __init__(self, robot = Robot(0,0), sizeLength = 8, cellsPerLength = 200):
        """
        Used at first to create the field storing all the different zones, and obstacles

        --------------------------------------------------------------
        Input:
            robot (Robot), robot object
            sizeLength (float), length of the field [m]
            cellsPerLength (int), number of cells per length of the field

        --------------------------------------------------------------
        Output: 
            None
        
        """
        self.robot = robot


        # everything after this wasn't used for the competition and is useless, put there for information
        # includes data for the astar algorithm

        self.sizeLength = sizeLength
        self.cellsPerLength = cellsPerLength

        self.cellSize = sizeLength / cellsPerLength

        self.baseGrid = np.zeros((cellsPerLength, cellsPerLength))
        self.obstacleGrid = np.zeros((cellsPerLength, cellsPerLength))

        # base grid numbers : 
        # 1 : zone 1 (biggest zone, in the middle)
        # 2 : zone 2 (carpet zone)
        # 3 : zone 3 (button zone)
        # 4 : zone 4 (platform zone)
        # 5 : wall
        # 6 : mid objective
        # 7 : good objective
        # 8 : button
        # 9 : door

        for i in range(cellsPerLength):
            for j in range(cellsPerLength):
                # there is 16 by 16 cells in the map calculate in which cell we are

                xCell = int(i * self.cellSize * 16 / self.sizeLength)
                yCell = int(j * self.cellSize * 16 / self.sizeLength)

                buttonLimits = [6, 10]
                carpetLimits = [8, 4]
                platformLimits = [10, 10]
                bigPointsLimits = [2, 2]
                midPointsLimits = [4, 4]

                if(xCell < bigPointsLimits[0] and yCell < bigPointsLimits[1]): #big points
                    self.baseGrid[i][j] = 7
                elif(xCell < midPointsLimits[0] and yCell < midPointsLimits[1]): #mid points
                    self.baseGrid[i][j] = 6
                elif(xCell < buttonLimits[0] and yCell >= buttonLimits[1]): #zone 3 with the button
                    self.baseGrid[i][j] = 3
                elif(xCell >= carpetLimits[0] and yCell < carpetLimits[1]): # zone 2 with the carpet
                    self.baseGrid[i][j] = 2
                elif(xCell >= platformLimits[0] and yCell >= platformLimits[1]): # zone 4 with the platform
                    self.baseGrid[i][j] = 4
                else: # zone 1
                    self.baseGrid[i][j] = 1

                # print(f"({i},{j}),({i * self.cellSize},{i * self.cellSize / 16}) : {self.baseGrid[i][j]}")

                ithCell = i * self.cellSize * 16 / self.sizeLength
                jthCell = j * self.cellSize * 16 / self.sizeLength
                iPlus1Cell = (i + 1) * self.cellSize * 16 / self.sizeLength
                jPlus1Cell = (j + 1) * self.cellSize * 16 / self.sizeLength

                if((ithCell < buttonLimits[0] and \
                iPlus1Cell >= buttonLimits[0] and \
                jthCell > buttonLimits[1] ) or \
                (jthCell <= buttonLimits[1] and \
                jPlus1Cell > buttonLimits[1] and \
                ithCell < buttonLimits[0] )):
                    self.baseGrid[i][j] = 5

                if((ithCell <= carpetLimits[0] and \
                iPlus1Cell > carpetLimits[0] and \
                jthCell < carpetLimits[1] ) or \
                (jthCell < carpetLimits[1] and \
                jPlus1Cell >= carpetLimits[1] and \
                ithCell > carpetLimits[0] )):
                    self.baseGrid[i][j] = 5

                if((ithCell <= platformLimits[0] and \
                iPlus1Cell > platformLimits[0] and \
                jthCell >= platformLimits[1] ) or \
                (jthCell <= platformLimits[1] and \
                jPlus1Cell > platformLimits[1] and \
                ithCell >= platformLimits[0] )):
                    self.baseGrid[i][j] = 5

        walls = self.baseGrid == 5
        self.obstacleGrid[walls] = 1




    def showMap(self):

        showGrid = np.transpose(self.baseGrid)
        
        # base grid numbers : 
        # 1 : zone 1 (biggest zone, in the middle)
        # 2 : zone 2 (carpet zone)
        # 3 : zone 3 (button zone)
        # 4 : zone 4 (platform zone)
        # 5 : wall
        # 6 : mid objective
        # 7 : good objective
        # 8 : button
        # 9 : door
        
        zones = [['white', 'gray', 'black', 'brown', 'black', 'green', 'yellow','green', 'orange'],
                 ["Zone 1", "Zone 2", "Zone 3", "Zone 4", "Wall", "Mid objective", "Good objective", "Button", "Door"]
                ]

        #discrete color scheme
        cMap = ListedColormap(zones[0]) 

        #data
        data = showGrid
        fig, ax = plt.subplots()
        heatmap = ax.pcolor(data, cmap=cMap)

        #legend
        cbar = plt.colorbar(heatmap)

        #square the grid
        ax.set_aspect('equal')

        plt.show()

    def showObstacles(self):
        # the obstacles map is a discret map, with it's values discret as well

        showGrid = np.transpose(self.obstacleGrid)

        # it should be shown like an image, with a color bar
        fig,ax = plt.subplots()

        heatmap = ax.pcolor(showGrid)
        ax.set_aspect('equal')

        #put a color bar
        cbar = plt.colorbar(heatmap)

        plt.show()


if __name__ == "__main__":
    f = Field()
    f.showMap()