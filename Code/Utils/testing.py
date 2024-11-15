from classes import *
import random
#from perlin_noise import PerlinNoise

#noise = PerlinNoise(octaves=3.5, seed=69)

# set the seed for the random number generator
random.seed(0)


isTestingTime = False


def getTestingData():
    """
    Used for testing purposes, returns a dictionary with the field and robot objects

    --------------------------------------------------------------
    Input:
        None

    --------------------------------------------------------------
    Output: 
        data (dict)
    
    """
    robot = Robot(0,0)
    field = Field()

    field.robot = robot

    #put some perlin noise in the obstacle grid

    for i in range(field.cellsPerLength):
        for j in range(field.cellsPerLength):
            # print(i,j)
            field.obstacleGrid[i][j] = 4#int((noise([i/field.cellsPerLength,j/field.cellsPerLength]) + 0.5) * 10)

    data = {
        "field": field,
        "robot": robot,
    }

    return data

if __name__ == "__main__":
    data = getTestingData()

    field = data["field"]
    field.showObstacles()