import time
import numpy as np

from utils import *

angleList = [0]

def makeMovement(objective = {"X":1,"Y":1,"W":2,"H":2}):
    """
    controls the indicated movement by the navigation

    --------------------------------------------------------------
    Input:
        objective (dict), contains the objective of the robot

    --------------------------------------------------------------
    Output: 
        commands (dict), contains the commands for the robot
    
    """
    coefSpeed = 1

    name = objective["name"]

    if name == "duplo":
        coefSpeed = 0.7

    else:
        coefSpeed = 1

    deltaV = calcDeltaV(objective) * coefSpeed
    vitesse = calcVitesse(objective) * coefSpeed

    # instead of creating a new object, as to the objective, we could just modify the objective
    objective["deltaV"] = deltaV
    objective["vitesse"] = vitesse

    return objective



def calcDeltaV(objective):
    """
    calculates the deltaV in function of the angle, 
    using a PID controler, for now the robot always move in a straight line (no curve turns)
    => error for the angle is always angle - 0

    --------------------------------------------------------------
    Input:
        angle (float) [rad], angle en degres (0 = in front)

    --------------------------------------------------------------
    Output: 
        deltaV (float) [mm/s], deltaV
    
    """

    angle = objective["angle"]
    distance = objective["distance"]

    #vitesse [m/s]
    coefVitesse = 20
   
    deltaV = angle * coefVitesse

    return deltaV


def calcVitesse(objective):
    """
    calculates the speed in function of the distance, 
    if the robot is close the speed is inversely proportional to the distance to the objective (the closer the slower)
    To determine if it's good or not, we need to test it

    --------------------------------------------------------------
    Input:
        distance (float) [mm], distance a l'objectif

    --------------------------------------------------------------
    Output: 
        vitesse (float) [mm/s], vitesse
    
    
    """

    angle = objective["angle"]
    distance = objective["distance"]

    threasholdIsClose = 1 #distance [mm]
    
    vitesseNominale = 100
    vitesseMin = 10 #vitesse [mm/s]

    if distance < threasholdIsClose:
        # make a function that goes from vitesseNominale to vitesseMin in an inversily proportional way
        vitesse = vitesseMin + (vitesseNominale - vitesseMin) * (distance) / threasholdIsClose
        # vitesse = vitesseNominale
        # print(f"vitesse : {vitesse}")	
    else:
        vitesse = vitesseNominale

    angleThresholdTurning = 100 / rad2deg
    
    angleMultiplier = (angleThresholdTurning - abs(angle)) / angleThresholdTurning # fait baisser la vitesse en fonction de l'angle, si l'ange est trop élevé, il n'avance pas
    angleMultiplier = max(0, angleMultiplier)
    vitesse = vitesse * angleMultiplier

    return vitesse