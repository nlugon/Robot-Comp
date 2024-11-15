#!/usr/bin/env python3
import serial
import time
import numpy as np
import matplotlib.pyplot as plt

from ComputerVision import *


def initArduinoSerial():
    """
    Initialize the serial communication with the arduino

    --------------------------------------------------------------
    Input:
        None

    --------------------------------------------------------------
    Output: 
        ser (serial.Serial)
    
    """
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1,inter_byte_timeout=0.1)
        time.sleep(0.5) # wait for serial to open
            
        if ser.isOpen(): 
            print("Arduino connected and opened")
            time.sleep(1)
            return ser
        else:
            print("Arduino connected but not opened")
            return None

    except:
        print("Couldn't open Arduino connection")
        return None

def sendCommand(ser,command:str): #
    """
    send a command to the arduino

    --------------------------------------------------------------
    Input:
        ser (serial.Serial)
        command (str)

    --------------------------------------------------------------
    Output: 
        None
    
    """
    if not ser.isOpen():
        print("Serial port is not open")
        return 1
    
    ser.write(str(command).encode())

    return 0

def listenArduino(arduino):
    """
    Listen to the arduino, read the console output

    --------------------------------------------------------------
    Input:
        variable (type) [unit]

    --------------------------------------------------------------
    Output: 
        variable (type) [unit]
    
    """

    time.sleep(0.01)
        
    answer = ""
    counter = 0
    maxiter = 1000
    
    while arduino.inWaiting()==0 and counter < maxiter:
        pass
        time.sleep(0.001)
        counter += 1
        if counter %1000 == 0:
            print(f"Listening waited {counter/1000}s")

    if counter >= maxiter:
        print("Listening failed")

    if arduino.inWaiting()>0:
        answer=arduino.readline()
        print(answer)
        
    arduino.flushInput()

    return answer

def clamp(n, minn, maxn):
    """
    Clamp a value between a min and a max

    --------------------------------------------------------------
    Input:
        n (float)
        minn (float)
        maxn (float)

    --------------------------------------------------------------
    Output: 
        n (float)
    
    """

    return max(min(maxn, n), minn)

def clampMovement(commands, isClamp = True):
    """
    Clamp the movement between -99 and 99

    --------------------------------------------------------------
    Input:
        commands (dict)
        isClamp (bool)

    --------------------------------------------------------------
    Output: 
        commands (dict)
    
    """
    deltaV = commands["deltaV"]
    vitesse = commands["vitesse"]
    

    vGauche = vitesse - deltaV
    vDroite = vitesse + deltaV
    
    if isClamp:
        vGauche = clamp(int(vGauche),0,99) #int between -99 and 99
        vDroite = clamp(int(vDroite),0,99)
    else:
    
        vGauche = clamp(int(vGauche),-99,99) #int between -99 and 99
        vDroite = clamp(int(vDroite),-99,99)
        
    commands["vGauche"] = vGauche
    commands["vDroite"] = vDroite

    return commands
  
def sendMovement(ser,commands,debug = False):
    """
    Calculate the movement and the command we send

    --------------------------------------------------------------
    Input:
        ser (serial.Serial)
        commands (dict)
        debug (bool)

    --------------------------------------------------------------
    Output: 
        result (str)
        (vGauche, vDroite) (float,float)
    
    """

    clamp = commands["name"] != "aligning" and commands["name"] != "aligningButton"
    commands = clampMovement(commands,isClamp = clamp)

    vGauche = commands["vGauche"]
    vDroite = commands["vDroite"]

    if commands["name"] == "alone":
        command = "a\n"
    elif commands["name"] == "unloading":
        command = "u\n"
    elif commands["name"] == "pushingButton":
        command = "b\n"
    else:
        
        formatted1 = "{:+03d}".format(vGauche)
        formatted2 = "{:+03d}".format(vDroite)

        command = "m" + formatted1 + formatted2 + "\n"
    
    if debug:
        print(f"Command sent : {command}")
    
    result = sendCommand(ser,command)

    return result, (vGauche, vDroite)

if __name__ == '__main__':

    # ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    # ser.reset_input_buffer()
    
    # while True:
    #     ser.write(b"Hello from Raspberry Pi!\n")
    #     line = ser.readline().decode('utf-8').rstrip()
    #     print(line)
    #     time.sleep(1)

    # with serial.Serial("/dev/ttyACM0", 115200, timeout=1) as arduino:
    arduino = initArduinoSerial()
    print(arduino)
    time.sleep(0.1) #wait for serial to open
    if arduino.isOpen(): 
        print("{} connected!".format(arduino.port))  
        try:
            while True:
                cmd=input("Enter command : ")
                # arduino.write(str(cmd).encode())
                result = sendCommand(arduino,cmd)
                # time.sleep(0.1) #wait for arduino to answer
                # while arduino.inWaiting()==0: pass 
                # if  arduino.inWaiting()>0: 
                #     answer=arduino.readline()
                #     print(answer)
                #     arduino.flushInput() #remove data after reading, 
                result = listenArduino(arduino)
                
        except KeyboardInterrupt:
            print("KeyboardInterrupt has been caught.")

