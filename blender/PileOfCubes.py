import bpy

import math
import mathutils
import os

import numpy as np
import random
import copy

class Duplo():
    def __init__(self, xSize = 2, ySize = 2, zSize = 1):
        """
        Constructor of the Duplo class. It creates a Duplo object with the given dimensions.

        --------------------------------------------------------------
        Input:
            xSize (int)
            ySize (int)
            zSize (int)

        --------------------------------------------------------------
        Output: 
            None
        
        """

        # this position is for the bottom left corner of the Duplo (-x,-y,-z)
        self.x = 0
        self.y = 0
        self.z = 0

        self.xSize = xSize
        self.ySize = ySize
        self.zSize = zSize

        self.onTopOf = self
        
        self.correspondingObject = None

        self.probability = 1/(xSize*ySize*zSize)**2

    def setPos(self, x, y, z):
        self.x = x
        self.y = y 
        self.z = z

    @property
    def xMax(self):
        return self.x + self.xSize - 1
    
    @property
    def yMax(self):
        return self.y + self.ySize - 1
    
    @property
    def zMax(self):
        return self.z + self.zSize - 1
    
    @property
    def xmiddle(self):
        return self.x + self.xSize / 2
    
    @property
    def ymiddle(self):
        return self.y + self.ySize / 2
    
    @property
    def zmiddle(self):
        return self.z + self.zSize / 2

    def intersect(self, testedDuplo):
        return not (self.xMax < testedDuplo.x or self.x > testedDuplo.xMax or self.yMax < testedDuplo.y or self.y > testedDuplo.yMax or self.zMax < testedDuplo.z or self.z > testedDuplo.zMax)

def setupDuplosToChoose(duplosToChoose):
    """
    This function creates a list of Duplo objects with all the possible dimensions.

    --------------------------------------------------------------
    Input:
        duplosToChoose (list)

    --------------------------------------------------------------
    Output: 
        None
    
    """

    sum_of_dimentions = 0

    for duplo in bpy.data.objects:
        #the name of the duplos is 2x3
        if len(duplo.name) == 3 and duplo.name[0].isdigit() and duplo.name[1] == "x" and duplo.name[2].isdigit():
            # print(duplo.name)
            xSize = int(duplo.name[0])
            ySize = int(duplo.name[2])
            duplosToChoose.append(Duplo(xSize, ySize, 1))


    for duplo in duplosToChoose:
        # put the corresponding object in the duplo
        duplo.correspondingObject = bpy.data.objects.get(f"{duplo.xSize}x{duplo.ySize}")
        print(duplo.correspondingObject)







def printDuploPile(duploPile):
    """
    Show the pile of duplos in the console easily

    --------------------------------------------------------------
    Input:
        duploPile (list)

    --------------------------------------------------------------
    Output: 
        None
    
    """

    matrixSize = 15
    origin = (7,7,0)
    
    pileMatrix = np.zeros((matrixSize,matrixSize,7))
    for i,duplo in enumerate(duploPile):
        pileMatrix[(duplo.x+origin[0]):(duplo.xMax+origin[0]+1), (duplo.y+origin[1]):(duplo.yMax+origin[1]+1), duplo.z:duplo.zMax+1] = i+1
        print(f"duplo {i+1} : {duplo.xSize}x{duplo.ySize} at {duplo.x}=>{duplo.xMax} on {duplo.onTopOf.x}=>{duplo.onTopOf.xMax},{duplo.y}=>{duplo.yMax} on {duplo.onTopOf.y}=>{duplo.onTopOf.yMax},{duplo.z} ")

    # print the pile matrix, x,y planes one by one from the bottom
    for z in range(7):
        print(f"z={z}")

        for y in range(matrixSize):
            print(f"{y%10} : {pileMatrix[:,y,z]} : {y}")
            

def placeDuplos(duploPile):
    """
    Using the duploPile, place all the duplos in the scene.

    --------------------------------------------------------------
    Input:
        duploPile (list)

    --------------------------------------------------------------
    Output: 
        None
    
    """
    #places the duplos in the the collection "Assembly"

    distanceFactorXY = 0.015
    distanceFactorZ = 0.018
    assemblyCollection = bpy.data.collections.get("Assembly")
    bricksCollection = bpy.data.collections.get("Bricks")
    mainCollection = bpy.data.collections.get("Collection")


    
#    bpy.ops.object.empty_add(type='PLAIN_AXES',radius=0.1,location=(0,0,0))
#    duploAssembly = bpy.context.active_object
#    duploAssembly.name = "DuploAssembly"
    
    duplosList = []
    
    # Remove object from all collections not used in a scene
#    bpy.ops.collection.objects_remove_all()
#    # add it to our specific collection
##    mainCollection.objects.link(duploAssembly)

#    #clear the collection 
#    for obj in assemblyCollection.all_objects:
#        bpy.data.collections["Assembly"].objects.unlink(obj)
#        bpy.data.objects.remove(obj)
        

    for i,duplo in enumerate(duploPile):
#        print("placing duplo",i+1,"/",duploPile.size)
        # copy the object in the collection
        bpy.ops.object.select_all(action='DESELECT')
        duplo.correspondingObject.select_set(True)
        bpy.context.view_layer.objects.active = duplo.correspondingObject
        
        #duplicate the object
        bpy.ops.object.duplicate(linked=0)
        newDuplo = bpy.context.active_object

        #select the object to copy
#        newDuplo = bpy.context.active_object.copy()
        
        duplosList.append(newDuplo)

        # print("testDuplo",testDuplo)
        # print("newDuplo",newDuplo)
        
        

        bricksCollection.objects.unlink(newDuplo)
#        assemblyCollection.objects.link(newDuplo)
        mainCollection.objects.link(newDuplo)

#        print(duploAssembly)

        # newDuplo.parent = duploAssembly
        newDuplo.name = f"duplo_{i+1}_{duplo.xSize}x{duplo.ySize}"



        #move the object
        newDuplo.location = (duplo.x*distanceFactorXY, duplo.y*distanceFactorXY, duplo.z*distanceFactorZ)

        duplo.correspondingObject.select_set(False)

    print(duplosList)

    if bpy.context.active_object is not None:
        bpy.context.active_object.select_set(False)
    bpy.ops.object.select_all(action='DESELECT')
    #select an active object
    
    bpy.context.view_layer.objects.active = duplosList[0]
    for i,duplo in enumerate(duplosList):
        #apply a random material
#        duplo.active_material = random.choice(materials)
        #select the duplo
        duplo.select_set(True)

#    # join the duplos
    bpy.ops.object.join()

#    #rename the object
    bpy.context.active_object.name = "DuploAssembly"
    
def makeDuploPile(duploPile, duplosToChoose,numberOfDuplos = 1, maxHeight = 3):
    """
    Main function to generate the duplo piles (Assemblies)

    --------------------------------------------------------------
    Input:
        duploPile (list)
        duplosToChoose (list)
        numberOfDuplos (int)
        maxHeight (int)

    --------------------------------------------------------------
    Output: 
        None
    
    """

    bounds = [-5,5,-5,5,0,maxHeight]

    for i in range(numberOfDuplos):
        nouveauDuplo = copy.copy(random.choices(duplosToChoose, weights=[duplo.probability for duplo in duplosToChoose],k=1)[0])
        # DuploPile = np.append(DuploPile, random.choice(DuplosToChose))

        if duploPile.size == 0:
            duploPile = np.append(duploPile, nouveauDuplo) #type: ignore
        else:
            while True:
                #check if the duplo doesn't intersect with any other duplo

                #choose a random position for the duplo on top of another duplo
                #check if the duplo doesn't intersect with any other duplo

                randomDuplo = random.choice(duploPile)

                #choose a random position for the duplo on top of another duplo
                
                x = randomDuplo.x + random.randint( - nouveauDuplo.xSize + 1, randomDuplo.xSize - 1)
                y = randomDuplo.y + random.randint( - nouveauDuplo.ySize + 1, randomDuplo.ySize - 1)
                z = randomDuplo.z + randomDuplo.zSize

                nouveauDuplo.setPos(x, y, z)

                # print(f"Choosen Duplo ({randomDuplo.xSize}x{randomDuplo.ySize}, z={randomDuplo.z}) min and max x: {randomDuplo.x} to {randomDuplo.xMax} and y: {randomDuplo.y} to {randomDuplo.yMax}")
                # print(f"New Duplo ({nouveauDuplo.xSize}x{nouveauDuplo.ySize}, z={nouveauDuplo.z}) min and max x: {nouveauDuplo.xSize} to {nouveauDuplo.xMax} and y: {y} to {nouveauDuplo.yMax}")
                # print(1 - randomDuplo.xSize, randomDuplo.xSize)


                if nouveauDuplo.x < bounds[0] or nouveauDuplo.xMax > bounds[1] or nouveauDuplo.y < bounds[2] or nouveauDuplo.yMax > bounds[3] or nouveauDuplo.zMax > bounds[5]:
                    continue

                intesection = False

                #check if the duplo doesn't intersect with any other duplo
                for duploInPile in duploPile:
                    # print(f"Checking {nouveauDuplo.xSize}x{nouveauDuplo.ySize} at {nouveauDuplo.x},{nouveauDuplo.y},{nouveauDuplo.z} and {duploInPile.xSize}x{duploInPile.ySize} at {duploInPile.x},{duploInPile.y},{duploInPile.z}")
                    if nouveauDuplo.intersect(duploInPile):
                        intesection = True
                        print(f"- {nouveauDuplo.xSize}x{nouveauDuplo.ySize} intersects with {duploInPile.xSize}x{duploInPile.ySize}, positions : {nouveauDuplo.x},{nouveauDuplo.y},{nouveauDuplo.z} and {duploInPile.x},{duploInPile.y},{duploInPile.z}")
                        break

                if not intesection:
                    nouveauDuplo.onTopOf = randomDuplo
                    duploPile = np.append(duploPile, nouveauDuplo) #type: ignore
                    print(f"+ Added {nouveauDuplo.xSize}x{nouveauDuplo.ySize} at {nouveauDuplo.x},{nouveauDuplo.y},{nouveauDuplo.z}, duplo pile size : {duploPile.size}")
                    break
                
    if False:
        printDuploPile(duploPile)

    placeDuplos(duploPile)

    return duploPile
            

if __name__ == "__main__":
    duploPile = np.array([])
    duplosToChoose = []

    setupDuplosToChoose(duplosToChoose)
    makeDuploPile(duploPile,duplosToChoose,10,maxHeight = 4)


        