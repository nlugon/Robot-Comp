import bpy
from pascal_voc_writer import Writer
import random
from datetime import datetime
import math
import mathutils
import os

import numpy as np
import copy


# Code inspired by https://github.com/mantyni/Multi-object-detection-duplo

# Choose one of rendering option
#1. Render all
#2. Render batch
#3. Render individual parts
#4. Render backgrounds

run_type = 3

number_of_duplos = 15

proba_assembly = 0.8 # Probability of assembly.

num = 255 
colors = []
materials = []
background = []
objects = []
object_count = []
start_range = 0 
step_size = 15 # Default 15. Choose step size for part rotation when rendering indivdual parts. 
batch_size = 2000 # How many images to render. For me, more than 400 images @ 640x640 resolution crashes Blender.
i = 0

# Define colors, based on offical Duplo RGB colors: 
colors0=[(255,10, 10), # Red 
        (0, 255, 122), # Green  
        (0,156,253), # Blue 
        (243,213,10), # Yellow
        (243,243,230), # White
        # (232, 80, 156), # Cyan
        # (160,161,159) # Magenta
        ] 

# Set output resolution
r_settings = bpy.context.scene.render
r_settings.resolution_x = 500
r_settings.resolution_y = 500
r_settings.image_settings.color_mode ='RGBA'


print("######################")
print("Rendering duplo parts. ", datetime.now())
print("Run_type: ", run_type)
print()

# Define output directory for images and annotations
directory = "C:/Users/clare/Documents/Work documents/Cours/EPFL/Ma 2/LeRobotEnY/blender/renders"

print(not os.path.exists(directory+'/images'))
print(not os.path.exists(directory+'/annotations'))

if not os.path.exists(directory):
    print("Creating directory for output")
    os.makedirs(directory)

if not os.path.exists(directory+'/images'):
    print("Creating directory for images")
    os.makedirs(directory+'/images')

if not os.path.exists(directory+'/annotations'):
    os.makedirs(directory+'/annotations')

output_dir = directory
output_dir_images = output_dir + '/images'
output_dir_annotations = output_dir + '/annotations'

print("Rendering output directory: ", output_dir)

# FUNCTIONS: --------------------------------------------------------------------------------------------

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
        """
        Check if a duplo intersects another duplo using objects' coordinates.

        --------------------------------------------------------------
        Input:
            testedDuplo (Duplo)

        --------------------------------------------------------------
        Output: 
            intersect (bool)
        
        """

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
        # print("placing duplo",i+1,"/",duploPile.size)
        # copy the object in the collection
        bpy.ops.object.select_all(action='DESELECT')
        duplo.correspondingObject.select_set(True)
        bpy.context.view_layer.objects.active = duplo.correspondingObject
        
        #duplicate the object
        bpy.ops.object.duplicate(linked=0)
        newDuplo = bpy.context.active_object
        
        #select the object to copy
        # newDuplo = bpy.context.active_object.copy()
        
        duplosList.append(newDuplo)

        bricksCollection.objects.unlink(newDuplo)
        # assemblyCollection.objects.link(newDuplo)
        mainCollection.objects.link(newDuplo)

        # print(duploAssembly)

        # newDuplo.parent = duploAssembly
        newDuplo.name = f"duplo_{i+1}_{duplo.xSize}x{duplo.ySize}"

        #move the object
        newDuplo.location = (duplo.x*distanceFactorXY, duplo.y*distanceFactorXY, duplo.z*distanceFactorZ)

        duplo.correspondingObject.select_set(False)

    # print(duplosList)

    if bpy.context.active_object is not None:
        bpy.context.active_object.select_set(False)
    bpy.ops.object.select_all(action='DESELECT')
    #select an active object
    
    bpy.context.view_layer.objects.active = duplosList[0]
    for i,duplo in enumerate(duplosList):
        #apply a random material
        duplo.active_material = random.choice(materials)
        #select the duplo
        duplo.select_set(True)

    # join the duplos
    bpy.ops.object.join()

    #set the origin to the center of the object
    bpy.ops.object.origin_set(type='GEOMETRY_ORIGIN', center='MEDIAN')

    # rename the object
    bpy.context.active_object.name = "DuploAssembly"
    
def makeDuploPile(duploPile, duplosToChoose,numberOfDuplos = 1, maxHeight = 3, Verbose = False):
    """
    Main function to generate the duplo piles (Assemblies)

    --------------------------------------------------------------
    Input:
        duploPile (list)
        duplosToChoose (list)
        numberOfDuplos (int)
        maxHeight (int)
        Verbose (bool)

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

                if Verbose:
                    print(f"Choosen Duplo ({randomDuplo.xSize}x{randomDuplo.ySize}, z={randomDuplo.z}) min and max x: {randomDuplo.x} to {randomDuplo.xMax} and y: {randomDuplo.y} to {randomDuplo.yMax}")
                    print(f"New Duplo ({nouveauDuplo.xSize}x{nouveauDuplo.ySize}, z={nouveauDuplo.z}) min and max x: {nouveauDuplo.xSize} to {nouveauDuplo.xMax} and y: {y} to {nouveauDuplo.yMax}")
                    # print(1 - randomDuplo.xSize, randomDuplo.xSize)


                if nouveauDuplo.x < bounds[0] or nouveauDuplo.xMax > bounds[1] or nouveauDuplo.y < bounds[2] or nouveauDuplo.yMax > bounds[3] or nouveauDuplo.zMax > bounds[5]:
                    continue

                intesection = False

                #check if the duplo doesn't intersect with any other duplo
                for duploInPile in duploPile:
                    # print(f"Checking {nouveauDuplo.xSize}x{nouveauDuplo.ySize} at {nouveauDuplo.x},{nouveauDuplo.y},{nouveauDuplo.z} and {duploInPile.xSize}x{duploInPile.ySize} at {duploInPile.x},{duploInPile.y},{duploInPile.z}")
                    if nouveauDuplo.intersect(duploInPile):
                        intesection = True
                        if Verbose:
                            print(f"- {nouveauDuplo.xSize}x{nouveauDuplo.ySize} intersects with {duploInPile.xSize}x{duploInPile.ySize}, positions : {nouveauDuplo.x},{nouveauDuplo.y},{nouveauDuplo.z} and {duploInPile.x},{duploInPile.y},{duploInPile.z}")
                        break

                if not intesection:
                    nouveauDuplo.onTopOf = randomDuplo
                    duploPile = np.append(duploPile, nouveauDuplo) #type: ignore
                    if Verbose:
                        print(f"+ Added {nouveauDuplo.xSize}x{nouveauDuplo.ySize} at {nouveauDuplo.x},{nouveauDuplo.y},{nouveauDuplo.z}, duplo pile size : {duploPile.size}")
                    break
        
        # print(f"placing duplo {i+1}/{numberOfDuplos}")
                
    if Verbose:
        printDuploPile(duploPile)

    placeDuplos(duploPile)

    return duploPile


def deleteExistingDuplos(keepOne = False):
    """
    Delete all the existing duplos

    --------------------------------------------------------------
    Input:
        keepOne (bool)

    --------------------------------------------------------------
    Output: 
        None
    
    """

    #delete all the objects named "DuploAssembly"
    
    print("Deleting existing Duplo Assemblies...")

    is_first = True

    DuploAssemblies = [obj for obj in bpy.data.objects if obj.name.startswith("DuploAssembly")]

    for i,obj in enumerate(DuploAssemblies):
        print(f"Deleting {obj.name} ({i+1}/{len(DuploAssemblies)})")
        if keepOne and is_first:
            is_first = False
            continue
        else:
            bpy.data.objects.remove(obj, do_unlink=True)
            is_first = False
 

    # delete unused materials hat are not in the list of materials to use

    for material in bpy.data.materials:
        if material.users == 0 and material not in materials:
            bpy.data.materials.remove(material)

    #delete unused meshes

    for mesh in bpy.data.meshes:
        if mesh.users == 0:
            bpy.data.meshes.remove(mesh)


def makeNDuplosAssemblies(n = 5, deleteExisting = True, Verbose = False):
    """
    Make n duplo assemblies using the makeDuploPile function

    --------------------------------------------------------------
    Input:
        n (int)
        deleteExisting (bool)
        Verbose (bool)

    --------------------------------------------------------------
    Output: 
        None
    
    """

    if deleteExisting:
        deleteExistingDuplos()

    for i in range(n):
        numberOfDuplos = random.randint(3, 10)
        maxHeight = math.ceil(numberOfDuplos/2)
        makeDuploPile(duploPile,duplosToChoose,numberOfDuplos,maxHeight = maxHeight, Verbose=Verbose)
        if not Verbose:
            print(f"Created Duplo Assembly {i+1}/{n}")
        

 
# Update camera location:
def update_camera(camera, focus_point=mathutils.Vector((0.0, 0.0, 0.0)), distance=10.0):
    """
    Focus the camera to a focus point and place the camera at a specific distance from that
    focus point. The camera stays in a direct line with the focus point.

    --------------------------------------------------------------
    Input:
        camera (bpy.types.object)
        focus_point (mathutils.Vector)
        distance (float)

    --------------------------------------------------------------
    Output: 
        None
    
    """

    # print(f"Moving camera, focus point : {focus_point}, distance : {distance}, camera location : {camera.location}, rotation : {camera.rotation_euler}")
    
    looking_direction = camera.location - focus_point
    rot_quat = looking_direction.to_track_quat('Z', 'Y')

    camera.rotation_euler = rot_quat.to_euler()
    # Use * instead of @ for Blender <2.8
    camera.location = rot_quat @ mathutils.Vector((0.0, 0.0, distance))

    # print(f"Camera location : {camera.location}, rotation : {camera.rotation_euler}, looking direction : {looking_direction}")

# Functions to get annotaion bounding boxes 
def clamp(x, minimum, maximum):
    """
    Clamp a value between a minimum and a maximum

    --------------------------------------------------------------
    Input:
        x (float)
        minimum (float)
        maximum (float)

    --------------------------------------------------------------
    Output: 
        x (float)
    
    """

    return max(minimum, min(x, maximum))

def camera_view_bounds_2d(scene, cam_ob, me_ob):
    """
    Returns camera space bounding box of mesh object.

    Negative 'z' value means the point is behind the camera.

    Takes shift-x/y, lens angle and sensor size into account
    as well as perspective/ortho projections.

    --------------------------------------------------------------
    Input:
        scene (bpy.types.Scene)
        cam_ob (bpy.types.object)
        me_ob (bpy.types.object)

    --------------------------------------------------------------
    Output: 
        tuple of floats (x, y, width, height)
    
    """

    mat = cam_ob.matrix_world.normalized().inverted()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    mesh_eval = me_ob.evaluated_get(depsgraph)
    me = mesh_eval.to_mesh()
    me.transform(me_ob.matrix_world)
    me.transform(mat)

    camera = cam_ob.data
    frame = [-v for v in camera.view_frame(scene=scene)[:3]]
    camera_persp = camera.type != 'ORTHO'

    lx = []
    ly = []

    for v in me.vertices:
        co_local = v.co
        z = -co_local.z

        if camera_persp:
            if z == 0.0:
                lx.append(0.5)
                ly.append(0.5)
            # Does it make any sense to drop these?
            # if z <= 0.0:
            #    continue
            else:
                frame = [(v / (v.z / z)) for v in frame]

        min_x, max_x = frame[1].x, frame[2].x
        min_y, max_y = frame[0].y, frame[1].y

        x = (co_local.x - min_x) / (max_x - min_x)
        y = (co_local.y - min_y) / (max_y - min_y)

        lx.append(x)
        ly.append(y)

    min_x = clamp(min(lx), 0.0, 1.0)
    max_x = clamp(max(lx), 0.0, 1.0)
    min_y = clamp(min(ly), 0.0, 1.0)
    max_y = clamp(max(ly), 0.0, 1.0)

    mesh_eval.to_mesh_clear()

    r = scene.render
    fac = r.resolution_percentage * 0.01
    dim_x = r.resolution_x * fac
    dim_y = r.resolution_y * fac

    # Sanity check
    if round((max_x - min_x) * dim_x) == 0 or round((max_y - min_y) * dim_y) == 0:
        return (0, 0, 0, 0)

    return (
        round(min_x * dim_x),            # X
        round(dim_y - max_y * dim_y),    # Y
        round((max_x - min_x) * dim_x),  # Width
        round((max_y - min_y) * dim_y)   # Height
    )


# Render scene in PNG format
def render_scene(it):
    """
    Render the scene in PNG format

    --------------------------------------------------------------
    Input:
        it (int) : iteration number

    --------------------------------------------------------------
    Output: 
        None
    
    """
    bpy.context.scene.render.image_settings.file_format='PNG'
    bpy.context.scene.render.filepath = output_dir_images + "/%0.5d.png"%it
    bpy.ops.render.render(use_viewport = True, write_still=True)

# Export annotations of boundig boxes in VOC format
def save_annotations(object, it):
    """
    Save the annotations of the bounding boxes in VOC format

    --------------------------------------------------------------
    Input:
        object (bpy.types.object)
        it (int) : iteration number

    --------------------------------------------------------------
    Output: 
        None
    
    """
    writer = Writer(output_dir_images + "/%0.5d.png"%it, r_settings.resolution_x, r_settings.resolution_y)
    if object is not None:
        bound_x, bound_y, bound_w, bound_h = (camera_view_bounds_2d(bpy.context.scene, bpy.context.scene.camera, object))
        part_name = str(object.name).split(".", 1)
        writer.addObject(part_name[0], bound_x, bound_y, bound_x+bound_w, bound_y+bound_h)
    writer.save(output_dir_annotations + "/%0.5d.xml"%it)

# PROGRAM CODE: ----------------------------------------------------------------------------------------------------------

duploPile = np.array([])
duplosToChoose = []

setupDuplosToChoose(duplosToChoose)

camera_distance_ratio = 0.005
random_position_width = 0.1

showBackgrounds = False
changeLights = True

def randomizeLights(lights):
    """
    Randomize the lights in the scene

    --------------------------------------------------------------
    Input:
        lights (bpy.types.object)

    --------------------------------------------------------------
    Output: 
        None
    
    """
    global changeLights
     # randomly select the lights
    if changeLights:
        for l in lights:
            if random.random() < 0.8:
                l.hide_set(False)
                l.hide_render = False
            else:
                l.hide_set(True)
                l.hide_render = True

            #randomize the position, inside a sphere
            if random.random() < 0.2:
                radius = random.random() * 1 + 0.1
                theta = random.random() * 2 * math.pi
                phi = random.random() * math.pi
                l.location = (radius * math.sin(theta) * math.cos(phi), radius * math.sin(theta) * math.sin(phi), radius * math.cos(theta))

# Normalise color values. Blender requires colors to be define between 0-1. 
for c in colors0:
    c = [x/num for x in c]
    c.append(1) # add 4th element alpha = 1, in case PNG format is required
    colors.append(c)
    
# Create list of materials.
for i in range(len(colors)):
    color_name = "color_"+str(i)
    mat1 = bpy.data.materials.new(color_name)    
    mat1.diffuse_color = colors[i]
    r1 = random.randint(0,1)
    if r1 > 0:
        mat1.shadow_method = ("NONE")
    else:
        mat1.shadow_method = ("OPAQUE")
    materials.append(mat1)

# create a list of all the lights in the scene
lights = []

for obj in bpy.data.collections['Collection'].all_objects:
    if (obj.type == 'LIGHT'):
        lights.append(obj)


# Create list of backgrounds
# Backgrounds are located in 'Collection 2' object container
for obj_bg in bpy.data.collections['Collection 2'].all_objects:
#    print(obj_bg.type + " " + obj_bg.name)
    if (obj_bg.type == 'MESH' or obj_bg.type == 'IMAGE' or obj_bg.type == 'EMPTY'):
        background.append(obj_bg)


makeNDuplosAssemblies(number_of_duplos)

# Create list of duplo part objects 
for obj in bpy.data.collections['Collection'].all_objects:
    if (obj.type == 'MESH'):
        objects.append(obj)
        
        

# Create a table to count every object appearance in rendering batches
# Used to check if random selection of parts has a uniform distribution
# Prints the table after script is finished
for x in objects:
    object_count.append([x.name]) # Part name
 
for c in range(0,len(objects)):
    object_count[c].append(0) # Number of times part has been used


# Reset camera location and orientation towards an object
objects[0].select_set(True)
objects[0].location = (0,0,0)
objects[0].rotation_euler = (0, 0, 0)
bpy.data.objects['Camera'].location = (0,0.5,0)#0.5 * camera_distance_ratio
bpy.data.objects['Camera'].rotation_euler = (0,0,0)
#bpy.ops.view3d.camera_to_view_selected()
# If resolution not 300x300, adjust y (-1.0) to suit, otherwise object part might be out of the picture.
update_camera(bpy.data.objects['Camera'],focus_point=mathutils.Vector((0.0, 0, 0)), distance=0.5)

# Hide all objects and backgrounds
for x in objects:
    x.hide_set(True)
    x.hide_render = True

for bg in background:
    bg.hide_set(True)
    bg.hide_render = True


# GENERATING INDIVIDUAL OBJECT RENDERS

# Render backgrounds
if (run_type == 1 or run_type == 4): 
    # Render few images without parts, only background 
    # This is not necessary (but recommended) since most object detection networks can handle empty scenes 
    for rr in range(0,3): 
        for bg in background:
            bg.hide_set(False) # Unhide one background
            bg.hide_render = False # Make it visible in renderings

            render_scene(start_range)
            save_annotations(None, start_range) # No object to annotate so passing None
            
            start_range += 1
            
            bg.hide_set(True)
            bg.hide_render = True
            
# Render individual parts rotated around all axes
if (run_type == 1 or run_type == 3):
    # Set camera to default location for rendering individual parts
    bpy.data.objects['Camera'].location = (0,0.5,0)
    bpy.data.objects['Camera'].rotation_euler = (0,0,0)
    update_camera(bpy.data.objects['Camera'],focus_point=mathutils.Vector((0.0, -0.1, 0)), distance=0.5)

    number_to_render_individually = len(objects) / 5 # Number of objects to render individually
    objects_to_render = random.sample(objects, int(number_to_render_individually)) # Randomly select objects to render individually

    for x in objects_to_render: 

        cam_distance = 6 * min(x.dimensions)
        random_position_width = 0.15 * cam_distance
        update_camera(bpy.data.objects['Camera'],focus_point=mathutils.Vector((0.0, -0.1, 0)), distance=cam_distance)

        # Check if the part is not a dublicate so not to render twise same parts. Duplicates have suffix ".001"
        
        x.hide_set(False)
        x.hide_render = False
        x.select_set(True)    
        
        # Randomise part location a bit
        x.location = (0,0,0)
        x.rotation_euler = (0, 0, 0)
        
        # rotate around x
        for x_or in range(0,360,step_size):
            
            # Adjust part rotation
            x.rotation_euler = (math.radians(x_or), 0, 0)
                
            # Choose material randomly
            # r1 = random.randint(0, len(materials)-1) 
            # x.active_material = materials[r1]
            
            # Hide all backgrounds    
            for bg in background:
                bg.hide_set(True)
                bg.hide_render = True
                
            # unhide one background randomly
            if showBackgrounds:
                r3 = random.randint(0,len(background)-1)
                background[r3].hide_set(False)
                background[r3].hide_render = False     

            randomizeLights(lights)

            render_scene(start_range)
            save_annotations(x, start_range)
        
            # Reset orientation & randommise location of the object
            x.location = (round(random.uniform(-random_position_width, random_position_width), 5), 0, round(random.uniform(-random_position_width, random_position_width), 5))
            x.rotation_euler = (0, 0, 0)

            # increase counter
            start_range += 1   
        
        x.location = (0,0,0)
        x.rotation_euler = (0, 0, 0)
        
        
        # rotate around y
        for y_or in range(0,360,step_size):
            
            x.rotation_euler = (0, math.radians(y_or), 0)
            
            # r1 = random.randint(0, len(materials)-1) 
            # x.active_material = materials[r1]
            
            for bg in background:
                bg.hide_set(True)
                bg.hide_render = True
            
            if showBackgrounds:    
                r3 = random.randint(0,len(background)-1)
                background[r3].hide_set(False)
                background[r3].hide_render = False   
                
            randomizeLights(lights)
                
            render_scene(start_range)
            save_annotations(x, start_range)
            
            x.location = (round(random.uniform(-random_position_width, random_position_width), 5), 0, round(random.uniform(-random_position_width, random_position_width), 5))
            x.rotation_euler = (0, 0, 0)

            start_range += 1   

        
        x.location = (0,0,0)
        x.rotation_euler = (0, 0, 0)
        
        
        # rotate around z
        for z_or in range(0,360,step_size):
            
            x.rotation_euler = (0, 0, math.radians(z_or))
        
            # r1 = random.randint(0, len(materials)-1) 
            # x.active_material = materials[r1]
            
            for bg in background:
                bg.hide_set(True)
                bg.hide_render = True
                
            if showBackgrounds:
                r3 = random.randint(0,len(background)-1)
                background[r3].hide_set(False)
                background[r3].hide_render = False    

            randomizeLights(lights)

            render_scene(start_range)
            save_annotations(x, start_range)
            
            x.location = (round(random.uniform(-random_position_width, random_position_width), 5), 0, round(random.uniform(-random_position_width, random_position_width), 5))
            x.rotation_euler = (0, 0, 0)

            start_range += 1   
        

        # Hide the part after finished rendering it's set
        x.hide_set(True)
        x.hide_render = True
    # END OF GENERATING INDIVIDUAL PARTS

if (run_type == 1 or run_type == 2):

    # CODE FOR BATCH RENDERING
    path, dirs, files = next(os.walk(output_dir+"/images"))
    # file_count = len(files)
    # start_range = files[-1].split(".")[0]
    # start_range = int(start_range) + 1 

    start_range = 6000
    
    # start_range = 0 # Modify start_range to start from previous renders of individual parts
    image_set = start_range + batch_size

    print("Printing batch of images. Start_range from: ", start_range)
    
    # Deselect all objects
    for iii in range(start_range,image_set):

        
        bpy.ops.object.select_all(action='DESELECT')
        
        # Hide all objects
        for x in objects:
            x.hide_set(True)
            x.hide_render = True
            
        number_of_duplo_parts = random.randint(2, min(len(objects),6))
        # Get a maximum number of 5 random duplo part objects 
        list_of_objects_numbers = random.sample(range(len(objects)), number_of_duplo_parts)      
        # print("Duplo parts selected: ", list_of_objects_numbers)

        print(f"{iii - start_range} / {batch_size} with parts {list_of_objects_numbers} ")	

        for obj_num in list_of_objects_numbers: 
            
            
            # r1 = random.randint(0, len(materials)-1) 
            # r2 = random.randint(0,1) # random number to decide in the part is in the scene
            
            x = objects[obj_num]

        # Unhide and select duplo object that's in the list
            x.hide_set(False)
            x.hide_render = False
            x.select_set(True)

        # Move duplo object to a random location within given constriants
            x.location = (round(random.uniform(-random_position_width*4, random_position_width*4), 4), 0, round(random.uniform(-random_position_width*4, random_position_width*4), 4))

            # Duplo orientation randomisation
            x.rotation_euler = (math.radians(random.randint(0,180)), math.radians(random.randint(0,180)), math.radians(random.randint(0,180)))

            # Duplo material randomisation
#            x.active_material = materials[r1]
            
        # Update duplo part counting table
        cc = 0
        for x in objects:
            if x.hide_render == False:
                object_count[cc][1] += 1
            cc += 1
            
            
        # Background randomisation
        # Hide all backgrounds
        for bg in background:
            bg.hide_set(True)
            bg.hide_render = True
        
        # unhide one background randomly
        if showBackgrounds:
            r3 = random.randint(0,len(background)-1)
            background[r3].hide_set(False)
            background[r3].hide_render = False    

        randomizeLights(lights)
                
        # Fit camera scene within the objects
        bpy.ops.view3d.camera_to_view_selected()

        render_scene(iii)
        
        save_annotations(x, start_range)

        writer = Writer(output_dir_images + "/%0.5d.png"%iii, r_settings.resolution_x, r_settings.resolution_y)

        # Save annotations
        for x in objects:
            if x.hide_render == False:
                bound_x, bound_y, bound_w, bound_h = (camera_view_bounds_2d(bpy.context.scene, bpy.context.scene.camera, x))
                part_name = str(x.name).split(".", 1)
                # print(part_name)
                # Save annotations of rectangle around the object: x_min, y_min, x_max, y_max
                writer.addObject(part_name[0], bound_x, bound_y, bound_x+bound_w, bound_y+bound_h)
                
        writer.save(output_dir_annotations + "/%0.5d.xml"%iii)


    # Print out times each Duplo object was used
    print("Object count: ")
    for cc in object_count:
        print(cc)
    # END OF BATCH RENDERING CODE

deleteExistingDuplos(keepOne = True)

print("All done.")

