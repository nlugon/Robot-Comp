import os
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt


import shutil

path = "C:/Users/clare/Documents/Work documents/Cours/EPFL/Ma 2/LeRobotEnY/blender/Models Images"

imagesPath = os.path.join(path, "Images")
annotationPath = os.path.join(path, "Annotations")
backgroundPath = os.path.join(path, "Backgrounds")

newImagesPath = os.path.join(path, "NewImages")
newAnnotationPath = os.path.join(path, "NewAnnotations")

alreadyDoneFiles = os.path.join(path, "alreadyDone.txt")

def changeImage(img):

    
    # add background behind the image, where the image is transparent, the image should be the size of img

    alphaBackground = img[:, :, 3] / 255.0	
    alphaBackground = cv.merge( (alphaBackground, alphaBackground, alphaBackground) )

    # take a random background
    background = cv.imread(os.path.join(backgroundPath, np.random.choice(os.listdir(backgroundPath))), cv.IMREAD_UNCHANGED)
    # if there is no alpha channel, add one
    if background.shape[2] == 3:
        background = cv.cvtColor(background, cv.COLOR_BGR2BGRA)


    #if it's not big enough
    while background.shape[0] < img.shape[0]:
        # put them next to each other
        background = np.concatenate((background, background), axis=0)
    while background.shape[1] < img.shape[1]:
        # put them next to each other
        background = np.concatenate((background, background), axis=1)


    


    # take a random position
    x = np.random.randint(0, background.shape[1] - img.shape[1])
    y = np.random.randint(0, background.shape[0] - img.shape[0])

    #change contrast and brightness
    alpha = np.random.uniform(0.5, 1.5)
    beta = np.random.randint(-25, 25)
    img = cv.convertScaleAbs(img, alpha=alpha, beta=beta)

    
    #take the background in the right place :
    background = background[y:y+img.shape[0], x:x+img.shape[1]]

    # same for background
    alpha = np.random.uniform(0.5, 1.5)
    beta = np.random.randint(-25, 25)
    background = cv.convertScaleAbs(background, alpha=alpha, beta=beta)



    # blend the two images using the alpha channel as controlling mask
    newimg = cv.convertScaleAbs(img[:, :, :3] * alphaBackground + background[:, :, :3] * (1 - alphaBackground))

    return newimg



def dataAugment():

    #if the alreadyDone.txt file doesn't exist, create it
    if not os.path.exists(alreadyDoneFiles):
        with open(alreadyDoneFiles, "w") as f:
            f.write("")

    # same for all the new folders
    if not os.path.exists(newImagesPath):
        os.makedirs(newImagesPath)

    if not os.path.exists(newAnnotationPath):
        os.makedirs(newAnnotationPath)

    # Read the images
    with open(alreadyDoneFiles, "r") as f:
        alreadyDoneFilesList = f.read().splitlines()
        for file in os.listdir(imagesPath):
            if file.endswith(".png") and file not in alreadyDoneFilesList:

                
                # save annotation name name.xml
                annotationName = file.split(".")[0] + ".xml"

                # test if the annotation file exists
                if not os.path.exists(os.path.join(annotationPath, annotationName)):
                    #delete the image
                    os.remove(os.path.join(imagesPath, file))
                    print(f"file {file} doesn't have an annotation file")
                    continue

                img = cv.imread(os.path.join(imagesPath, file), cv.IMREAD_UNCHANGED)
                

                print(f"doing file {file}")
                #change image
                img = changeImage(img)

                # save image
                cv.imwrite(os.path.join(newImagesPath,file), img)

                # save file name
                with open(alreadyDoneFiles, "a") as f:
                    f.write(file + "\n")
                # show image
                plt.pause(0.1)
                plt.close()

                # copy the annotation file that has the same name.xml with shutils
                shutil.copyfile(os.path.join(annotationPath, annotationName), os.path.join(newAnnotationPath, annotationName))
                print(f"file {file} done")




if __name__ == "__main__":
    dataAugment()