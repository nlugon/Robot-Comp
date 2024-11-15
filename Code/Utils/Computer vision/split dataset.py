import os
import shutil
from PIL import Image



# the goal of this script is to split the data set into training, validation and test sets

training_percentage = 0.8
validation_percentage = 0.1
test_percentage = 0.1

# the path to the data set
data_set_path = r"C:\Users\clare\Documents\Work documents\Cours\EPFL\Ma 2\LeRobotEnY\blender\Images for training"

# the path to the folder where the data set will be split
split_data_set_path = r"C:\Users\clare\Documents\Work documents\Cours\EPFL\Ma 2\LeRobotEnY\blender\Images for training\Split data set"

# the path to the folder where the sets will be saved
training_path = os.path.join(split_data_set_path, "training")
validation_path = os.path.join(split_data_set_path, "validation")
test_path = os.path.join(split_data_set_path, "test")


def split():
    # create the folders
    if not os.path.exists(split_data_set_path):
        os.mkdir(split_data_set_path)
    if not os.path.exists(training_path):
        os.mkdir(training_path)
    if not os.path.exists(validation_path):
        os.mkdir(validation_path)
    if not os.path.exists(test_path):
        os.mkdir(test_path)

    # get the list of the folders in the data set
    folders = os.listdir(data_set_path)

    # for each folder in the data set
    for folder in folders:
        # get the path to the folder
        folder_path = os.path.join(data_set_path, folder)
        if folder_path == split_data_set_path:
            continue

        # get the list of the files in the folder
        files = os.listdir(folder_path)
        # get the number of files in the folder
        number_of_files = len(files)
        # get the number of files in each set
        training_number_of_files = int(number_of_files * training_percentage)
        validation_number_of_files = int(number_of_files * validation_percentage)
        test_number_of_files = int(number_of_files * test_percentage)
        # get the list of the files in each set
        training_files = files[:training_number_of_files]
        validation_files = files[training_number_of_files:training_number_of_files + validation_number_of_files]
        test_files = files[training_number_of_files + validation_number_of_files:]
        # create the folders for the sets
        training_folder_path = os.path.join(training_path, folder)
        validation_folder_path = os.path.join(validation_path, folder)
        test_folder_path = os.path.join(test_path, folder)
        if not os.path.exists(training_folder_path):
            os.mkdir(training_folder_path)
        if not os.path.exists(validation_folder_path):
            os.mkdir(validation_folder_path)
        if not os.path.exists(test_folder_path):
            os.mkdir(test_folder_path)
        # copy the files in each set
        for file in training_files:
            file_path = os.path.join(folder_path, file)
            new_file_path = os.path.join(training_folder_path, file)
            shutil.move(file_path, new_file_path)
        for file in validation_files:
            file_path = os.path.join(folder_path, file)
            new_file_path = os.path.join(validation_folder_path, file)
            shutil.move(file_path, new_file_path)
        for file in test_files:
            file_path = os.path.join(folder_path, file)
            new_file_path = os.path.join(test_folder_path, file)
            shutil.move(file_path, new_file_path)


def transform2JPG(folder_path):
   
    # for each folder in the folder_path recusively call the function
    folders = os.listdir(folder_path)
    
    print(f"folder path: {folder_path}, folder {folders}")

    for file in folders:
        current_file_path = os.path.join(folder_path, file)
        if os.path.isdir(current_file_path):
            transform2JPG(current_file_path)
            continue

        # check if it's a png file
        if file.endswith(".png"):
            #if a file with the same jpg name doesn't exist
            if not os.path.exists(os.path.join(folder_path, file[:-4] + ".jpg")):
                #open it and save it as a jpg file
                file_path = os.path.join(folder_path, file)
                image = Image.open(file_path)
                new_file_path = os.path.join(folder_path, file[:-4] + ".jpg")
                image.save(new_file_path)
                print(f"changed file {file} to {file[:-4] + '.jpg'}")
            
            #delete the png file 
            os.remove(os.path.join(folder_path, file))
            continue

        # if it's an xml file
        elif file.endswith(".xml"):
            # change everything inside with png to jpg
            file_path = os.path.join(folder_path, file)
            with open(file_path, "r") as f:
                xml = f.read()
            xml = xml.replace(".png", ".jpg")
            #new_file_path = os.path.join(folder_path, file[:-4] + ".xml")

            # raplace the class 'duplo_1_8x2' by DuploAssembly
            # xml = xml.replace("duplo_1_8x2", "DuploAssembly")
            # xml = xml.replace("duplo_1_2x8", "DuploAssembly")
            

            with open(file_path, "w") as f:
                f.write(xml)
            print(f"changed file {file}")


transform2JPG(split_data_set_path)