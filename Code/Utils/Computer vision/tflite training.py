import numpy as np
import os

from tflite_model_maker.config import ExportFormat, QuantizationConfig
from tflite_model_maker import model_spec
from tflite_model_maker import object_detector

from tflite_support import metadata

import tensorflow as tf
assert tf.__version__.startswith('2')

# tf.get_logger().setLevel('ERROR')
from absl import logging
# logging.set_verbosity(logging.ERROR)

print(tf.config.list_physical_devices())

# path_to_images = "C:/Users/clare/Documents/Work documents/Cours/EPFL/Ma 2/LeRobotEnY/blender/renders/images"
# path_to_annotations = "C:/Users/clare/Documents/Work documents/Cours/EPFL/Ma 2/LeRobotEnY/blender/renders/annotations"

path_to_test_images = r"C:\Users\clare\Documents\Work documents\Cours\EPFL\Ma 2\LeRobotEnY\blender\Images for training\Split data set\test\Images"
path_to_test_annotations = r"C:\Users\clare\Documents\Work documents\Cours\EPFL\Ma 2\LeRobotEnY\blender\Images for training\Split data set\test\Annotations"
path_to_training_images = r"C:\Users\clare\Documents\Work documents\Cours\EPFL\Ma 2\LeRobotEnY\blender\Images for training\Split data set\training\Images"
path_to_training_annotations = r"C:\Users\clare\Documents\Work documents\Cours\EPFL\Ma 2\LeRobotEnY\blender\Images for training\Split data set\training\Annotations"
path_to_validation_images = r"C:\Users\clare\Documents\Work documents\Cours\EPFL\Ma 2\LeRobotEnY\blender\Images for training\Split data set\validation\Images"
path_to_validation_annotations = r"C:\Users\clare\Documents\Work documents\Cours\EPFL\Ma 2\LeRobotEnY\blender\Images for training\Split data set\validation\Annotations"


print(f"loading images from {path_to_training_images} and {path_to_validation_images}")

train_data = object_detector.DataLoader.from_pascal_voc(
    path_to_training_images,
    path_to_training_annotations,
    ["DuploAssembly"]
)

val_data = object_detector.DataLoader.from_pascal_voc(
    path_to_validation_images,
    path_to_validation_annotations,
    ["DuploAssembly"]
)

# print(train_data)

print("training model")
spec = model_spec.get('efficientdet_lite0')
model = object_detector.create(train_data, model_spec=spec, batch_size=4, train_whole_model=True, epochs=10, validation_data=val_data)

# don't override all the previous models

modelName = "BigBossTraining.tflite"

model.export(export_dir='.', tflite_filename=modelName)




