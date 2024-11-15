import numpy as np
import os

import cv2

import numpy as np
from PIL import Image
import tensorflow as tf

import time
import cv2
import pathlib


assert tf.__version__.startswith('2')

#@title Run object detection and show the detection results


INPUT_IMAGE_FOLDER_URL = "C:/Users/clare/Documents/Work documents/Cours/EPFL/Ma 2/LeRobotEnY/Code/Computer vision/Test Dataset/Original" #@param {type:"string"}
INPUT_IMAGE_URL = "C:/Users/clare/Documents/Work documents/Cours/EPFL/Ma 2/LeRobotEnY/Code/Computer vision/Test Dataset/Original/IMG_20230407_124233.jpg" #@param {type:"string"}
DETECTION_THRESHOLD = 0.6 #@param {type:"number"}
TFLITE_MODEL_PATH = "C:/Users/clare/Documents/Work documents/Cours/EPFL/Ma 2/LeRobotEnY/Code/Computer vision/firstTraining.tflite" #@param {type:"string"}

interpreter = tf.lite.Interpreter(model_path=TFLITE_MODEL_PATH)
interpreter.allocate_tensors()
_, HEIGHT, WIDTH, _ = interpreter.get_input_details()[0]['shape']
print(f"Height and width accepted by the model: {HEIGHT, WIDTH}")


def preprocess_image(image_path):
    img = tf.io.read_file(image_path)
    img = tf.io.decode_image(img, channels=3)
    img = tf.image.convert_image_dtype(img, tf.float32)
    original_image = img
    resized_img = tf.image.resize(img, (HEIGHT, WIDTH))
    resized_img = resized_img[tf.newaxis, :]
    return resized_img, original_image









# # Load TFLite model and allocate tensors.
# interpreter = tf.lite.Interpreter(model_path=TFLITE_MODEL_PATH)

# # Get input and output tensors.
# input_details = interpreter.get_input_details() 
# output_details = interpreter.get_output_details()

# interpreter.allocate_tensors()

# # input details
# print(input_details)

# # output details
# print(output_details)


# def draw_rect(image, box):
#     y_min = int(max(1, (box[0] * image.height)))
#     x_min = int(max(1, (box[1] * image.width)))
#     y_max = int(min(image.height, (box[2] * image.height)))
#     x_max = int(min(image.width, (box[3] * image.width)))
    
#     # draw a rectangle on the image
#     cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (255, 255, 255), 2)

# for file in pathlib.Path(folder_path).iterdir():
#     # read and resize the image
#     img = cv2.imread(r"{}".format(file.resolve()))
#     new_img = cv2.resize(img, (320, 320))

#     # input_details[0]['index'] = the index which accepts the input
#     interpreter.set_tensor(input_details[0]['index'], [new_img])

#     # run the inference
#     interpreter.invoke()

#     # output_details[0]['index'] = the index which provides the input
#     output_data = interpreter.get_tensor(output_details[1]['index'])
#     rects = interpreter.get_tensor(output_details[0]['index'])
#     scores = interpreter.get_tensor(output_details[2]['index'])


#     print(scores)
#     print(rects)

#     # print("For file {}, the output is {}".format(file.stem, output_data))



#     for index, score in enumerate(scores[0]):
#         if score > 0.5:
#           draw_rect(new_img,rects[0][index])


#     # show the image
#     cv2.imshow("image", new_img)
#     cv2.waitKey(0)


def set_input_tensor(interpreter, image):
  """Sets the input tensor."""
  tensor_index = interpreter.get_input_details()[0]['index']
  input_tensor = interpreter.tensor(tensor_index)()[0]
  input_tensor[:, :] = image


def get_output_tensor(interpreter, index):
  """Returns the output tensor at the given index."""
  output_details = interpreter.get_output_details()[index]
  tensor = np.squeeze(interpreter.get_tensor(output_details['index']))
  return tensor


def detect_objects(interpreter, image, threshold):
  """Returns a list of detection results, each a dictionary of object info."""
  set_input_tensor(interpreter, image)
  interpreter.invoke()

  # Get all output details
  boxes = get_output_tensor(interpreter, 0)
  classes = get_output_tensor(interpreter, 1)
  scores = get_output_tensor(interpreter, 2)
  count = get_output_tensor(interpreter, 3)

  print(classes)

  results = []
  for i in range(len(boxes)):
    if scores[i] >= threshold:
      result = {
          'bounding_box': boxes[i],
          'class_id': classes[i],
          'score': scores[i]
      }
      results.append(result)
  return results

LABEL_DICT = {1: 'DuploAssembly',
}

COLORS = np.random.randint(0, 255, size=(len(LABEL_DICT), 3), 
                            dtype="uint8")

def display_results(image_path, threshold=0.3):
    # Load the input image and preprocess it
    preprocessed_image, original_image = preprocess_image(image_path)
    # print(preprocessed_image.shape, original_image.shape)

    # =============Perform inference=====================
    start_time = time.monotonic()
    results = detect_objects(interpreter, preprocessed_image, threshold=threshold)
    print(f"Elapsed time: {(time.monotonic() - start_time)*1000} miliseconds")

    # =============Display the results====================
    original_numpy = original_image.numpy()
    for obj in results:
        # Convert the bounding box figures from relative coordinates
        # to absolute coordinates based on the original resolution
        ymin, xmin, ymax, xmax = obj['bounding_box']
        xmin = int(xmin * original_numpy.shape[1])
        xmax = int(xmax * original_numpy.shape[1])
        ymin = int(ymin * original_numpy.shape[0])
        ymax = int(ymax * original_numpy.shape[0])

        # Grab the class index for the current iteration
        idx = int(obj['class_id'])
        # Skip the background
        if idx >= len(LABEL_DICT):
            continue

        # draw the bounding box and label on the image
        color = [int(c) for c in COLORS[idx]]
        cv2.rectangle(original_numpy, (xmin, ymin), (xmax, ymax), 
                    color, 2)
        y = ymin - 15 if ymin - 15 > 15 else ymin + 15
        label = "{}: {:.2f}%".format(LABEL_DICT[obj['class_id']],
            obj['score'] * 100)
        cv2.putText(original_numpy, label, (xmin, y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # return the final ima
    original_int = (original_numpy * 255).astype(np.uint8)
    return original_int

resultant_image = display_results(INPUT_IMAGE_URL)
Image.fromarray(resultant_image)