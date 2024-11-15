import cv2 as cv 
import matplotlib.pyplot as plt
import time
import numpy as np
import time

from tflite_support.task import core 
from tflite_support.task import processor  
from tflite_support.task import vision 
from utils import *

# Code modified from the official TFLite documentation
_MARGIN = 10  # pixels
_ROW_SIZE = 10  # pixels
_FONT_SIZE = 1.3
_FONT_THICKNESS = 3
_TEXT_COLOR = (0, 0, 0)  # black
_TEXT_COLOR_OPPOSITE = (255, 255, 255)  # white

# Variables to calculate FPS
counter, fps = 0, 0
start_time = time.time()

def calcVision(detector, capture = None, sendImage = False):
    """
    Main function for the vision system it uses a model and a detector to detect the object, it uses the object to calculate the angle and distance to the object and then sends the data to the robot


    --------------------------------------------------------------
    Input:
        detector (tflite model)
        capture (cv2.VideoCapture)
        sendImage (bool)

    --------------------------------------------------------------
    Output: 
        rgbImage (np.ndarray)
        bestBox (dict)
        imgSize (tuple)
    
    """
    
    global counter, fps, start_time

    plt.ion()

    if capture is None:
        result, img = cv.imread('ImageTest.jpg')
    else:
        img = getWebcamImage(capture, numberOfFrames = 1)
        
        # flip image
        img = cv.flip(img, 0) # Flip vertically

    # Visualization parameters
    row_size = 20  # pixels
    left_margin = 24  # pixels
    text_color = _TEXT_COLOR  # red
    font_size = 1
    font_thickness = 1
    fps_avg_frame_count = 10
    
    counter += 1
    #img = cv.flip(img, 0) # Flip vertically
    img = cv.flip(img, 1)

    # Convert the image from BGR to RGB as required by the TFLite model.
    rgb_image = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    #     plt.subplot(2,2,2)
    #     plt.imshow(rgb_image)

    # Create a TensorImage object from the RGB image.
    input_tensor = vision.TensorImage.create_from_array(rgb_image)

    # Run object detection estimation using the model.
    detection_result = detector.detect(input_tensor)
    #     plt.subplot(2,2,2)
    #     plt.imshow(rgb_image)
    
    # Draw keypoints and edges on input image
    bestBox = getBestBox(detection_result)
    for box in detection_result.detections:
        #(box, bestBox, box == bestBox)
        if box == bestBox:
            rgb_image = showBox(rgb_image, box, color = (0, 255, 0))
        else:
            rgb_image = showBox(rgb_image, box, color = (255,255,0))
    
    #     plt.subplot(2,2,3)
    #     plt.imshow(rgb_image)

    # Calculate the FPS
    if counter % fps_avg_frame_count == 0:
        end_time = time.time()
        fps = fps_avg_frame_count / (end_time - start_time)
        start_time = time.time()

    # Show the FPS
    fps_text = 'FPS = {:.1f}'.format(fps)
    text_location = (left_margin, row_size)
    cv.putText(rgb_image, fps_text, text_location, cv.FONT_HERSHEY_PLAIN,
                font_size, text_color, font_thickness)

    # Stop the program if the ESC key is pressed.
    if cv.waitKey(1) == 27:

        if (capture is not None):
            capture.release()
        cv.destroyAllWindows()

    
    #     plt.subplot(2,2,4)
    #     plt.imshow(rgb_image)
    #     plt.show() # block=False
    #     plt.pause(.1)

    imgSize = rgb_image.shape

    if not sendImage:
        rgb_image = None
        
    return rgb_image, bestBox, imgSize
    

def getBestBox(detectionResult):
    """
    Calculates the best box from the detection result

    --------------------------------------------------------------
    Input:
        detectionResult (tflite model)

    --------------------------------------------------------------
    Output: 
        best (dict)
    
    """
    
    if len(detectionResult.detections) == 0:
        return

    best = detectionResult.detections[0]
    bestScore = best.bounding_box.width * best.bounding_box.height

    for detection in detectionResult.detections:

        # Draw bounding_box
        bbox = detection.bounding_box
        bwidth, bheight = bbox.width, bbox.height
        start_point = bbox.origin_x, bbox.origin_y
        end_point = bbox.origin_x + bwidth, bbox.origin_y + bheight
        currentScore = bwidth * bheight

        if currentScore > bestScore:
            best = detection
            bestScore = currentScore

    return best 

def showBox(image, result, color = (255, 0, 0)):
    """
    show a box, label and score on the image

    --------------------------------------------------------------
    Input:
        image (np.ndarray)
        result (tflite model)
        color (tuple)

    --------------------------------------------------------------
    Output: 
        image (np.ndarray)
    
    """

    bbox = result.bounding_box
    start_point = bbox.origin_x, bbox.origin_y
    end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
    cv.rectangle(image, start_point, end_point, color, 3) 

    # Draw label and score
    category = result.categories[0]
    category_name = category.category_name
    if category_name == "DuploAssembly":
        category_name = "Duplo"
    probability = round(category.score, 2)
    result_text = category_name + ' (' + str(probability) + ')'

    #print(result_text)
    text_location = (_MARGIN + bbox.origin_x - 50,
                     _MARGIN + _ROW_SIZE + bbox.origin_y)
    cv.putText(image, result_text, text_location, cv.FONT_HERSHEY_PLAIN,
                _FONT_SIZE, _TEXT_COLOR, _FONT_THICKNESS + 2)
    
    cv.putText(image, result_text, text_location, cv.FONT_HERSHEY_PLAIN,
                _FONT_SIZE, _TEXT_COLOR_OPPOSITE, _FONT_THICKNESS)

    return image


def initModel(model = "BigBossTraining.tflite",enable_edgetpu: bool = False, threshold:float = 0.7,num_threads: int = 4):
  """
    Initializes the TFLite object detection model.

    --------------------------------------------------------------
    Input:
        model (String): Name of the TFLite object detection model
        enable_edgetpu (bool): True/False whether the model is a EdgeTPU model
        threshold (float): The threshold for the model
        num_threads (int): The number of CPU threads to run the model

    --------------------------------------------------------------
    Output: 
        detector (tflite model)
    
    """

  # Initialize the object detection model
  base_options = core.BaseOptions(
      file_name=model, use_coral=enable_edgetpu, num_threads=num_threads)
  detection_options = processor.DetectionOptions(
      max_results=3, score_threshold=threshold)
  options = vision.ObjectDetectorOptions(
      base_options=base_options, detection_options=detection_options)
  detector = vision.ObjectDetector.create_from_options(options)

  return detector

def map(x, in_min, in_max, out_min, out_max):
    """
    Map a value from one range to another range in a linear fashion

    --------------------------------------------------------------
    Input:
        x (float) : input value
        in_min (float)
        in_max (float)
        out_min (float)
        out_max (float)

    --------------------------------------------------------------
    Output: 
        (float) : mapped value
    
    """

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if __name__ == '__main__':


    webcam = initWebcam()
    model = initModel("BigBossTraining.tflite")

    #get the image from the webcam
    while True:
        img = getWebcamImage(webcam)

        # cv.imshow('image', img)
        # img = None

        if img is None:
            img = cv.imread('ImageTest.jpg')

        # imgAnalysee,img2 = calcVision(img)
        img2, box,resolution = calcVision(model,webcam,sendImage = True) 
        # imgAnalysee = img

        #show the image but in the right color

        #         img = cv.cvtColor(img, cv.COLOR_RGB2BGR)
        #         plt.cla()
        #         axs[0].imshow(imgAnalysee)
        #         axs[1].imshow(img2)

        # the previous way is too slow, so find a way to show the two images
        # at the same time
        
        #if it's a gray scale image

        #         if len(img2.shape) == 2:
        #             img2 = cv.cvtColor(img2, cv.COLOR_GRAY2BGR)
        # 
        #         imgFinal = np.concatenate((imgAnalysee, img2), axis=1)

        cv.imshow('Result', img2) 

        if cv.waitKey(1) & 0xFF == ord('q'):
            break
  