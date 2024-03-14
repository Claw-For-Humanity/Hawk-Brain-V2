import os
import time
import numpy as np
import json
import csv
import random
from imgaug import augmenters as iaa
import cv2
from dataset import Taco
import model as modellib
from model import MaskRCNN
from config import Config
import visualize
import utils
import matplotlib
import matplotlib.pyplot as plt
import tkinter as tk
import threading
from PIL import Image
matplotlib.use('TkAgg')  # Use an appropriate backend
from pycocotools.cocoeval import COCOeval
from pycocotools import mask as maskUtils


class detector:
    # True Root directory of the models
    TRUE_ROOT_DIR = os.path.abspath("./")

    # Detector Root directory of the models
    DETECTOR_ROOT_DIR = os.path.abspath("./detector")
    print(f'\n\nroot directory is {DETECTOR_ROOT_DIR}\n')

    # Directory to save logs and model checkpoints
    DEFAULT_LOGS_DIR = os.path.join(DETECTOR_ROOT_DIR, "models", "maskModel")
    print(f'\ndefault log directory is {DEFAULT_LOGS_DIR}\n')

    # round integer
    ROUND_INT = 0
    print(f'\nround integer is {ROUND_INT}\n')

    # path to class_map 
        # *** change csv as you want***
    CLASSMAP_DIR = os.path.join(DETECTOR_ROOT_DIR, "taco_config", "map_10.csv")
    print(f'\nclass map directory is {CLASSMAP_DIR}\n')

    # path to dataset
    DATASET_DIR = os.path.join(TRUE_ROOT_DIR, "data")
    print(f'\nDATASET_DIR is {DATASET_DIR}\n')

    # path to model_path
    MODEL_PATH = os.path.join(DEFAULT_LOGS_DIR, "mask_rcnn_taco_0100.h5")
    print(f'\nMODEL_PATH is {MODEL_PATH}\n')

    # path to test images
    TESTIMG_PATH = os.path.join(TRUE_ROOT_DIR, "testImages")
    print(f'\nTESTIMG_PATH is {TESTIMG_PATH}\n\n')

    # thread event
    flag_thread = threading.Event()


    # realtime
    def inference_realtime(self, model, dataset):
        print('\nentered inference realtime\n')
        camera = cv2.VideoCapture(0)  # Open the default camera (change the index if necessary)
        print(f'flag status is {self.flag_thread.isSet()}')

        while not self.flag_thread.is_set():
            ret, frame = camera.read()  # Capture frame from the camera
            if not ret:
                print("Error: Failed to capture frame")
                break

            r = model.detect([frame], verbose=0)[0]

            # Perform instance fusion if necessary
            if r['class_ids'].shape[0] > 0:
                r_fused = utils.fuse_instances(r)
            else:
                r_fused = r

            returned_Major_Bucket = visualize.return_inference(frame, r_fused['rois'], r_fused['masks'], r_fused['class_ids'],
                                        dataset.class_names, r_fused['scores'], title="Predictions fused")
            
            # receive masked frame from bucket and remove it
            masked_frame = returned_Major_Bucket['masked_img']
            del returned_Major_Bucket ['masked_img']

            print(returned_Major_Bucket)

            # Show the frame with predictions
            cv2.imshow("Real-Time Detection", masked_frame)

            # Break the loop if 'q' is pressed or if maximum frames reached
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # for picture detection (work on this for google)
    def inference_picture(image_dir, model, dataset):
        image = Image.open(image_dir)

        if image.format == "PNG":
            print('\nimage is png\n')
            image = np.array(image)
            if image.shape[2] == 4:
                image = image[:, :, :3]

        else:
            pass

        r = model.detect([image], verbose=0)[0]
        if r['class_ids'].shape[0] > 0:
            r_fused = utils.fuse_instances(r)
        else:
            r_fused = r

        returned_Major_Bucket = visualize.return_inference(image, r_fused['rois'], r_fused['masks'], r_fused['class_ids'],
                                            dataset.class_names, r_fused['scores'], title="Predictions fused")
        
        # receive masked frame from bucket and remove it
        masked_frame = returned_Major_Bucket['masked_img']
        del returned_Major_Bucket ['masked_img']

        print(f'dictionary of detected objects are \n{returned_Major_Bucket}')

        # TODO: for i in loop and get center and stuff


        # visualization using matplotlib 
        # fig, (ax) = plt.subplots(1,1, figsize=(16, 16))
        # visualize.display_instances(image, r_fused['rois'], r_fused['masks'], r_fused['class_ids'],
        #                                 dataset.class_names, r_fused['scores'], title="Predictions fused", ax=ax)
        # plt.show()

        # visualization using cv2
        # while True:
        #     cv2.imshow("Real-Time Detection", masked_frame)
        #     if cv2.waitKey(1) & 0xFF == ord('q'):
        #         break

        return returned_Major_Bucket
    
    def bucket_handler (bucket):
        # TODO: add bucket handler here (for i)
        pass



    # initialize
    def init(self, mode):
        # TODO: change this later (repeting on the top)
        round_int = self.ROUND_INT
        classMap_dir = self.CLASSMAP_DIR
        dataset_dir = self.DATASET_DIR
        model_path = self.MODEL_PATH


        # Read map of target classes
        class_map = {}
        map_to_one_class = {}

        with open(classMap_dir) as csvfile:
            reader = csv.reader(csvfile)
            # fill in class_map and 
            class_map = {row[0]: row[1] for row in reader}
            map_to_one_class = {c: 'Litter' for c in class_map}

        # Test dataset
        dataset_test = Taco()
        taco = dataset_test.load_taco(dataset_dir, round_int, "test", class_map=class_map, return_taco=True)
        dataset_test.prepare()
        nr_classes = dataset_test.num_classes

        # configuration
        class TacoTestConfig(Config):
            NAME = "taco"
            GPU_COUNT = 1
            IMAGES_PER_GPU = 1
            DETECTION_MIN_CONFIDENCE = 10 #if args.command == "evaluate" else 10
            NUM_CLASSES = nr_classes
            USE_OBJECT_ZOOM = False
        config = TacoTestConfig()
        config.display()

        # load inference model
        model = MaskRCNN(mode="inference", config=config, model_dir=self.DEFAULT_LOGS_DIR)

        # load weights
        model.load_weights(model_path, model_path, by_name=True)

        # run inference
        if mode == "realtime":
            self.inference_realtime(model,dataset_test)
        
        
        elif mode == "picture":
            picture_directory = os.path.join(self.TESTIMG_PATH, "2.png")
            bucket = self.inference_picture(picture_directory,model,dataset_test)


        else:
            raise('check mode')