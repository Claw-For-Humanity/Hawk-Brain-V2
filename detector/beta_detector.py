"""
/Users/changbeankang/anaconda3/envs/testTACO/bin/python /Users/changbeankang/Claw_For_Humanity/TACO-master-backup/detector/detector.py test --dataset=/Users/changbeankang/Claw_For_Humanity/TACO-master-backup/data --model=/Users/changbeankang/Claw_For_Humanity/TACO-master-backup/detector/models/logs/mask_rcnn_taco_0100.h5 --round 0 --class_map=/Users/changbeankang/Claw_For_Humanity/TACO-master-backup/detector/taco_config/map_10.csv --use_aug


    # First make sure you have split the dataset into train/val/test set. e.g. You should have annotations_0_train.json
    # in your dataset dir.
    # Otherwise, You can do this by calling
    python3 split_dataset.py --dataset_dir ../data

    # Train a new model starting from pre-trained COCO weights on train set split #0
    python3 -W ignore detector.py train --model=coco --dataset=../data --class_map=./taco_config/map_10.csv --round 0

    # Continue training a model that you had trained earlier
    python3 -W ignore detector.py train  --dataset=../data --model=<model_name> --class_map=./taco_config/map_10.csv --round 0

    # Continue training the last model you trained with image augmentation
    python3 detector.py train --dataset=../data --model=last --round 0 --class_map=./taco_config/map_10.csv --use_aug

    # Test model and visualize predictions image by image
    python3 detector.py test --dataset=../data --model=<model_name> --round 0 --class_map=./taco_config/map_10.csv

    # Run COCO evaluation on a trained model
    python3 detector.py evaluate --dataset=../data --model=<model_name> --round 0 --class_map=./taco_config/map_10.csv

    # Check Tensorboard
    tensorboard --logdir ./models/logs

"""

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

matplotlib.use('TkAgg')  # Use an appropriate backend

from pycocotools.cocoeval import COCOeval
from pycocotools import mask as maskUtils

# Root directory of the models
ROOT_DIR = os.path.abspath("./models")

# Path to trained weights file
COCO_MODEL_PATH = os.path.join(ROOT_DIR, "mask_rcnn_coco.h5")

# Directory to save logs and model checkpoints
DEFAULT_LOGS_DIR = os.path.join(ROOT_DIR, "logs")

############################################################
#  Testing functions
############################################################

def test_Dataset(model, dataset, nr_images):

    for i in range(nr_images):

        image_id = dataset.image_ids[i] if nr_images == len(dataset.image_ids) else random.choice(dataset.image_ids)

        image, image_meta, gt_class_id, gt_bbox, gt_mask = \
            modellib.load_image_gt(dataset, config, image_id, use_mini_mask=False)
        info = dataset.image_info[image_id]

        r = model.detect([image], verbose=0)[0]

        print(r['class_ids'].shape)
        if r['class_ids'].shape[0]>0:
            r_fused = utils.fuse_instances(r)
        else:
            r_fused = r

        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(16, 16))

        # Display predictions
        visualize.display_instances(image, r['rois'], r['masks'], r['class_ids'],
                                    dataset.class_names, r['scores'], title="Predictions", ax=ax1)

        visualize.display_instances(image, r_fused['rois'], r_fused['masks'], r_fused['class_ids'],
                                     dataset.class_names, r_fused['scores'], title="Predictions fused", ax=ax2)

        # # Display ground truth
        visualize.display_instances(image, gt_bbox, gt_mask, gt_class_id, dataset.class_names, title="GT", ax=ax3)

        # Voilà
        plt.show()


def test_realtime(model, dataset):
    camera = cv2.VideoCapture(0)  # Open the default camera (change the index if necessary)
    obj_detection_window = tk.Tk()
    obj_detection_window.title ("Object Detection Engine")
    obj_detection_window.geometry("2000x1000")
    obj_detection_window.resizable(True,True)
    detection_canvas = tk.Canvas(obj_detection_window, width=2000, height=1000)
    detection_canvas.pack()
    canvas_data = detection_canvas.create_image(0,50,anchor = tk.NW)
    

    while True:
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

        # Display predictions on the frame
        # visualize.display_instances(frame, r['rois'], r['masks'], r['class_ids'],
        #                             dataset.class_names, r['scores'], title="Predictions")

        masked = visualize.return_inference(frame, r_fused['rois'], r_fused['masks'], r_fused['class_ids'],
                                     dataset.class_names, r_fused['scores'], title="Predictions fused")
        
      
        

        # Show the frame with predictions
        cv2.imshow("Real-Time Detection", masked)

        # Break the loop if 'q' is pressed or if maximum frames reached
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close OpenCV windows
    camera.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import argparse

    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Run Mask R-CNN on TACO.')
    parser.add_argument("command", metavar="<command>",help="Opt: 'train', 'evaluate', 'test'")
    parser.add_argument('--model', required=True, metavar="/path/weights.h5", help="Path to weights .h5 file or 'coco'")
    parser.add_argument('--dataset', required=True, metavar="/path/dir", help='Directory of the dataset')
    parser.add_argument('--round', required=True, type=int, help='Split number')
    parser.add_argument('--lrate', required=False, default=0.001, type=float, help='learning rate')
    parser.add_argument('--use_aug', dest='aug', action='store_true')
    parser.set_defaults(aug=False)
    parser.add_argument('--use_transplants', required=False, default=None, help='Path to transplanted dataset')
    parser.add_argument('--class_map', required=True, metavar="/path/file.csv", help=' Target classes')

    args = parser.parse_args()
    print("Command: ", args.command)
    print("Model: ", args.model)
    print("Dataset: ", args.dataset)
    print("Logs: ", DEFAULT_LOGS_DIR)

    # Read map of target classes
    class_map = {}
    map_to_one_class = {}
    with open(args.class_map) as csvfile:
        reader = csv.reader(csvfile)
        class_map = {row[0]: row[1] for row in reader}
        map_to_one_class = {c: 'Litter' for c in class_map}

    if args.command == "test":
        # Test dataset
        dataset_test = Taco()
        taco = dataset_test.load_taco(args.dataset, args.round, "test", class_map=class_map, return_taco=True)
        dataset_test.prepare()
        nr_classes = dataset_test.num_classes

    # Configurations
    if True:
        class TacoTestConfig(Config):
            NAME = "taco"
            GPU_COUNT = 1
            IMAGES_PER_GPU = 1
            DETECTION_MIN_CONFIDENCE = 0 if args.command == "evaluate" else 10
            NUM_CLASSES = nr_classes
            USE_OBJECT_ZOOM = False
        config = TacoTestConfig()

    config.display()

    # Create inference model
    if args.command == "test":
        model = MaskRCNN(mode="inference", config=config, model_dir=DEFAULT_LOGS_DIR)


    # Select weights file to load
    if True:
        # _, model_path = model.get_last_checkpoint(args.model)
        model_path = '/Users/changbeankang/Claw_For_Humanity/TACO-master-backup/detector/models/logs/mask_rcnn_taco_0100.h5'

    # Load weights
    if True:
        model.load_weights(model_path, model_path, by_name=True)

    # evaluate
    if args.command == "test":
        test_Dataset(model, dataset_test, len(dataset_test.image_ids))
        # test_realtime(model,dataset_test)
        
    elif args.command == "realtime":
        test_realtime(model,dataset_test)
    else:
        print("'{}' is not recognized. "
              "Use 'train' or 'evaluate'".format(args.command))
