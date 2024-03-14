import cv2
import time
import random
import string
import os

class capture:

    def init(camPort):
        camera = cv2.VideoCapture(camPort)
        if not camera.isOpened():
            raise('camera initialization failed. Check camera port')
        
        return camera

    def frame():
        pass

class stream:
    SAVING_DIR = os.path.
    def run(camera, flag):
        '''returns frame'''
        while not flag.is_set():
            ret, frame = camera.read(0)

            if not ret:
                print('camera reading failed. re-initialize cam thread')
                break

            return frame
    
    def save(frame):
       
        characters = string.ascii_letters + string.digits
        random_string = ''.join(random.choice(characters) for i in range(8))
        randint = random.randint()
        name_int = int(time.time)
        cv2.imwrite(f'{random_string}_{time.time}.png', frame)


