import cv2
import time
import random
import string
import os

class capture:
    
    # saving directory
    SAVING_DIR = None
    
    def init(camPort, base):
        camera = cv2.VideoCapture(camPort)
        if not camera.isOpened():
            raise('camera initialization failed. Check camera port')

        if not os.path.exists(os.path.join(base, 'capturedImages')):
            os.mkdir(os.path.join(base, 'capturedImages'))
        
        capture.SAVING_DIR = os.path.join(base, 'capturedImages')
        
        return camera
    
    def frame(frame):
        '''saves picture in png format to default directory'''
        characters = string.ascii_letters + string.digits
        random_string = ''.join(random.choice(characters) for i in range(8))
        pic_name = f'{random_string}_{time.time}.png'

        cv2.imwrite(os.path.join(capture.SAVING_DIR, pic_name), frame)
    



class stream:
    def run(camera, flag):
        '''returns frame'''
        while not flag.is_set():
            ret, frame = camera.read(0)

            if not ret:
                print('camera reading failed. re-initialize cam thread')
                break

            return frame
    


