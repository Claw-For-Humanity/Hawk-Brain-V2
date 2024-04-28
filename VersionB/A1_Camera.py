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
            print('camera initialization failed. Check camera port')
            exit()

        if not os.path.exists(os.path.join(base, 'capturedImages')):
            print('creating capturedImages dir')
            os.mkdir(os.path.join(base, 'capturedImages'))
        
        capture.SAVING_DIR = os.path.join(base, 'capturedImages')
        
        return camera
    
    def frame(frame) -> str:
        '''saves picture in png format to default directory'''
        characters = string.ascii_letters + string.digits
        random_string = ''.join(random.choice(characters) for i in range(8))
        pic_name = f'{random_string}_{time.time}.png'

        cv2.imwrite(os.path.join(capture.SAVING_DIR, pic_name), frame)
        return pic_name
    



class stream:
    def camera(camera):
        '''returns frame'''
        ret, frame = camera.read(0)

        if not ret:
            print('camera reading failed. re-initialize cam thread')

        return ret, frame
    


camera = capture.init()
