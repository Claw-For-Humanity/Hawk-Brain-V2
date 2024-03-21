# this will be the object detection software running on pi continuously.
import cv2


class camera:
    # edit this
    camPort = 0
    
    # bucket infos.
    cam = None
    ret = False

    def __init__():
        camera.cam = cv2.VideoCapture(camera.camPort)
        