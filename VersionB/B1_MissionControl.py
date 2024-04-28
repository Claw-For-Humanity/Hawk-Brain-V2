from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord
import os
import time
import A1_plugins_serverCom as Server
import A1_Camera as Camera
import A1_test as Test
import threading


class initialize:
    camPort = 0
    base = os.path.abspath('./')
    debuggingFlag = False

    def init_Arm():
        print('initializing arm')
        bucket.mc = MyCobot("/dev/ttyAMA0", 115200)



    # initialize camera and server com
    def __init__():
        # initialize servercom
        Server.initializer.__init__(initialize.base,'cfh-hawkeye-adb016b59179','1OlbAmA_yr76eaoT217e3nyAdT9X7ZkBh')
        print(f'initialize session time is {Server.tools.current_time()}')

        # initialize camera
        Camera.capture.init(initialize.camPort, initialize.base)

        # testings before full operation
        Test.comTEST.__init__(bucket.mc)
        Test.ledTEST.__init__(bucket.mc)
        Test.armTEST.__init__(bucket.mc)


        print('initialization complete')

class bucket:
    camera = None
    mc = None
    current_frame = None

class main:
    # Get the coordinates and posture of the current head
    coords = bucket.mc.get_coords()
    print(f'current coords is {coords}')

class threads:
    killFlag = threading.Event()
    def camThread(camera):
        while not threads.killFlag.isSet():
            ret, frame = Camera.stream.camera()

            if not ret:
                print('error loading camera')
                break

            bucket.current_frame = frame
    
    # TODO: test out A1 version of tower and update it
    # make this work on only in debugging modde
    def towerThread():
        while not threads.killFlag.isSet() and initialize.debuggingFlag:
            print('tower')
    