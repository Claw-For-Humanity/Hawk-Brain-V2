from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord
import os
import time
import A1_plugins_serverCom as Server
import A1_Camera as Camera
import A1_test as Test
import threading

class bucket:

    camera = None
    mc = None
    current_frame = None

    # Return Value: list: a list containing coordinates and postures.
    # Six axes: The length is 6, and they are [x, y, z, rx, ry, rz] in order.
    # Four axes: The length is 6, and they are [x, y, z, rx] in order.
    
    ready = [0,0,0,0,0,0] # TODO: find the ready position coordinate

    ready_bin_1 = [1,2,3,1,2,3] # TODO: find the ready position coordinate
    ready_bin_2 = [1,2,3,1,2,3] # TODO: find the ready position coordinate
    ready_bin_3 = [1,2,3,1,2,3] # TODO: find the ready position coordinate



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
        bucket.camera = Camera.capture.init(initialize.camPort, initialize.base)

        # testings before full operation
        Test.comTEST.__init__(bucket.mc)
        Test.ledTEST.__init__(bucket.mc)
        Test.armTEST.__init__(bucket.mc)


        print('initialization complete')


class main:
    # Get the coordinates and posture of the current head
    coords = bucket.mc.get_coords()
    print(f'current coords is {coords}')

    # Six axes: The length of the coordinate value of [x, y, z, rx, ry, rz] is 6.
    # Four axes: The length of the coordinate value of [x,y,z,rx] is 4.
    # speed: means the movement speed of the robot arm, ranging from 0 to 100.
    # mode: (int ): The value is limited to 0 and 1.
    # 0 means that the movement path of the robot arm head is non-linear, i.e. the movement route is randomly planned just to make sure that the head moves to a specified point with a specified posture.
    # 1 means that the movement path of the robot arm head is linear, i.e. the movement route is intelligently planned just to make sure that the head moves to a specified point with a specified posture in a linear manner.
    bucket.mc.send_coords(bucket.ready, 10, 1) 
    
    ret, frame = Camera.stream.camera(bucket.camera)

    if ret:
        token = Camera.capture.frame(frame)
        pic_name = f'{token}.png'
        print(f'saved as {token}.png')

    else:
        print('image saving failed')

    # upload image
    Server.uploader.photo(token, pic_name)
    
    # edit token
    Server.editor.mainInitiator(token)


    # check for output and if output is none, keep waiting for it
    if Server.editor.accessor(token) == False:
      print('output is none')
      time.sleep(1)
      output = Server.editor.accessor(token)
    
    else:
        pass
    
    output #TODO: need to estimate the 3d pose from here




    





