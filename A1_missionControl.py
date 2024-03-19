from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord
import time
import threading
import os
import A1_Camera as cam
import A1_test as test
import A1_tower as tower
import A2_google_drive_uploader as googley

class main:
    # connect to the robot
    mc = MyCobot("/dev/ttyAMA0", 115200)
    current_frame = None

    # testings
    test.comTEST.__init__(mc)
    test.ledTEST.__init__(mc)
    test.armTEST.resetPosition(mc)

        
    # Get the coordinates and posture of the current head
    coords = mc.get_coords()
    print(coords)

    # threading flags
    killFlag = threading.Event()


    # initiate limiter
    def limiter_looper(mc):
        while not main.killFlag.isSet():
            tower.watcher.limiter(mc)
            time.sleep(0.1)
        print('safely killed thread')

    # TODO: needs optimization here
    def camera_looper(camera):
        while not main.killFlag.isSet():
            ret, frame = cam.stream.camera()

            if not ret:
                print('error! camera reading failed!')
                break

            main.current_frame = frame

    

    # We don't have camera yet, so test and find the correct angle/posture of the arm through testings
        # default positions = above bucket
        # bucket positions = above bucket, on bucket
        # garbage positions = above garbage
        # recycling positions = above recycling
        # unsure position = above unsure

    def set_position(position) -> bool:
        if position == 'default':
            main.mc.send_coords
            time.sleep(2)
            return True
        
        if position == 'pickUp':
            main.mc.send_coords
            time.sleep(2)
            print('done!')
            return True

        elif position == 'garbage':
            main.mc.send_coords
            time.sleep(2)
            print('done!')
            return True

        elif position == 'recycling':
            main.mc.send_coords
            time.sleep(2)
            print('done!')
            return True
        
        elif position == 'unsure':
            main.mc.send_coords
            time.sleep(2)
            print('done!')
            return True
        
        else:
            print('error... no matching position')
            return False

    def pickUp():
        if main.set_position('default'):
            pass

        # TODO: machine learning active -> detects object
        # takes the picture and saves in CapturedImages

        # TODO: gripper grab here

        if main.set_position('pickUp'):
            pass
        
        # TODO: receive from google colab

        # throw it away to target bin
        received = '(what the fuck ever)'

        if main.set_position(f'{received}'):
            pass

        # TODO: gripper lose here
    
    def capture_save_upload():
        # capture and saved
        saved_name = cam.capture.frame(main.current_frame)

        # upload to drive
        googley.uploader.photo(saved_name, saved_name)

        print(f'uploaded to drive as {saved_name}!')


    def perform():
        target = None
        target = main.pickUp()

        if target is not None:
            print(f'target is {target}')

        state = main.set_position(target)
        
        if state:
            print("Success!")
            main.set_position('default')

        else:
            print('stopped at A1 mission Control for no reason - line 103 fuck')
            exit()
        

    def init():
        # enter these numbers
        camPort = 0
        base = os.path.abspath('./') # base folder

        # initialize camera
        camera = cam.capture.init(camPort, base)

        # initialize googley
        googley.initializer.__init__(base, "cfh-hawkeye-adb016b59179", "1NPciFwLIoW_ysdBg3ObeXRN4QvJBMXBl")


        threading.Thread(target=main.limiter_looper, args=(main.mc)).start()
        threading.Thread(target=main.camera_looper, args=(camera)).start()
