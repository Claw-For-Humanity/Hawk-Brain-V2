from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord
import time
import threading
import A1_test as test
import A1_tower as tower
import cv2
import realtime_detector



# connect to the robot
mc = MyCobot("/dev/ttyAMA0", 115200)

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
    while not killFlag.isSet():
        tower.watcher.limiter(mc)
        time.sleep(0.1)
    print('safely killed thread')

threading.Thread(target=limiter_looper, args=(mc)).start()

# We don't have camera yet, so test and find the correct angle/posture of the arm through testings
    # default positions = above bucket
    # bucket positions = above bucket, on bucket
    # garbage positions = above garbage
    # recycling positions = above recycling
    # unsure position = above unsure

# unsure

def set_position(position) -> bool:
    if position == 'default':
        mc.send_coords
        time.sleep(2)
        return True
    
    if position == 'pickUp':
        mc.send_coords
        time.sleep(2)
        print('done!')
        return True

    elif position == 'garbage':
        mc.send_coords
        time.sleep(2)
        print('done!')
        return True

    elif position == 'recycling':
        mc.send_coords
        time.sleep(2)
        print('done!')
        return True
    
    elif position == 'unsure':
        mc.send_coords
        time.sleep(2)
        print('done!')
        return True
    else:
        print('error... no matching position')
        return False

def pickUp() -> str:
    if set_position('default'):
        pass
    if set_position('pickUp'):
        pass
    # TODO: picture here

    # TODO: gripper here

    # return target bin

def perform():
    target = None

    target = pickUp()

    if target is not None:
        print(f'target is {target}')

    state = set_position(target)
    if state: 
        print("Success!")
        set_position('default')
    else:
        raise('stopped at A1 mission Control for no reason - line 103 fuck')