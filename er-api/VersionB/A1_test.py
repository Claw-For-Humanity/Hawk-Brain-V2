from pymycobot.mycobot import MyCobot

import time
class ledTEST:
    # The above needs to be written at the beginning of the code, which means importing the project package

    # MyCobot class initialization requires two parameters:
    #   The first is the serial port string:"/dev/ttyAMA0"
    #   The second is the baud rate: 115200
    #
    #    Example:
    #           mc = MyCobot("/dev/ttyAMA0", 115200)
    #
    # Initialize a MyCobot object
    # mc = MyCobot("/dev/ttyAMA0", 115200)

    def __init__(mc) -> bool:
        '''returns boolean of progress state'''
        print('beginning')
        mc.set_color(0, 0, 255)  # blue light on
        time.sleep(0.5)  # wait for 0.5 seconds				
        mc.set_color(255, 0, 0)  # red light on
        time.sleep(0.5)  # wait for 0.5 seconds
        mc.set_color(0, 255, 0)  # green light on
        time.sleep(0.5)  # wait for 0.5 seconds
        return True



class comTEST:
    def __init__(mc) -> bool:
        '''returns boolean of progress state'''
        if mc.is_controller_connected() != 1:
            print("Please connect the robot arm correctly for program writing")
            exit(0)
        else:
            pass
        return True


class armTEST:
    def __init__(mc) -> bool:
        '''joint test, returns boolean'''
        armTEST.resetPosition(mc)
        joint_stat = mc.is_all_servo_enable
        print(joint_stat)

        return joint_stat
    
    
    def resetPosition(mc):
        # Robotic arm recovery
        mc.send_angles([0, 0, 0, 0, 0, 0], 50)
        time.sleep(2.5)