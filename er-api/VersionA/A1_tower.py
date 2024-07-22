class watcher:
    j1_max = None
    j2_max = None
    j3_max = None
    j4_max = None
    j5_max = None
    j6_max = None

    j1_min = None
    j2_min = None
    j3_min = None
    j4_min = None
    j5_min = None
    j6_min = None
    
    def __init__(mc) -> None:
        print("\ngetting joint max/min angles")
        # max angles
        watcher.j1_max = mc.get_joint_max_angle(1)
        watcher.j2_max = mc.get_joint_max_angle(2)
        watcher.j3_max = mc.get_joint_max_angle(3)
        watcher.j4_max = mc.get_joint_max_angle(4)
        watcher.j5_max = mc.get_joint_max_angle(5)
        watcher.j6_max = mc.get_joint_max_angle(6)

        # min angles
        watcher.j1_min = mc.get_joint_min_angle(1)
        watcher.j2_min = mc.get_joint_min_angle(2)
        watcher.j3_min = mc.get_joint_min_angle(3)
        watcher.j4_min = mc.get_joint_min_angle(4)
        watcher.j5_min = mc.get_joint_min_angle(5)
        watcher.j6_min = mc.get_joint_min_angle(6)
        print("obtained joint max/min angles\n")


        print('\nentering servo test')
        servoErrors = mc.get_servo_status()
        i = 0
        while (i<6):
            if servoErrors[i] == 1:
                # [voltage, sensor, temperature, current, angle, overload], a value of 0 means no error, a value of 1 indicates an error
                print(f"servo error at {i}")
                print("[voltage, sensor, temperature, current, angle, overload]")
            else:
                pass
            i += 1
        print('servo test completed\n')

        print('\nsystem error check')
        errors = mc.get_error_information()
        print(f'system error is {errors}')
        print('system check completed\n')        
                                                
    def limiter(mc, maxSafeSpd, safeAngle):
        # MAX SPEED
        if mc.get_servo_speeds() > maxSafeSpd:
            mc.stop()
            raise("tower intercepted due to max speed exceed problem")
        
        # MAX/MIN ANGLE
        angles = mc.get_angle()
        # joint 1
        if angles[0] > (watcher.j1_max - safeAngle) or angles[0] < (watcher.j1_min + safeAngle):
            mc.stop()
            raise("tower intercepted due to max/min angle exceed problem on joint 1")
        
        # joint 2
        if angles[1] > (watcher.j2_max - safeAngle) or angles[1] < (watcher.j2_min + safeAngle):
            mc.stop()
            raise("tower intercepted due to max/min angle exceed problem on joint 2")        
        
        # joint 3
        if angles[2] > (watcher.j3_max - safeAngle) or angles[2] < (watcher.j3_min + safeAngle):
            mc.stop()
            raise("tower intercepted due to max/min angle exceed problem on joint 3")

        # joint 4
        if angles[3] > (watcher.j4_max - safeAngle) or angles[4] < (watcher.j4_min + safeAngle):
            mc.stop()
            raise("tower intercepted due to max/min angle exceed problem on joint 4")

        # joint 5
        if angles[4] > (watcher.j5_max - safeAngle) or angles[4] < (watcher.j5_min + safeAngle):
            mc.stop()
            raise("tower intercepted due to max/min angle exceed problem on joint 5")
        
        # joint 6
        if angles[5] > (watcher.j6_max - safeAngle) or angles[5] < (watcher.j6_min + safeAngle):
            mc.stop()
            raise("tower intercepted due to max/min angle exceed problem on joint 6")
                            
        