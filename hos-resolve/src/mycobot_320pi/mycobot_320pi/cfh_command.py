
import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import pickle


class MyCobotListener(Node):

    def __init__(self):
        super().__init__('listener')

        self.declare_parameter('tcp_ip', '169.254.221.247')  # Raspberry Pi's IP
        self.declare_parameter('tcp_port', 22411)
        self.declare_parameter('debugging', False)

        tcp_ip = self.get_parameter('tcp_ip').get_parameter_value().string_value
        tcp_port = self.get_parameter('tcp_port').get_parameter_value().integer_value
        debugging = self.get_parameter('debugging').get_parameter_value().bool_value
        if debugging:
            self.client_socket = None
            print('[LOG]: debugging enabled.\nSkipping connection to rpi')

        else:
            print('Attempting to connect to Raspberry Pi')
            
            # ------------------ CONNECTION TO RASPBERRY PI ------------------ 
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((tcp_ip, tcp_port))
            print(f"Connected to Raspberry Pi @ {tcp_ip}:{tcp_port}")
            
        time.sleep(1)
        
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)



    def listener_callback(self, msg):
        data_list = []

        if(len(msg.name) != 6 and len(msg.name) != 7):
            print('error! length is not 6 or 7\nactual length: {}'.format(len(msg.name)))
            return
        
        
        joint_data = sorted(zip(msg.name, msg.position), key=lambda x: x[0])
        msg.name, msg.position = zip(*joint_data)
        msg.position = list(msg.position)


        # print(msg.name)
        print("===="*3)

        data_list = [round(math.degrees(pos), 2) for pos in msg.position]
        if all(value == 0 for value in data_list):
            print("All values are 0")
            return

        data_list[2] = -data_list[2]
        print(data_list)


        if self.client_socket == None:
            
            return


        ############################### com part ###############################

        print("dumping")
        data = pickle.dumps(data_list)

        # Send the length of the serialized data first (4 bytes)
        self.client_socket.sendall(len(data).to_bytes(4, byteorder="big"))
        
        # Send the actual serialized data
        self.client_socket.sendall(data)

        ack = self.client_socket.recv(1024).decode('utf-8')
        if ack == "ACK":
            print("Acknowledgment received from Raspberry Pi")


        # example data is [-7.6, -150.85, 30.91, -71.54, -11.48, 105.88, 0.0]
        # the following is the coordination of the data
        # [gripper, j2-1, j3-2, j4-3, j5-4, j6-5, j6o-j6]

        print(data_list)


        
        time.sleep(1)

    def __del__(self):
        
        self.client_socket.close()
        print('laptop server closing')
        exit()



def main(args=None):
    rclpy.init(args=args)

    mycobot_listener = MyCobotListener()

    rclpy.spin(mycobot_listener)

    mycobot_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()