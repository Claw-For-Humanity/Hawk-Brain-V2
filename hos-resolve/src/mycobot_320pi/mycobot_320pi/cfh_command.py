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

        self.declare_parameter('tcp_ip', '169.254.221.247')  # Replace with Raspberry Pi's IP
        self.declare_parameter('tcp_port', 22411)
        self.declare_parameter('debugging', False)  # New parameter for debugging
        debugging = self.get_parameter('debugging').get_parameter_value().bool_value
        
        if not debugging:
            print('Attempting to connect to Raspberry Pi')
            tcp_ip = self.get_parameter('tcp_ip').get_parameter_value().string_value
            tcp_port = self.get_parameter('tcp_port').get_parameter_value().integer_value

            # ------------------ CONNECTION TO RASPBERRY PI ------------------ 
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((tcp_ip, tcp_port))
            print(f"Connected to Raspberry Pi @ {tcp_ip}:{tcp_port}")
        
        else:
            print("Debugging mode enabled. Skipping Raspberry Pi connection.")
            self.client_socket = None  # No socket connection in debugging mode

        time.sleep(1)
        
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)




    def listener_callback(self, msg):

        data_list = []

        if(len(msg.name) != 7):
            print(f'error has occured at line 49 ')

            print(f'current length is {len(msg.name)}')

            return
        

        joint_data = sorted(zip(msg.name, msg.position), key=lambda x: x[0])
        msg.name, msg.position = zip(*joint_data)
        msg.position = list(msg.position)


        # print(msg.name)
        print("===="*3)


        data_list = [round(math.degrees(pos), 2) for pos in msg.position]
        if all(value == 0 for value in data_list):
            print(msg.position)
            print("All values are 0")
            return

        data_list[2] = -data_list[2]
        print(data_list)

        if self.client_socket is None:  # If debugging, skip data sending
            
            print("Debugging mode: Processing joint states without sending to Raspberry Pi.")

            return

        else:
            pass

        print("dumping")
        data = pickle.dumps(data_list)

        # Send the length of the serialized data first (4 bytes)
        self.client_socket.sendall(len(data).to_bytes(4, byteorder="big"))
        
        # Send the actual serialized data
        self.client_socket.sendall(data)

        ack = self.client_socket.recv(1024).decode('utf-8')
        if ack == "ACK":
            print("Acknowledgment received from Raspberry Pi")

        
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