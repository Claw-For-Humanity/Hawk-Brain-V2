import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import pickle

from pymycobot.mycobot import MyCobot

class MyCobotListener(Node):

    def __init__(self):
        super().__init__('listener')
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.declare_parameter('tcp_ip', '0.0.0.0')
        self.declare_parameter('tcp_port', 22411)
        
        tcp_ip = self.get_parameter('tcp_ip').get_parameter_value().string_value
        tcp_port = self.get_parameter('tcp_port').get_parameter_value().integer_value
    



        self.get_logger().info(f'port: {port}, baud: {baud}')
        self.mycobot = MyCobot(port, str(baud))
        self.mycobot.set_free_mode(1)

        print(
            'connection established with the arm'
        )


        time.sleep(1)

        print('Attempting to connect on laptop')

    # ------------------ LAPTOP CONNECTION ------------------ 
        
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((tcp_ip, tcp_port))
        self.server_socket.listen(1)
        print(f"Server listening on {tcp_ip}:{tcp_port}")

        self.conn, self.addr = self.server_socket.accept()
        print(f"Connection established with laptop @ {self.addr}")

        
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)



    def listener_callback(self, msg):
        
        try:
            # Read the length of the incoming message (4 bytes)
            data_length_bytes = self.conn.recv(4)
            if not data_length_bytes:
                raise ValueError("Failed to read message length")

            data_length = int.from_bytes(data_length_bytes, byteorder="big")
            print(f"Expecting {data_length} bytes of data")

            # Receive the exact amount of data
            received_data = b""
            while len(received_data) < data_length:
                chunk = self.conn.recv(1024)
                if not chunk:
                    raise ValueError("Incomplete message received")
                received_data += chunk

            # Deserialize the received data
            final_data = pickle.loads(received_data)
            # print("Received list:", final_data)
            
            self.mycobot.set_color(3, 252, 24)
            time.sleep(0.1)
            self.mycobot.send_angles(final_data, 80)
            time.sleep(0.1)
            self.mycobot.set_color(40, 3, 252)  # Reset LED color
            time.sleep(0.1)
            print('sending ack')
            self.conn.sendall("ACK".encode('utf-8'))

        except Exception as e:
            self.get_logger().error(f"Error is {e}")

        finally:
            print("Ready for next message.")

        time.sleep(0.1)


    def __del__(self):
        self.conn.close()
        self.server_socket.close()
        print("Closed server socket.")
        exit()





def main(args=None):
    rclpy.init(args=args)

    mycobot_listener = MyCobotListener()

    rclpy.spin(mycobot_listener)

    mycobot_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()