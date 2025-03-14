#! /usr/bin/env python3


import rclpy # Python client library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header  # Import the Header message

# Define constants
names_of_joints = [ 'joint2_to_joint1', 
                    'joint3_to_joint2', 
                    'joint4_to_joint3', 
                    'joint5_to_joint4', 
                    'joint6_to_joint5', 
                    'joint6output_to_joint6']

class BasicJointTrajectoryPublisher(Node):
    """This class executes a sample trajectory for a robotic arm
    
    """      
    def __init__(self):
        """ Constructor.
      
        """
        # Initialize the class using the constructor
        super().__init__('basic_joint_trajectory_publisher')    
 
        # Create the publisher of the desired arm goal poses
        self.pose_publisher = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 1)
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.frame_id = "base"

        # Set the desired goal poses for the robotic arm.
        # To make the code cleaner, I could have imported these positions from a yaml file.
        self.positions = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [-0.0269, -0.0246, 0.00528, -0.00592, 0.00778, -0.03],
            [-0.0538, -0.0492, 0.01056, -0.01184, 0.01556, -0.06],
            [-0.0807, -0.0738, 0.01584, -0.01776, 0.02334, -0.09],
            [-0.1076, -0.0984, 0.02112, -0.02368, 0.03112, -0.12],
            [-0.1345, -0.123, 0.0264, -0.0296, 0.0389, -0.15],
            [-0.1614, -0.1476, 0.03168, -0.03552, 0.04668, -0.18],
            [-0.1883, -0.1722, 0.03696, -0.04144, 0.05446, -0.21],
            [-0.2152, -0.1968, 0.04224, -0.04736, 0.06224, -0.24],
            [-0.2421, -0.2214, 0.04752, -0.05328, 0.07002, -0.27],
            [-0.269, -0.246, 0.0528, -0.0592, 0.0778, -0.3],
            [-0.2959, -0.2706, 0.05808, -0.06512, 0.08558, -0.33],
            [-0.3228, -0.2952, 0.06336, -0.07104, 0.09336, -0.36],
            [-0.3497, -0.3198, 0.06864, -0.07696, 0.10114, -0.39],
            [-0.3766, -0.3444, 0.07392, -0.08288, 0.10892, -0.42],
            [-0.4035, -0.369, 0.0792, -0.0888, 0.1167, -0.45],
            [-0.4304, -0.3936, 0.08448, -0.09472, 0.12448, -0.48],
            [-0.4573, -0.4182, 0.08976, -0.10064, 0.13226, -0.51],
            [-0.4842, -0.4428, 0.09504, -0.10656, 0.14004, -0.54],
            [-0.5111, -0.4674, 0.10032, -0.11248, 0.14782, -0.57],
            [-0.538, -0.492, 0.1056, -0.1184, 0.1556, -0.6],
            [-0.5649, -0.5166, 0.11088, -0.12432, 0.16338, -0.63],
            [-0.5918, -0.5412, 0.11616, -0.13024, 0.17116, -0.66],
            [-0.6187, -0.5658, 0.12144, -0.13616, 0.17894, -0.69],
            [-0.6456, -0.5904, 0.12672, -0.14208, 0.18672, -0.72],
            [-0.6725, -0.615, 0.132, -0.148, 0.1945, -0.75],
            [-0.6994, -0.6396, 0.13728, -0.15392, 0.20228, -0.78],
            [-0.7263, -0.6642, 0.14256, -0.15984, 0.21006, -0.81],
            [-0.7532, -0.6888, 0.14784, -0.16576, 0.21784, -0.84],
            [-0.7801, -0.7134, 0.15312, -0.17168, 0.22562, -0.87],
            [-0.807, -0.738, 0.1584, -0.1776, 0.2334, -0.9],
            [-0.8339, -0.7626, 0.16368, -0.18352, 0.24118, -0.93],
            [-0.8608, -0.7872, 0.16896, -0.18944, 0.24896, -0.96],
            [-0.8877, -0.8118, 0.17424, -0.19536, 0.25674, -0.99],
            [-0.9146, -0.8364, 0.17952, -0.20128, 0.26452, -1.02],
            [-0.9415, -0.861, 0.1848, -0.2072, 0.2723, -1.05],
            [-0.9684, -0.8856, 0.19008, -0.21312, 0.28008, -1.08],
            [-0.9953, -0.9102, 0.19536, -0.21904, 0.28786, -1.11],
            [-1.0222, -0.9348, 0.20064, -0.22496, 0.29564, -1.14],
            [-1.0491, -0.9594, 0.20592, -0.23088, 0.30342, -1.17],
            [-1.076, -0.984, 0.2112, -0.2368, 0.3112, -1.2],
            [-1.1029, -1.0086, 0.21648, -0.24272, 0.31898, -1.23],
            [-1.1298, -1.0332, 0.22176, -0.24864, 0.32676, -1.26],
            [-1.1567, -1.0578, 0.22704, -0.25456, 0.33454, -1.29],
            [-1.1836, -1.0824, 0.23232, -0.26048, 0.34232, -1.32],
            [-1.2105, -1.107, 0.2376, -0.2664, 0.3501, -1.35],
            [-1.2374, -1.1316, 0.24288, -0.27232, 0.35788, -1.38],
            [-1.2643, -1.1562, 0.24816, -0.27824, 0.36566, -1.41],
            [-1.2912, -1.1808, 0.25344, -0.28416, 0.37344, -1.44],
            [-1.3181, -1.2054, 0.25872, -0.29008, 0.38122, -1.47],
            [-1.345, -1.23, 0.264, -0.296, 0.389, -1.5],
            [-1.345, -1.23, 0.264, -0.296, 0.389, -1.5]
        ]

        # Keep track of the current trajectory we are executing
        self.index = 0

        # Indicate the direction of movement in the list of goal positions.
        self.forward = True

    def timer_callback(self):
        """Set the goal pose for the robotic arm.
    
        """
        # Create a new JointTrajectory message
        msg = JointTrajectory()
        msg.header = Header()  # Initialize the header
        msg.header.frame_id = self.frame_id  
        msg.joint_names = names_of_joints

        # Create a JointTrajectoryPoint
        point = JointTrajectoryPoint()
        point.positions = self.positions[self.index]
        point.time_from_start = Duration(sec=0, nanosec=int(self.timer_period * 1e9))  # Time to next position
        msg.points.append(point)
        self.pose_publisher.publish(msg)

        # Move index forward or backward
        if self.forward:
            if self.index < len(self.positions) - 1:
                self.index += 1
            else:
                self.forward = False
        else:
            if self.index > 0:
                self.index -= 1
            else:
                self.forward = True
    
def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)
  
    # Create the node
    basic_joint_trajectory_publisher = BasicJointTrajectoryPublisher()
  
    # Spin the node so the callback function is called.
    rclpy.spin(basic_joint_trajectory_publisher)
    
    # Destroy the node
    basic_joint_trajectory_publisher.destroy_node()
  
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
