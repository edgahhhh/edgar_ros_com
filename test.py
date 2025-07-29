import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode


class OffboardControl(Node):
    """ Creating a ros2 node called offboard_control_node"""
    def __init__(self):
        super().__init__('offboard_control_node')
        
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)


    