import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode
 
class HeartbeatPublisher(Node):
    """ Creating a ros2 node called heartbeat_publisher_node"""
    def __init__(self):
        super().__init__('heartbeat_publisher_node')
        
        self.publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        
        self.timer = self.create_timer(
            1.0, self.publish_heartbeat)
        
        self.get_logger().info('heartbeat_publisher_node started')
