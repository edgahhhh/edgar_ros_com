import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode
 
class HeartbeatPublisher(Node):
    """ Node for sending a heartbeat to the vehicle."""
    
    def __init__(self):
        super().__init__('heartbeat_publisher_node')
        
        # Create publishers
        self.publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        
        # Create a timer and a logger
        self.timer = self.create_timer(
            1.0, self.publish_heartbeat)
        
        self.get_logger().info('heartbeat_publisher_node started')

    def publish_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

def main(args=None):
    print('Starting heartbeat signal node... ')
    rclpy.init(args = args)
    offboard_control = HeartbeatPublisher()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__'
    main()