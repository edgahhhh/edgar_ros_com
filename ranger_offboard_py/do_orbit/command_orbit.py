import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleStatus, VehicleCommand

import numpy as np

# STATUS: need to test on ubuntu

class CommandOrbit(Node):
    def __init__(self):
        super().__init__('command_orbit_node')
        """ Configure QOS profile """
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        """ Create publishers """
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )
        """ Create subscribers """
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        """ Initialize variables """
        self.vehicle_status = VehicleStatus()

        self.orbit_radius= 25  # Radius (m)
        self.orbit_velocity = 15    # velocity (m/s)
        self.orbit_yaw_behavior = 2    # yaw behavior, 2 = uncontrolled?
        self.orbit_x_position= 100        # Latitude/X
        self.orbit_y_position= -100        # Longitude/Y
        self.orbit_z_position= -120      # Altitude/Z

        self.orbit_counter = 0

        """ Create timer and logger """
        self.timer_period = 0.1
        self.timer = self.create_timer(
            self.timer_period , self.timer_callback)
        self.get_logger().info('orbit_control_node started') 


    def vehicle_status_callback(self, vehicle_status):
        """ Status callback """
        self.vehicle_status = vehicle_status

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_do_orbit(self, radius, velocity, yaw_behavior, x_position, y_position, z_position):
        """ Publish do orbit vehicle command """
        self.publish_vehicle_command(command=34, 
                                     param1=radius, 
                                     param2=velocity, 
                                     param3=yaw_behavior, 
                                     param5=x_position, 
                                     param6=y_position, 
                                     param7=z_position)
    
    def timer_callback(self):
        if self.orbit_counter == 0:
            """ publish orbit command """
            self.publish_do_orbit(self.orbit_radius, 
                                  self.orbit_velocity,
                                  self.orbit_yaw_behavior,
                                  self.orbit_x_position,
                                  self.orbit_y_position,
                                  self.orbit_z_position)
            self.orbit_counter += 1
            
            
def main(args=None):
    print('Starting command_orbit_node... ')

    rclpy.init(args = args)

    command_orbit = CommandOrbit()

    rclpy.spin('command_orbit')

    command_orbit.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
