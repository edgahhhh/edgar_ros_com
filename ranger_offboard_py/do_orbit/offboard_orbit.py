import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus, VehicleCommand

import numpy as np


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control_node')
        """ Configure QOS profile """
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        """ Create publishers """
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
            )
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
        self.orbit_velocity= 2*np.pi*self.orbit_param1*20    # Velocity (m/s)
        self.orbit_yaw_behavior= np.nan    # yaw behavior
        self.orbit_x_position= 50        # Latitude/X
        self.orbit_y_position= 50        # Longitude/Y
        self.orbit_z_position= -120      # Altitude/Z

        """ Create timer and logger """
        self.timer_period = 0.1
        self.timer = self.create_timer(
            self.timer_period , self.timer_callback)
        self.get_logger().info('offboard_control_node started') 

    def publish_heartbeat(self):
        """ offboard control mode message """
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    """ Position callback """
    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position
    """ Status callback """
    def vehicle_status_callback(self, vehicle_status):
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
        self.publish_vehicle_command(command=34, param1=radius, param2=velocity, param3=yaw_behavior, param5=x_position, param6=y_position, param7=z_position)

    def timer_callback(self):
        """ Timer callback to publish heartbeat and trajectory commands """
        self.publish_heartbeat()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            """ Command do_orbit """
            self.publish_do_orbit(radius=self.orbit_radius, 
                                  velocity=self.orbit_velocity,
                                  yaw_behavior=self.orbit_yaw_behavior,
                                  x_position=self.orbit_x_position,
                                  y_position=self.orbit_y_position,
                                  z_position=self.orbit_z_position)
            
            
def main(args=None):
    print('Starting offboard control mode... ')

    rclpy.init(args = args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
