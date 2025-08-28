# OFFBOARDCONTROL_TRAJECTORY_WAYPOINTS.py

import rclpy        # RCLPY wiki is your bible here
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectoryWaypoint, VehicleLocalPosition, VehicleStatus, VehicleCommand
import numpy as np

class OffboardControl(Node):
    """ Creating node to control position of vehicle. """

    def __init__(self):
        super().__init__('offboard_control_node')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers for node
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
            )
        self.trajectory_waypoint_publisher = self.create_publisher(
            TrajectoryWaypoint, '/fmu/in/trajectory_waypoint', qos_profile
            )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )
        # Create subscribers for node
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile
        )
        self.vehicle_stats_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )

        # Creating a timer and logger
        self.timer = self.create_timer(
            0.1 , self.timer_callback)
        self.get_logger().info('offboard_control_node started') 

        # Initialize variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        self.timer_reset_sec = 60   # Time to generate a new heading
        self.counter_reset_discrete = self.timer_reset_sec / (self.timer.timer_period_ns/1000000000) + 1 
        self.counter = 0
        self.waypoint_distance_x= 100   # distance for x waypoint in m
        self.waypoint_distance_y = 100  # distance for y waypoint in m
        self.x_position_waypoint = 0
        self.y_position_waypoint = 0
        self.z_position_waypoint = 0
        self.waypoint_counter = 0

    # Heartbeat signal publisher
    def publish_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    # Position callback
    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position

    # Vehicle status callback
    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    # Publish vehicle command
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

    # Offboard mode engage
    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def publish_trajectory_waypoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectoryWaypoint()
        msg.position = [x, y, z]
        msg.yaw = float(0)  # 90 degree yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    # White trash a discrete counter that's good enough
    def resettable_counter(self):
        # Count up if below reset threshold
        if  self.counter < self.counter_reset_discrete and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.counter += 1
        # Reset if at or above the reset threshold
        elif self.counter >= self.counter_reset_discrete and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.counter = 1

    def timer_callback(self):   # This runs every time period defined by the timer
        """
        Main loop and timer callback for script

        First, a heartbeat is always sent to the autopilot for proof of life @ 10 Hz
        Next, after 10 clicks (1 seconds) the autopilot will switch into offboard control mode.

        While in offboard mode, command vehicle to approach some waypoint in the N and E frame: (x,y)
        """
        self.publish_heartbeat()    # Send heartbeat signal as proof of life
        # self.resettable_counter()   # Call discrete counter for use in logic

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
                # Generate y and z position to hold
            self.y_position = self.vehicle_local_position.y
            self.z_position = self.vehicle_local_position.z

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            """
            Command vehicle waypoints using TrajectoryWaypoint message
            """

            if self.waypoint_counter < 1:
                """ Publish waypoint only once"""
                self.waypoint_counter += 1
                self.x_position_waypoint = self.vehicle_local_position.x + self.waypoint_distance_x
                self.y_position_waypoint = self.vehicle_local_position.y + self.waypoint_distance_y
                self.z_position_waypoint = np.nan
            
                self.publish_trajectory_waypoint( 
                    float(self.x_position_waypoint),                     # x position, chase
                    float(self.y_position_waypoint),                     # y position, hold
                    float(self.z_position_waypoint))                     # z position, hold









# My guess is that rclpy is used to begin this script before stopping itself once completed
def main(args=None):
    print('Starting heartbeat signal node... ')
    rclpy.init(args = args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


    

if __name__ == '__main__':
    main()
    