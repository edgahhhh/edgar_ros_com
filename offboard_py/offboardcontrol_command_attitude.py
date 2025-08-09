# OFFBOARDCONTROL_POSITION_SETPOINTS.py

import rclpy        # RCLPY wiki is your bible here
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleAttitude, VehicleAttitudeSetpoint, VehicleStatus, VehicleCommand
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
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
            )
        self.vehicle_attitude_setpoint_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )
        # Create subscribers for node
        # These are for echoing from ros2 interface?
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile
        )
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile
        )
        self.vehicle_status_subscriber = self.create_subscription(
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
        self.timer_reset_sec =    # Time to generate a new heading
        self.counter_reset_discrete = self.timer_reset_sec / (self.timer.timer_period_ns/1000000000) + 1 
        self.counter = 0
        self.flat_dist_m = 2    # distance for heading set point in m
        self.x_position = np.nan
        self.y_position = np.nan

        self.rates_logic = 1    # 1 = rates only, 2 = rates and position
        self.p_body = 0         # roll rate
        self.q_body = 0         # pitch rate
        self.r_body = 0         # yaw rate




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

    # Attitude callback
    def vehicle_attitude_callback(self, vehicle_attitude):
        self.vehicle_attitude = vehicle_attitude

    # Position callback
    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position

    # Vehicle status callback
    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")
        # If issues start with the attitude controller, reset the integral when starting offboard mode.

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

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # 0 yaw angle
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_attitude_setpoint(self, p_body: float, q_body: float, r_body: float):
        """
        Publish attitude setpoint
        --------------------------
        p_body : desired roll angle (rad)
        q_body : desired pitch angle (rad)
        r_body : desired yaw angle
        ---------------------------
        Angles are derived from the NED coordinate system, refer to : https://docs.px4.io/main/en/config/flight_controller_orientation.html
        
        """
        msg = VehicleAttitudeSetpoint()
        msg.roll_body = p_body
        msg.pitch_body = q_body
        msg.yaw_body = r_body
        msg.yaw_sp_move_rate = 1    # rad/s (only in use if commanding the yaw body)
        # Not issuing thrust or inragral commands just yet
        self.vehicle_attitude_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing rates {[p_body, q_body, r_body]}")

    def resettable_counter(self):
        """Resettable counter for discrete logic"""
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
        Next, after 10 clicks the autopilot will switch into offboard control mode.

        Once in offboard control, the vehicle will reach to some set altitude, then start telling itself to go to
        some randomly generated waypoint, once waypoint is reached the next one is generated and so on.

        NED coordinate system
        """
        self.publish_heartbeat()
        self.resettable_counter()   # Call the counter for every time this loop is called

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:











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
    