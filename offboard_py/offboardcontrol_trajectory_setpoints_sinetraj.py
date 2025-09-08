# OFFBOARDCONTROL_POSITION_SETPOINTS.py
# This script is to enable offboard control upon execution and send some position set point to the simulator

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus, VehicleCommand

import numpy as np

class OffboardControl(Node):
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
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )
        # Create subscribers for node
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile
        )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        self.timer_period = 0.1
        # Creating a timer and logger
        self.timer = self.create_timer(
            self.timer_period , self.timer_callback)
        self.get_logger().info('offboard_control_node started') 

        # Initialize variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        self.offboard_setpoint_counter = 0

        # X and Z trajectories share the same period and time constants
        self.traj_t = 0
        self.period = 50  # s

        # x_trajectory as function of time
        # Linear change of target x
        self.x_min = 0
        self.x_target = -1000 # m
        self.dx = self.x_target / self.period   # m/s
        self.x_tangent = 0

        # y trajectory
        self.y_position = 0
        self.y_tangent = 0

        # z_trajectory as function of time
        # Sine wave w/ amplitude of altitude change over some period
        self.z_min = -50
        self.z_amp = -10  # m
        self.omega = 2 * np.pi / self.period 
        self.sine_calc = 0
        self.z_tangent = 0

        # Counter for x position
        self.x_position_counter = 0

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

    def publish_trajectory_setpoint(self, x:float, y:float, z:float, vx:float, vy:float, vz:float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position[0] = x
        msg.position[1] = y
        msg.position[2] = z

        msg.velocity[0] = vx
        msg.velocity[1] = vy
        msg.velocity[2] = vz
        
        # msg.yaw = float(0)
        # msg.yawspeed = float(0)

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints: {[x, y, z]} \nPublishing velocity setpoints {[vx, vy, vz]}")

    def timer_callback(self):
        """ Timer callback to publish heartbeat and trajectory commands """
        self.publish_heartbeat()

        if self.offboard_setpoint_counter == 20:
            self.engage_offboard_mode()

        if self.offboard_setpoint_counter < 21:
            """ Attempt to engage offbaord mode """
            self.offboard_setpoint_counter += 1

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            """ Command altitude change as a wave and a forward moving x point, holding y. Vehicle
            should move in xz plane only """

            if self.x_position_counter == 0:
                self.x_min = self.vehicle_local_position.x
            if self.x_position_counter < 1:
                self.x_position_counter += 1

            # -- SHM trajectory --
            self.traj_t = self.traj_t + self.timer_period
            self.sine_calc = np.sin(self.omega * self.traj_t)

            # self.x_position = self.x_min + self.dx*self.traj_t
            self.x_position = self.x_min + self.z_amp*self.sine_calc
            self.z_position = self.z_min + self.z_amp*self.sine_calc

            self.x_tangent = self.dx
            self.y_tangent = 0
            self.z_tangent = self.omega*self.z_amp*np.cos(self.omega*self.traj_t)
            self.z_tangent = 0
            # Debugging variables
            self.x_position = 0
            self.y_position = 0
            self.z_position = -100

            if self.vehicle_local_position.x > self.x_position:
                self.x_tangent = -1
            else:
                self.x_tangent = 1

            # Setting speeds to NaN
            # self.x_tangent = np.nan
            # self.y_tangent = np.nan
            # self.z_tangent = np.nan

            self.publish_trajectory_setpoint(
                x = float(self.x_position),
                y = float(self.y_position),
                z = float(self.z_position),
                vx = float(self.x_tangent),
                vy = float(self.y_tangent),
                vz = float(self.z_tangent)
                )


def main(args=None):
    print('Starting heartbeat signal node... ')

    rclpy.init(args = args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
    