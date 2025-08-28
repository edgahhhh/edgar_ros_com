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
        self.waypoint_distance_x= 100   # distance for heading set point in m
        self.x_position = 0
        self.y_position = 0
        self.z_position = 0

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
        Next, after 10 clicks the autopilot will switch into offboard control mode.

        Once in offboard control, the vehicle will reach to some set altitude, then start telling itself to go to
        some randomly generated waypoint, once waypoint is reached the next one is generated and so on.

        NED coordinate system
        """
        self.publish_heartbeat()    # Send heartbeat signal as proof of life
        self.resettable_counter()   # Call discrete counter for use in logic

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
                # Generate y and z position to hold
            self.y_position = self.vehicle_local_position.y
            self.z_position = self.vehicle_local_position.z

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            """ 
            Command rates logic 
            Try same logic as the position commands, where a random rate is generated.
            This time let the discrete counter be shorter as to not have the vehicle go crazy
            In actual use case there will be a seperate controller commanding rates so we could also model a controller as well

            Try this:
            1. Command some rates
            2. After some time set rates to NAN
            3. After some time start back at 1.

            Next we can try this:
            1. Command some position to chase and some random rate 
            2. Keep commanding position but set rates to NAN
            3. After some time start back at 1

            Even more later on try this instead:
            
            """

            # Telling plane to hold some z and y, and chase some x
            self.x_position = self.vehicle_local_position.x + self.chase_dist_m

            self.publish_position_setpoint( 
                float(0),                     # x position, chase
                float(0),                     # y position, hold
                float(self.z_position))                     # z position, hold

            # This altitude command kind of worked but plane climbed slow, probably due to circling
            # # Position command, maybe set the z position as some constant and modify the constant here
            # if self.vehicle_local_position.z > -80:
            #     # Climb command
            #     self.publish_position_setpoint( 
            #         float(self.x_position),                     # x position, defined by heading
            #         float(self.y_position),                     # y position, defines by heading
            #         float(self.vehicle_local_position.z - 5))   # z position, climb a little higher
            # elif self.vehicle_local_position.z < -100:
            #     # Descend command
            #     self.publish_position_setpoint( 
            #         float(self.x_position),                     # x position, defined by heading
            #         float(self.y_position),                     # y position, defines by heading
            #         float(self.vehicle_local_position.z + 5))   # z position, controlled
            # else:
            #     # Hold altitude at -90
            #     self.publish_position_setpoint( 
            #         float(self.x_position),                     # x position, defined by heading
            #         float(self.y_position),                     # y position, defines by heading
            #         float(-90))                                 # z position, controlled


            # Generating a random heading for the plane to go to every time the timer resets
            # Start at 1 because the counter will be 0 the first time we get here
            # Generate some heading angle and command the vehicle to approach the x, y position in that direction
            # In this case heading 0 is North and goes counter clockwise
            # if self.counter == 1:
            #     self.heading = np.random.randint(0, 2) * np.pi
            #     self.x_position = self.vehicle_local_position.x - self.flat_dist_m * np.cos(self.heading)
            #     self.y_position = self.vehicle_local_position.y - self.flat_dist_m * np.sin(self.heading)








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
    