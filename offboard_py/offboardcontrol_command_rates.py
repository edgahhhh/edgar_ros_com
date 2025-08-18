# OFFBOARDCONTROL_COMMAND_RATES.py

import rclpy        # RCLPY wiki is your bible here
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleRatesSetpoint, VehicleLocalPosition, VehicleStatus, VehicleCommand
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
        self.vehicle_rates_setpoint_publisher = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile
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

        # Creating a timer and logger
        self.timer = self.create_timer(
            0.1 , self.timer_callback)

        self.get_logger().info('offboard_control_node started') 

        # Initialize variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0

        self.counter = 0
        self.flat_dist_m = 2    # distance for heading set point in m
        self.x_position = np.nan
        self.y_position = np.nan

        self.rates_logic = 1            # 1 = rates only, 2 = rates and position
        self.rates_to_control = 1       # 1 = p and q only,   2 = r only
        self.timer_reset_sec = 8       # Interval for rates logic, integer
        self.counter_reset_discrete = self.timer_reset_sec / (0.1) + 1
        if self.rates_to_control == 1:
            self.p_body = 0.1           # roll rate (rad/s)
            self.q_body = 0.25           # pitch rate
            self.r_body = 0.0           # yaw rate, not controlled?



    # Heartbeat signal publisher
    def publish_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = True
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

    def publish_vehicle_rates_setpoint(self, p: float, q: float, r: float, T: float):
        """
        Publish vehicle rates setpoint
        ------
        p : roll rate (rad/s)
        q : pitch rate (rad/s)
        r : yaw rate (rad/s)
        T : normlaized throttle command [-1, 1]
        """
        msg = VehicleRatesSetpoint()
        msg.roll = p 
        msg.pitch = q
        msg.yaw = r
        msg.thrust_body = [T, float(0), float(0)]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_rates_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing rate setpoints {[p, q, r]}. Publishing throttle {T}")

    # Note sure if im going to need this or not for this use case...
    # def publish_position_setpoint(self, x: float, y: float, z: float):
    #     """Publish the trajectory setpoint."""
    #     msg = TrajectorySetpoint()
    #     msg.position = [x, y, z]
    #     msg.yaw = 1.57079  # 0 yaw angle
    #     msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    #     self.trajectory_setpoint_publisher.publish(msg)
    #     self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

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

        First, a heartbeat is always sent to the autopilot for proof of life.
        Next, after 10 clicks the autopilot will switch into offboard control mode.

        Once in offboard control, the vehicle will reach to some set altitude, then start telling itself to go to
        some randomly generated waypoint, once waypoint is reached the next one is generated and so on.

        NED coordinate system
        """
        self.publish_heartbeat()
        self.resettable_counter()   # Call the counter for every time this loop is called
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            # print(self.vehicle_status.nav_state)
            # print(VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1



        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            """ 
            Command rates logic 
            Try same logic as the position commands, where a random rate is generated.
            This time let the discrete counter be shorter as to not have the vehicle go crazy
            In actual use case there will be a separate controller commanding rates so we could also model a controller as well

            Try this:
            1. Command some rates
            2. After some time set rates to NAN
            3. After some time start back at 1.

            Next we can try this:
            1. Command some position to chase and some random rate 
            2. Keep commanding position but set rates to NAN
            3. After some time start back at 1

            """

            # Option 1, rates only
            if self.rates_logic == 1:
                """
                The logic here is to have some discrete interval, 
                where half of it the rate is controlled and the other half the rates aren't controlled
                """
                self.interval_on = 0.5 * self.counter_reset_discrete + 1
                self.interval_off = self.counter_reset_discrete + 1 - self.interval_on
                # print(self.counter)
                if self.counter <= self.interval_on:
                    self.publish_vehicle_rates_setpoint(
                        float( self.p_body) ,       # roll rate
                        float( self.q_body ),       # pitch rate
                        float( self.r_body ),       # yaw rate
                        float(0.5)                         # 50 % throttle
                    )
                else:
                    self.publish_vehicle_rates_setpoint(
                        float(-self.p_body),
                        float(0),
                        float(-self.r_body),
                        float(0.5)
                    )
            # # Option 2
            # elif self.rates_logic == 2:
            #     print()









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
    