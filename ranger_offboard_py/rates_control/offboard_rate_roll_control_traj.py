import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, VehicleRatesSetpoint, TrajectorySetpoint, VehicleStatus, VehicleLocalPosition

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
        self.vehicle_rates_publisher = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
        )
        """ Create subscribers """
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile
        )
        """ Initialize variables """
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition

        self.rate_period = 5   # s
        self.rate_amp = 0.25  # rad/s
        self.rate_omega = 2*np.pi/self.rate_period    # rad/s

        self.rate_t = 0 # s
        self.rate_setpoint_roll = 0

        self.norm_thrust_setpoint = 0.7

        self.position_setpoint_x = 0
        self.position_setpoint_y = 0
        self.position_setpoint_z = -100

        self.velocity_setpoint_x = 1
        self.velocity_setpoint_y = 0
        self.velocity_setpoint_z = 0

        self.position_counter = 0

        """ Create rates timer """
        self.rates_timer_freq = 2   # Hz
        self.rates_timer_period = 1/self.rates_timer_freq
        self.rates_timer = self.create_timer(
            self.rates_timer_period, self.rates_timer_callback)
        
        """ Create timer and logger """
        self.timer_freq = 10
        self.timer_period = 1/self.timer_freq
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
        msg.body_rate = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def vehicle_status_callback(self, vehicle_status):
        """ vehicle status callback """
        self.vehicle_status = vehicle_status

    def vehicle_local_position_callback(self, vehicle_local_position):
        """ vehicle local position callback """
        self.vehicle_local_position = vehicle_local_position

    def publish_trajectory_setpoint(self, x:float, y:float, z:float, vx:float, vy:float, vz:float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position[0] = x
        msg.position[1] = y
        msg.position[2] = z

        msg.velocity[0] = vx
        msg.velocity[1] = vy
        msg.velocity[2] = vz

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing positions: {[x, y, z]} \nPublishing velocities: {[vx, vy, vz]}")


    def publish_roll_rate(self, p_dot:float, thrust_norm_x:float):
        """ publish body roll rate setpoint """
        msg = VehicleRatesSetpoint()
        msg.roll = p_dot
        msg.pitch = float(0)
        msg.yaw = float(0)
        msg.thrust_body = [thrust_norm_x, float(0), float(0)]
        msg.reset_integral = bool(1)
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.vehicle_rates_publisher.publish(msg)
        self.get_logger().info(f"Publishing rates: {[p_dot, float(0), float(0)]}")

    def timer_callback(self):
        """ timer callback """
        self.publish_heartbeat()

        # if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     """ Command roll rate  and trajectory """

        #     self.position_setpoint_y = self.vehicle_local_position.y

        #     if self.vehicle_local_position.x < 0:
        #         self.position_setpoint_x = 300
        #         self.velocity_setpoint_x = 1
        #         self.position_counter = 0
        #     elif self.vehicle_local_position.x > 300:
        #         self.position_setpoint_x = 0
        #         self.velocity_setpoint_x = -1
        #         self.position_counter = 0

        #     if self.position_counter == 0:
        #         self.publish_trajectory_setpoint(float(self.position_setpoint_x),
        #                                         float(self.position_setpoint_y),
        #                                         float(self.position_setpoint_z), 
        #                                         float(self.velocity_setpoint_x),
        #                                         float(self.velocity_setpoint_y),
        #                                         float(self.velocity_setpoint_z))
        #         self.position_counter += 1
            
    def rates_timer_callback(self):
        """ Publish rates at a different Hz than heartbeat """

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.rate_setpoint_roll = self.rate_amp*np.sin(self.rate_omega*self.rate_t)

            self.rate_t = self.rate_t + self.rates_timer_period
            
            self.publish_roll_rate(float(self.rate_setpoint_roll), 
                                    float(self.norm_thrust_setpoint))

def main(args=None):
    print('Starting offbaord control mode... ')

    rclpy.init(args = args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()