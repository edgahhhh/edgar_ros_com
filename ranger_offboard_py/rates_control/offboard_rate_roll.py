import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, VehicleRatesSetpoint, VehicleStatus

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
        """ Create subscribers """
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        """ Initialize variables """
        self.vehicle_status = VehicleStatus()

        self.rate_period = 5   # s
        self.rate_amp = 1  # rad/s
        self.rate_omega = 2*np.pi/self.rate_period    # rad/s

        self.rate_t = 0 # s
        self.rate_setpoint_roll = 0

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

    def publish_roll_rate(self, p_dot:float, thrust_norm_x:float):
        """ publish body roll rate setpoint """
        msg = VehicleRatesSetpoint()
        msg.roll = p_dot
        msg.pitch = float(np.nan)
        msg.yaw = float(np.nan)
        msg.thrust_body[0] = thrust_norm_x
        msg.thrust_body[1] = float(np.nan)
        msg.thrust_body[2] = float(np.nan)

        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.vehicle_rates_publisher.publish(msg)
        self.get_logger().info(f"Publishing rates: {[p_dot, np.nan, np.nan]}")

    def timer_callback(self):
        """ timer callback, publish heartbeat and roll command when in offboard mode 
        the 10Hz might be too fast of a frequency for the controllers if the rates are constantly updating, 
        if so, try a smaller frequency, like 5 Hz, by creating a new timer"""
        self.publish_heartbeat()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            """ Command roll rate """
            self.rate_setpoint_roll = self.rate_amp*np.sin(self.rate_omega*self.rate_t)

            self.rate_t = self.rate_t + self.timer_period
            
            self.publish_roll_rate(self.rate_setpoint_roll, self, )

def main(args=None):
    print('Starting offbaord control mode... ')

    rclpy.init(args = args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()