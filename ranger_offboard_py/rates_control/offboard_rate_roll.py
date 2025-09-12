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
        self.rate_amp = 0.15  # rad/s
        self.rate_omega = 2*np.pi/self.rate_period    # rad/s
        self.rate_time = 0 # s

        self.setpoint_roll_rate = 0

        self.setpoint_norm_thrust = 0.6

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

    def publish_roll_rate(self, roll_rate:float, thrust_norm_x:float, pitch_rate:float=0, yaw_rate:float=0):
        """ publish body roll rate setpoint """
        msg = VehicleRatesSetpoint()
        msg.roll = roll_rate
        msg.pitch = pitch_rate
        msg.yaw = yaw_rate
        msg.thrust_body = [thrust_norm_x, float(0), float(0)]
        msg.reset_integral = bool(0)
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.vehicle_rates_publisher.publish(msg)
        self.get_logger().info(f"Publishing rates: {[roll_rate, pitch_rate, yaw_rate]}")

    def timer_callback(self):
        """ timer callback to publish heartbeat """
        self.publish_heartbeat()
            
    def rates_timer_callback(self):
        """ Publish rates at a slower Hz than heartbeat """

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.setpoint_roll_rate = self.rate_amp*np.sin(self.rate_omega*self.rate_time)
            
            self.publish_roll_rate(float(self.setpoint_roll_rate), 
                                    float(self.setpoint_norm_thrust))
            
            self.rate_time += self.rates_timer_period

def main(args=None):
    print('Starting offboard control mode... ')

    rclpy.init(args = args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()