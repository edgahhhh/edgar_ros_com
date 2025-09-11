import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, VehicleStatus, VehicleAttitudeSetpoint

import numpy as np

from transforms3d.euler import euler2quat

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
        self.vehicle_attitude_setpoint_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile
        )
        """ Create subscribers """
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        """ Initialize variables """
        self.vehicle_status = VehicleStatus()

        self.roll_t = 0
        self.roll_period = 15   # s
        self.roll_omega = 2*np.pi/self.roll_period  # rad/s
        self.roll_amp = np.pi*1/3   # rad
        self.roll_min = np.pi/2     # rad

        self.euler_setpoint_q = self.roll_min   # rad
        self.euler_setpoint_r = 0
        self.euler_setpoint_w = np.pi/2

        self.quant_setpoint_qd = np.zeros(4)
        self.quant_setpoint_to_pub = [1, 1, 1, 1]
        
        self.thrust_norm_x = 0.9

        """ Create attitude timer """
        self.attitude_timer_freq = 2
        self.attitude_timer_period = 1/ self.attitude_timer_freq
        self.attitude_timer = self.create_timer(
            self.attitude_timer_period, self.attitude_timer_callback
        )
        """ Create timer and logger """
        self.timer_freq = 10    # HZ
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

    def publish_vehicle_attitude_setpoint(self, quaternion:np.float32, thrust_norm_x:float):
        """ publish attitude setpoint """
        msg = VehicleAttitudeSetpoint()
        msg.q_d = quaternion
        msg.thrust_body[0] = thrust_norm_x
        msg.thrust_body[1] = float(0)
        msg.thrust_body[2] = float(0)

        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        self.vehicle_attitude_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing q_d: {quaternion} \nPublishing thrust setpoint: {thrust_norm_x} ")

    def timer_callback(self):
        """ timer callback, publish heartbeat and attitude setpoint """
        self.publish_heartbeat()

    def attitude_timer_callback(self):
        """ attitude timer callback, to publish attitude commands """
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            """ Calculate and command quaternion """
            self.euler_setpoint_q = self.roll_min+self.roll_amp*np.sin(self.roll_omega*self.roll_t)

            self.quant_setpoint_qd = euler2quat(self.euler_setpoint_q, self.euler_setpoint_r, self.euler_setpoint_w)

            self.publish_vehicle_attitude_setpoint(self.quant_setpoint_qd.astype(np.float32), float(self.thrust_norm_x))

            self.roll_t = self.roll_t + self.timer_period


def main(args=None):
    print('Starting offbaord control mode... ')

    rclpy.init(args = args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
