import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleLocalPosition

import numpy as np

# STATUS: OKAY
    # Telling vehicle to follow a z sine wave and an x sine wave
    # x sine wave serves to keep vehicle going back in forth in the x axis
    # Sine wave to send slightly intelligent vx such that no authority is taken from vz
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
        self.vehicle_local_position = VehicleLocalPosition()

        self.mode = 2   # 0=Z-plane sine, 1=X-plane sine, 2=X and Z-plane sine
        
        self.position_setpoint_y = -50
        self.position_setpoint_x = 50
        self.position_setpoint_y = -120

        self.velocity_setpoint_x = 0
        self.velocity_setpoint_y = 0
        self.velocity_setpoint_z = 0

        self.initialize_position_counter = 0

        self.trajectory_t = 0   # initialize time

        self.z_trajectory_period = 50    # s
        self.z_omega = 2*np.pi/self.z_trajectory_period

        self.x_trajectory_period = 25  # s
        self.x_omega = 2*np.pi/self.x_trajectory_period

        if self.mode == 0 or self.mode == 2:
            # Initialize parameters for z trajectory
            self.z_min = -120    # m
            self.z_amp = -2.5    # m

        if self.mode == 1 or self.mode == 2:
            # Initialize parameters for x trajectory
            self.x_min = 0  # m
            self.x_amp = 150 # m

        """ Create timer and logger """
        self.timer_period = 0.1
        self.timer = self.create_timer(
            self.timer_period , self.timer_callback)
        self.get_logger().info('offboard_control_node started') 

    def publish_heartbeat(self):
        """ offboard control mode message """
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)


    def vehicle_local_position_callback(self, vehicle_local_position):
        """ Position callback """
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """ Status callback """
        self.vehicle_status = vehicle_status

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

    def timer_callback(self):
        """ Timer callback to publish heartbeat and trajectory commands """
        self.publish_heartbeat()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            """ Command specified trajectory """

            if self.initialize_position_counter == 0:
                """ Initialize x and y position commands """
                self.position_setpoint_x = self.vehicle_local_position.x
                self.x_min = self.position_setpoint_x
                self.position_setpoint_y = self.vehicle_local_position.y
                self.initialize_position_counter += 1

            if self.mode == 2:
                """ X and Z sine wave """
                self.position_setpoint_z = self.z_min + self.z_amp*np.sin(self.z_omega*self.trajectory_t)
                self.velocity_setpoint_z = self.z_omega*self.z_amp*np.cos(self.z_omega*self.trajectory_t)

                self.position_setpoint_x = self.x_min + self.x_amp*np.sin(self.x_omega*self.trajectory_t)
                self.velocity_setpoint_x = self.x_omega*self.x_amp*np.cos(self.x_omega*self.trajectory_t)

            self.trajectory_t += self.timer_period

            self.publish_trajectory_setpoint(float(self.position_setpoint_x),
                                             float(self.position_setpoint_y),
                                             float(self.position_setpoint_z), 
                                             float(self.velocity_setpoint_x), 
                                             float(self.velocity_setpoint_y), 
                                             float(self.velocity_setpoint_z))


def main(args=None):
    print('Starting offboard control mode... ')

    rclpy.init(args = args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
