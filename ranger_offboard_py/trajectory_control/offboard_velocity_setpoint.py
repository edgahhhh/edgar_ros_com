import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus, VehicleCommand

import numpy as np

# STATUS: NOT READY

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
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile
        )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        """ Initialize variables """
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        
        self.mode = 0   # 0=Constant Velocity, 1=Dynamic Velocity
        if self.mode == 0:
            # Constant velocity vector
            self.velocity_setpoint_x = -1000
            self.velocity_setpoint_y = -1000
            self.velocity_setpoint_z = -1000

        elif self.mode == 1:
        # Velocity trajectory
            self.period = 100   # seconds

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

    """ Position callback """
    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position
    """ Status callback """
    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def publish_velocity_setpoint(self,vx:float, vy:float, vz:float):
        """Publish trajectory setpoint."""
        msg = TrajectorySetpoint()

        msg.position[0] = float(self.vehicle_local_position.x)
        msg.position[1] = float(self.vehicle_local_position.y)
        msg.position[2] = float(self.vehicle_local_position.z)

        msg.velocity[0] = vx
        msg.velocity[1] = vy
        msg.velocity[2] = vz

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing velocity setpoints: {[vx, vy, vz]}")

    def timer_callback(self):
        """ Timer callback to publish heartbeat and trajectory commands """
        self.publish_heartbeat()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            """ Command vehicle to follow velocity trajectory """

            self.publish_velocity_setpoint(float(self.velocity_setpoint_x), float(self.velocity_setpoint_y), float(self.velocity_setpoint_z))
            
def main(args=None):
    print('Starting offbaord control mode... ')

    rclpy.init(args = args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
