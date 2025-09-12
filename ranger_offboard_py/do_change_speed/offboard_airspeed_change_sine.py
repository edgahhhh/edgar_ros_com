import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleStatus, VehicleCommand

import numpy as np

# STATUS ready
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
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )
        """ Create subscribers """
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        """ Create timer and logger """
        self.change_airspeed_timer_frequency = 1    # Hz
        self.change_airspeed_timer_period = 1 / self.change_airspeed_timer_frequency    # s  
        self.change_airspeed_timer = self.create_timer(
            self.change_airspeed_timer_period , self.change_airspeed_timer_callback)
        self.get_logger().info('change_airspeed_node started') 

        """ Initialize variables """
        self.vehicle_status = VehicleStatus()

        self.airspeed_period = 30   # s
        self.airspeed_omega = 2 * np.pi / self.airspeed_period  # rad/s
        self.airspeed_time = 0  # s
        self.airspeed_mid = 18  # m/s
        self.airspeed_amp = 2   # m/s

        self.setpoint_airspeed = self.airspeed_mid  # m/s
        self.setpoint_throttle = -1  # 0-1, (-1 = not controlled)

    def vehicle_status_callback(self, vehicle_status):
        """ Status callback """
        self.vehicle_status = vehicle_status

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
        self.get_logger().info(f"Publishing vehicle command: {command} \nparams: {params}")

    def publish_do_change_speed(self, Speed:float, Throttle:float):
        """  Publish do_change_speed vehicle command """
        self.publish_vehicle_command(command = 178, 
                                     param1 = float(0), 
                                     param2 = Speed, 
                                     param3 = Throttle)

    def change_airspeed_timer_callback(self):
        """ Publish orbit command as sine wave """
        self.setpoint_airspeed = self.airspeed_mid + self.airspeed_amp*np.sin(self.airspeed_omega*self.airspeed_time)

        self.publish_do_change_speed(float(self.setpoint_airspeed), float(self.setpoint_throttle))
        
        self.airspeed_time += self.change_airspeed_timer_period
        
def main(args=None):
    print('Starting offboard control mode... ')

    rclpy.init(args = args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
