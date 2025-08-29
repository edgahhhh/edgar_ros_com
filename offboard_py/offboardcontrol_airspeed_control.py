# OFFBOARDCONTROL_AIRSPEED_CONTROL.py

import rclpy
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleThrustSetpoint, VehicleStatus, VehicleCommand, AirspeedValidated
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
        self.vehicle_thrust_setpoint_publisher = self.create_publisher(
            VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint', qos_profile
            )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )
        """ Create subscribers """
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile
        )
        self.airspeed_validated_subscriber = self.create_subscription(
            AirspeedValidated, '/fmu/out/airspeed_validated', self.airspeed_validated_callback, qos_profile
        )
        """ Create timer and logger """
        self.timer = self.create_timer(
            0.1 , self.timer_callback)
        self.get_logger().info('offboard_control_node started') 
        """ Initialize variables """
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        self.airspeed_true = 0
        self.x_thrust_setpoint = 0

    """ Create callbacks for subscribers """
    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status
    def airspeed_validated_callback(self, airspeed_validated):
        self.airspeed_validated = airspeed_validated

    def publish_heartbeat(self):
        """ Publish heartbeat """
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.thrust_and_torque = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

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

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def publish_thrust_command(self, x_thrust_normalized: float, y_thrust_normalized: float = 0.0, z_thrust_normalized: float = 0.0):
        """ 
        Publish thrust command 
        ------------------------
        @param x_thrust_normalized: Thrust command normalized b/w [-1, 1]
        
        """
        msg = VehicleThrustSetpoint()
        msg.xyz = [x_thrust_normalized, y_thrust_normalized, z_thrust_normalized]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_thrust_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing thrust command {[x_thrust_normalized, y_thrust_normalized, z_thrust_normalized]}")
    
    def timer_callback(self):
        """ Timer callback to publish heartbeat and thrust commands """
        self.publish_heartbeat()

        if self.offboard_setpoint_counter == 0:
            """ Initialize controller """
            self.pid_controller = PIDController(
                Kp = 0.1, Ki = 0.001, Kd = 0, target_air_speed= 30, de_feedforward = 0, time_step = 1, offboard_time_step= 0.1)
            
        if self.offboard_setpoint_counter < 10:
            self.offboard_setpoint_counter += 1

        if self.offboard_setpoint_counter >= 10 and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            """ Attempt to engage offboard mode after 1 second of publishing setpoints """
            self.engage_offboard_mode()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            """ Publish thrust commands from controller to vehicle to try to achieve target speed """
            self.airspeed_true = self.airspeed_validated.true_airspeed_m_s
            self.x_thrust_setpoint = self.pid_controller.compute(self)
            self.publish_thrust_command(float(self.x_thrust_setpoint))


class PIDController:
    """ 
    PID Controller to cascade thrust control w/ airspeed set points
    -------------------------------------------
    @param Kp: Proportional gain
    @param Ki: Integral gain
    @param Kd: Derivative gain
    @param target_air_speed: Target airspeed to be achieved (m/s)
    @param Fx_feedforward: Feedforward thrust to be subtracted from PID output (0 to 1)
    @param pid_time_step: Time step to run PID controller (s), default 0.1s
    @param offboard_time_step: Time step of offboard control loop (s), default 0.1s

    """
    def __init__(self, Kp, Ki, Kd, target_air_speed, Fx_feedforward = 0, pid_time_step = 0.1, offboard_time_step = 0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target_air_speed = target_air_speed
        self.Fx_feedforward = Fx_feedforward
        self.integral_error = 0
        self.previous_error = 0
        self.time_step = pid_time_step 
        self.discrete_time_step = self.time_step / offboard_time_step
        self.discrete_counter = 0

    def saturate(self, value, min_value = 0 , max_value = 1):
        """
        Saturate value to be between min and max values
        -------------------------------------------
        @param value: Value to be saturated
        @param min_value: Minimum value, inclusive
        @param max_value: Maximum value, inclusive

        @return value_saturated: Saturated value

        """
        value_saturated = max(min(value, max_value), min_value)
        return value_saturated

    def compute(self, airspeed_true):
        """ 
        Compute error and compute command to plant 
        -------------------------------------------
        @param airspeed_true: Sensed airspeed

        @return Fx: Thrust command to vehicle

        """
        if self.discrete_counter < self.discrete_time_step:
            """ Run through PID control loop at beginning of time step"""
            self.discrete_counter += 1
            if self.discrete_counter == 1:
                airspeed_error = self.target_air_speed - airspeed_true
                p_gain = self.Kp * airspeed_error
                i_gain = self.integral_error + (self.Ki * airspeed_error * self.time_step)
                d_gain = self.Kd * (airspeed_error - self.previous_error) / self.time_step
                self.previous_error = airspeed_error
                Fx_feedback = p_gain + i_gain + d_gain
                Fx = self.saturate(Fx_feedback - self.Fx_feedforward)
                return Fx
        
        elif self.discrete_counter >= self.discrete_time_step:
            """ Reset discrete counter at end of time step """
            self.discrete_counter = 0
            return None
        
        else:
            sys.exit("Error in discrete counter")
            return None


def main(args=None):
    print('Starting heartbeat signal node... ')
    rclpy.init(args = args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
