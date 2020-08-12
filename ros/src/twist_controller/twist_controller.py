import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
from data_types import ControlCommand

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    """
    Uses PID and LowPassFilter for acceleration control.
    Uses YawController for steering angle control.

    Throttle values should be in the range 0 to 1. A throttle of 1 means
    fully engaged.

    Brake values should be in units of torque (N*m). The correct values can
    be computed using the desired acceleration, weight of the vehicle, and
    wheel radius.
    """

    def __init__(self, parameters):
        self.params = parameters

        # Yaw Controller
        self.yaw_controller = YawController(
            self.params.wheel_base,
            self.params.steer_ratio,
            0.1,  # min speed
            self.params.max_lat_accel,
            self.params.max_steer_angle,
        )

        # Throttle Controller
        kp = 0.2
        ki = 0.05
        kd = 0.01
        mn = 0.0  # min throttle
        mx = 1.0  # max throttle
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # LowPassFilter
        tau = 0.5  # 1/(2pi*tau) = cutoff frequency
        ts = 0.02  # sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.last_time = rospy.get_time()

    def control(self, current_vel, target_vel, is_dbw_enabled):
        command = ControlCommand()
        if not is_dbw_enabled:
            self.throttle_controller.reset()
            return command

        # linear error
        current_vel_linear = self.vel_lpf.filt(current_vel.linear)
        vel_error = target_vel.linear - current_vel_linear
        self.last_vel = current_vel

        # time
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        # control action
        command.throttle = self.throttle_controller.step(vel_error, sample_time)
        command.brake = 0

        if target_vel.linear == 0.0 and current_vel_linear < 0.1:
            # stop vehicle
            command.throttle = 0
            command.brake = 700  # !

        elif command.throttle < 0.1 and vel_error < 0:
            # almost stopped
            mass = self.params.vehicle_mass
            wheel_radius = self.params.wheel_radius
            decel = max(vel_error, self.params.decel_limit)

            command.throttle = 0
            command.brake = abs(decel) * mass * wheel_radius

        command.steer = self.yaw_controller.get_steering(
            target_vel.linear, target_vel.angular, current_vel_linear
        )

        return command
