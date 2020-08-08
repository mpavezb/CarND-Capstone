#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from data_types import ControlParameters, Velocity, ControlCommand
from twist_controller import Controller

"""
Keep in mind  the status of `dbw_enabled`. While in the simulator, its enabled
all the time, in the real car, that will not be the case. This may cause the
PID controller to accumulate error because the car could temporarily be driven
 by a human instead of your controller.
"""

# The DBW system expects messages at 50Hz, and will disengage (reverting
# control back to the driver) if control messages are published at less
# than 10hz.
FREQUENCY = 50

"""
The car has an automatic transmission, which means the car will roll forward
if no brake and no throttle is applied. To prevent it from moving requires
about 700 Nm of torque.
"""


class DBWNode(object):
    def __init__(self):
        rospy.init_node("dbw_node")

        # Parameters
        params = ControlParameters()
        params.vehicle_mass = rospy.get_param("~vehicle_mass", 1736.35)
        params.fuel_capacity = rospy.get_param("~fuel_capacity", 13.5)
        params.brake_deadband = rospy.get_param("~brake_deadband", 0.1)
        params.decel_limit = rospy.get_param("~decel_limit", -5)
        params.accel_limit = rospy.get_param("~accel_limit", 1.0)
        params.wheel_radius = rospy.get_param("~wheel_radius", 0.2413)
        params.wheel_base = rospy.get_param("~wheel_base", 2.8498)
        params.steer_ratio = rospy.get_param("~steer_ratio", 14.8)
        params.max_lat_accel = rospy.get_param("~max_lat_accel", 3.0)
        params.max_steer_angle = rospy.get_param("~max_steer_angle", 8.0)

        # Publishers
        self.steer_pub = rospy.Publisher(
            "/vehicle/steering_cmd", SteeringCmd, queue_size=1
        )
        self.throttle_pub = rospy.Publisher(
            "/vehicle/throttle_cmd", ThrottleCmd, queue_size=1
        )
        self.brake_pub = rospy.Publisher("/vehicle/brake_cmd", BrakeCmd, queue_size=1)

        # Controller
        self.command = ControlCommand()
        self.controller = Controller(params)

        # Subscrptions
        self.target_vel = None
        self.current_vel = None
        self.is_dbw_enabled = None
        rospy.Subscriber("/twist_cmd", TwistStamped, self.twist_cb)
        rospy.Subscriber("/current_velocity", TwistStamped, self.velocity_cb)
        rospy.Subscriber("/vehicle/dbw_enabled", Bool, self.dbw_enabled_cb)

        self.loop()

    def loop(self):
        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():
            # update controller
            if self.is_data_valid():
                self.command = self.controller.control(
                    self.current_vel, self.target_vel, self.is_dbw_enabled,
                )

            # only publish when intended
            if self.is_dbw_enabled:
                self.publish()
            rate.sleep()

    def publish(self):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = self.command.throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = self.command.steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = self.command.brake
        self.brake_pub.publish(bcmd)

    def dbw_enabled_cb(self, msg):
        self.is_dbw_enabled = msg.data

    def twist_cb(self, msg):
        linear = msg.twist.linear.x
        angular = msg.twist.angular.z
        self.target_vel = Velocity(linear, angular)

    def velocity_cb(self, msg):
        linear = msg.twist.linear.x
        angular = msg.twist.angular.z
        self.current_vel = Velocity(linear, angular)

    def is_data_valid(self):
        # TODO: dismiss outdated data
        return self.target_vel and self.current_vel and self.is_dbw_enabled


if __name__ == "__main__":
    DBWNode()
