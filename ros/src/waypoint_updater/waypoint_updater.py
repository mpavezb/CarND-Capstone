#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
from std_msgs.msg import Int32

import numpy as np
import math
from scipy.spatial import KDTree

"""
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
"""

LOOKAHEAD_WPS = 200  # Number of waypoints to publish.
FREQUENCY = 30  # Execution frequency


class WaypointUpdater(object):
    def __init__(self):
        self.base_waypoints = None
        self.pose = None
        self.traffic = None
        self.obstacles = None
        self.traffic_sim = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        rospy.init_node("waypoint_updater")
        rospy.Subscriber("/current_pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("/base_waypoints", Lane, self.waypoints_cb)
        # rospy.Subscriber("/traffic_waypoint", Int32, self.traffic_cb)
        # rospy.Subscriber("/obstacle_waypoint", Int32, self.obstacle_cb)
        # rospy.Subscriber("/vehicle/traffic_lights", TrafficLightArray, self.traffic_sim_cb)
        self.final_waypoints_pub = rospy.Publisher(
            "final_waypoints", Lane, queue_size=1
        )

        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()

    def step(self):
        if self.is_data_available():
            closest_id = self.get_closest_waypoint_id()
            self.publish_waypoints(closest_id)

    def publish_waypoints(self, closest_id):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[
            closest_id : closest_id + LOOKAHEAD_WPS
        ]
        self.final_waypoints_pub.publish(lane)

    def get_closest_waypoint_id(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coord
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        self.base_waypoints = msg
        if not self.waypoints_2d:
            self.init_kdtree()

    def traffic_cb(self, msg):
        self.traffic = msg

    def obstacle_cb(self, msg):
        self.obstacles = msg

    def traffic_sim_cb(self, msg):
        self.traffic_sim = msg

    def init_kdtree(self):
        self.waypoints_2d = [
            [wp.pose.pose.position.x, wp.pose.pose.position.y]
            for wp in self.base_waypoints.waypoints
        ]
        self.waypoint_tree = KDTree(self.waypoints_2d)

    def is_data_available(self):
        return self.base_waypoints and self.waypoint_tree and self.pose

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0

        def dist(a, b):
            return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

        for i in range(wp1, wp2 + 1):
            dist += dist(
                waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position
            )
            wp1 = i
        return dist


if __name__ == "__main__":
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start waypoint updater node.")
