#!/usr/bin/env python

# ROS
import rospy
import tf
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# libraries
import cv2
import yaml
from scipy.spatial import KDTree

# internal
from light_classification.tl_classifier import TLClassifier

STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    """
    Provides traffic light detection based on front camera images. For the
    purpose of this project, only RED lights are considered.

    Images are processed as soon as they are received, so the detector tries to
    keep the same rate as the camera topic. If no red light detected

    Subscribes:
    - /base_waypoints (styx_msgs/Lane) Complete list of waypoints.
    - /current_pose (geometry_msgs/PoseStamped): Vehicle Location.
    - /image_color (sensor_msgs/Image): Image stream from car camera.
    - /vehicle/traffic_lights (styx_msgs/TrafficLightArray):
        Provides the location of the traffic light in 3D map space. It helps to
        acquire an accurate ground truth data source for the traffic light
        classifier, by sending the current color state of all traffic lights in
        the simulator.

        When testing on the vehicle, the color state will not be available.
        You'll need to rely on the position of the light and the camera image
        to predict it.

    Publishes:
    - /traffic_waypoint (Int32)
        Index of the waypoint which is nearest to an upcoming red light's.
    """

    def __init__(self):
        rospy.init_node("tl_detector")

        # tools
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        # internal state
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # parameters
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        # publishers
        self.upcoming_red_light_pub = rospy.Publisher(
            "/traffic_waypoint", Int32, queue_size=1
        )

        # subscribers
        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []
        rospy.Subscriber("/current_pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("/base_waypoints", Lane, self.waypoints_cb)
        rospy.Subscriber("/image_color", Image, self.image_cb)
        rospy.Subscriber("/vehicle/traffic_lights", TrafficLightArray, self.traffic_cb)

        rospy.spin()

    def pose_cb(self, msg):
        # rospy.loginfo("Received pose message in topic '/current_pose'.")
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # rospy.loginfo("Received waypoints in topic '/base_waypoints'.")
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.init_kdtree()

    def init_kdtree(self):
        self.waypoints_2d = [
            [wp.pose.pose.position.x, wp.pose.pose.position.y]
            for wp in self.waypoints.waypoints
        ]
        self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # rospy.loginfo("Received tl info in topic '/vehicle/traffic_lights'.")
        self.lights = msg.lights

    def image_cb(self, msg):
        # rospy.loginfo("Received image in topic '/image_color'.")
        self.camera_image = msg
        if self.is_data_valid():
            self.process_input_data()
        self.publish()

    def is_data_valid(self):
        return (
            self.pose
            and self.waypoints
            and self.lights
            and self.camera_image
            and self.waypoints_2d
            and self.waypoint_tree
        )

    def publish(self):
        rospy.loginfo("Publishing tl '%d' in topic '/traffic_waypoint'." % self.last_wp)
        self.upcoming_red_light_pub.publish(Int32(self.last_wp))

    def process_input_data(self):
        """
        Publish upcoming red lights. Each predicted state has to occur
        `STATE_COUNT_THRESHOLD` number of times till we start using it.
        Otherwise the previous stable state is used.
        """
        light_wp, state = self.get_traffic_light_information()

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
        self.state_count += 1

    def get_traffic_light_information(self):
        """
        Finds closest visible traffic light, if one exists, and determines its
        location and color.

        Returns:
            int: index of waypoint closes to the upcoming stop line for a
                 traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for
        # a given intersection
        stop_line_positions = self.config["stop_line_positions"]

        if self.pose:
            car_x = self.pose.pose.position.x
            car_y = self.pose.pose.position.y
            car_wp_idx = self.get_closest_waypoint(car_x, car_y)

            # find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)  # is this correct??
            for i, light in enumerate(self.lights):
                # get stop line waypoint index
                line = stop_line_positions[i]
                line_x = line[0]
                line_y = line[1]
                temp_wp_idx = self.get_closest_waypoint(line_x, line_y)

                # find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx

        if closest_light:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state

        return -1, TrafficLight.UNKNOWN

    def get_closest_waypoint(self, x, y):
        """
        Identifies the closest path waypoint to the given position.
        Returns index of the closest waypoint in self.waypoints.
        """
        return self.waypoint_tree.query([x, y], 1)[1]

    def get_light_state(self, light):
        """
        Determines the current color of the traffic light.
        Returns ID of traffic light color.
        """
        # for testing
        return light.state

        # if not self.camera_image:
        #     self.prev_light_loc = None
        #     return False

        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # # Get classification
        # return self.light_classifier.get_classification(cv_image)


if __name__ == "__main__":
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start traffic node.")
