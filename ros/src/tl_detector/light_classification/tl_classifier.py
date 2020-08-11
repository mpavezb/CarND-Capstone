from styx_msgs.msg import TrafficLight


class TLClassifier(object):
    def __init__(self):
        # TODO load classifier
        pass

    def get_classification(self, image):
        """
        Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        # TODO implement light color prediction

        # Obs: Only RED value is needed by the node!.
        # You can safely return UNKNOWN or GREEN.
        # TrafficLight.UNKNOWN=4
        # TrafficLight.GREEN=2
        # TrafficLight.YELLOW=1
        # TrafficLight.RED=0
        return TrafficLight.UNKNOWN
