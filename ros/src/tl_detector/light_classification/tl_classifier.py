from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        # Initial state
        state = TrafficLight.UNKNOWN

        # Match pixel area
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask_image = cv2.inRange(hsv_image, np.array([150, 100, 150]), np.array([180, 255, 255]))
        extracted_image = cv2.bitwise_and(image, image, mask=mask_image)
        area = cv2.countNonZero(mask_image)

        # Check threshold
        pixels = 40

        if area > pixels:
            state = TrafficLight.RED

        # Return traffic light state - only UNKNOWN / RED

        return state
