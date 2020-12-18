#!/usr/bin/env python3

"""
    ROS component that implement a ball detector
"""

# Import of libraries
import sys
import time
import numpy as np
from scipy.ndimage import filters
import imutils
import cv2
import roslib
import rospy

from sensor_msgs.msg import CompressedImage
from sensoring.srv import DetectImage,DetectImageResponse

## Variable for logging purpose
VERBOSE = False

class image_feature:

    """
        A class used to detect a green ball

        Attributes
        -----
        @param subscriber: variable that represents a subscriber to the camera topic
        @type subscriber: Subscriber

        @param resp_center: center of the ball
        @type resp_center: int

        @param resp_radius: radius of the ball
        @type resp_radius: int

        Methods
        -----
        getCenter():
            Get the center of the ball
        getRadius()
            Get the radius of the ball
        callback(ros_data)
            Callback function of subscribed topic. 
            Here images get converted and features detected
    """

    def __init__(self):

        '''
            Constuctor. Initialize the node and the attributes, subscribe to topic of the camera
        '''

        rospy.init_node('image_detector', anonymous=True)

        ## ROS Subsriber object for getting the images  
        self.subscriber = rospy.Subscriber("/robot/camera1/image_raw/compressed",CompressedImage, self.callback,  queue_size=1)

        ## Center of the ball
        self.resp_center = -1

        ## Radius of the ball
        self.resp_radius = -1

    def getCenter(self):

        '''
            Get the center of the ball

            @returns: center of the ball
            @rtype: int
        '''

        return self.resp_center

    def getRadius(self):

        '''
            Get the radius of the ball

            @returns: radius of the ball
            @rtype: int
        '''

        return self.resp_radius

    def callback(self, ros_data):
        
        '''
            Callback function for converting the images and
            detecting the features
        '''

        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)

            self.resp_center = center[0]
            self.resp_radius = radius

        else:
            self.resp_center = -1
            self.resp_radius = -1

        cv2.imshow('window', image_np)
        cv2.waitKey(2)

class ball_info:

    """
        A class used to represent a service for providing the radius
        and center of the ball

        Attributes
        -----
        @param ic: istance of class image_feature
        @type ic: image_feature

        @param s: service object
        @type s: Service

        Methods
        -----
        handle_object(req):
            Received a request and reply with the center and radius
            of the ball        
    """

    def __init__(self):

        '''
            Constuctor. Initialize the node and service, create an instance of the class
            image_feature
        '''

        rospy.init_node('image_detector', anonymous=True)

        ## Image feature object
        self.ic = image_feature()

        ## ROS service object
        self.s = rospy.Service('detect_image', DetectImage, self.handle_object)

    def handle_object(self,req):

        """
            Received a request and reply with the center and radius
            of the ball(the request is empty)

            @returns: radius and center of the ball
            @rtype: DetectImageResponse

        """
        
        resp = DetectImageResponse()
        resp.object = str(self.ic.getCenter())+" "+str(self.ic.getRadius())
        return resp


def main(args):
    '''
        Main function.Starting the nodes
    '''
    c = ball_info()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)