#!/usr/bin/env python
import roslib
roslib.load_manifest('ball_finder')
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from ball_finder.msg import BallLocation
from cv_bridge import CvBridge, CvBridgeError
class Detector:
    def __init__(self):
        # The image publisher is for debugging and figuring out
        # good color values to use for ball detection
        self.impub = rospy.Publisher('/ball_detector/image', Image, queue_size=1)
        self.locpub = rospy.Publisher('/ball_detector/ball_location', BallLocation, queue_size=1)
        self.bridge = CvBridge()
        self.bearing = -1
        self.distance = -1
	    self.average = -1
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.handle_image)
        rospy.Subscriber('/scan', LaserScan, self.handle_scan)

    def handle_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print e

        self.count = 0
        self.rowCount = 0
        self.bearing = -1
        for rows in range(100,400,5) :
            for cols in range(0,639,5):
                if(40 < image[rows,cols,0] <80 and 180 < image[rows,cols,1] < 210 and 180 < image[rows,cols,2] < 220):
                    self.count += 1
                    self.rowCount += cols
                    image[rows,cols,:] = 0

        if self.count != 0:
            self.average = self.rowCount / self.count
        print(self.average)
        # Find the average column of the bright yellow pixels
        # and store as self.bearing. Store -1 if there are no
        # bright yellow pixels in the image.
        if self.count > 0:
            self.bearing = self.average
        else:
            self.bearing = -1
        # Feel free to change the values in the image variable
        # in order to see what is going on
        # Here we publish the modified image; it can be
        # examined by running image_view
        self.impub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
    def handle_scan(self, msg):
        # If the bearing is valid, store the corresponding range
        # in self.distance. Decide what to do if range is NaN.
        if self.bearing > 0:
	    self.distance = msg.ranges[-self.bearing]
            if np.isnan(self.distance) or self.bearing < 0:
                self.distance = -1
            else:
                print("BALL")



    def start(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            location = BallLocation()
            location.bearing = self.bearing
            location.distance = self.distance
            self.locpub.publish(location)
            rate.sleep()

rospy.init_node('ball_detector')
detector = Detector()
detector.start()
