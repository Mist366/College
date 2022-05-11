#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from ball_finder.msg import BallLocation
import time

class PID:
    def __init__(self, goal, kp, ki, kd, max1):
        self.goal = goal
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max = max1
        self.previous_error = 0
        self.integral = 0

    def get_output(self, measurement):
        error = self.goal - measurement
        self.integral = self.integral + error
        derivative = error - self.previous_error
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        output = max(output, -self.max)
        output = min(output, self.max)
        self.previous_error = error
        return output

class Robot:
    def __init__(self):
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        rospy.Subscriber('/ball_detector/ball_location', BallLocation, self.handleLocation)
        self.bearing = -1
        self.distance = -1
        self.state = "search"
        self.time = time.time()
        self.BPID = PID(320, .006, 0, .003, 1.5)
        self.DPID = PID(1.5, -.8, 0, .02, .5)
        self.slow = 0
        self.case = 0

    def handleLocation(self, msg):
        self.bearing = msg.bearing
        self.distance = msg.distance

    def run(self):
        rate = rospy.Rate(10)
        twist = Twist()
        slow = self.slow
        while not rospy.is_shutdown():
            if self.state == "search":
                print("searching")
                if slow > 0:
                    twist.linear.x = slow -.8
                else:
                    twist.linear.x = 0
                if self.case == 0:
                    twist.angular.z = .8
                if self.case == 1:
                    twist.angular.z = -.8
                if self.distance != -1 and self.bearing != -1:
                    self.state="approach"

            elif self.state == "kick":
                print("KICK!")
                twist.angular.z = 0.0
                twist.linear.x = 1.5
                if self.time < time.time():
                    self.state="search"

            elif self.state == "approach":
                print("Approaching")
                twist.angular.z = self.BPID.get_output(self.bearing)
                twist.linear.x = self.DPID.get_output(self.distance)
                self.slow = self.DPID.get_output(self.distance)
                if self.bearing < 320:
                    self.case = 0
                elif self.bearing > 320:
                    self.case = 1
                if self.distance == -1 or self.bearing == -1:
                    self.state="search"
                elif 1.4 < self.distance < 1.6:
                    self.state="kick"
                    self.time= time.time() + 1.5

        self.pub.publish(twist)
        rate.sleep()
rospy.init_node('robot_puppy')
robot = Robot()
robot.run()
