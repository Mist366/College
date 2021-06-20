#!/usr/bin/env python

from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ball_finder.msg import BallLocation
import rospy
import roslib
import time
import math

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
        rospy.Subscriber('/odom', Odometry, self.handle_pose)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        rospy.Subscriber('/ball_detector/ball_location', BallLocation, self.handleLocation)
        self.bearing = -1
        self.distance = -1
        self.state = "search"
        self.time = time.time()
        self.BPID = PID(320, .005, 0, .003, 1)
        self.DPID = PID(1.5, -1, -.2, -.3, .5)
        self.BGPID = PID(0, -1.25, 0, -.06, 2)
        self.DGPID = PID(0, -.4, 0, -.1, .5)
        self.goalX = 0
        self.goalY = 0
        self.kickX = 0
        self.kickY = 0
        self.slow = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.case = 0

    def handleLocation(self, msg):
        self.bearing = msg.bearing
        self.distance = msg.distance

    def handle_pose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        (_, _, self.theta) = euler_from_quaternion(q)

    def get_vector(self, from_x, from_y, to_x, to_y):
        bearing = math.atan2(to_y - from_y, to_x - from_x)
        distance = math.sqrt((from_x - to_x)**2 + (from_y - to_y)**2)
        return (bearing, distance)

    def run(self):
        rate = rospy.Rate(10)
        twist = Twist()
        slow = self.slow
        while not rospy.is_shutdown():
            if self.state == "search":
                print("Searching")
                if slow > 0:
                     twist.linear.x = slow -.4
                else:
                    twist.linear.x = 0
                if self.case == 0:
                    twist.angular.z = 1
                elif self.case == 1:
                    twist.angular.z = -1
                if self.distance != -1 and self.bearing != -1:
                    self.state="approach"
                    if self.distance > 1.5:
                        twist.linear.x = .25
                    if self.distance < 1.5:
                        twist.linear.x = -.25

            elif self.state == "approach":
                print("Approaching")
                twist.angular.z = self.BPID.get_output(self.bearing)
                print(self.distance)
                twist.linear.x = self.DPID.get_output(self.distance)
                self.slow = self.DPID.get_output(self.distance)

                #what side was the ball on last?
                if 0 < self.bearing < 320:
                    self.case = 0
                elif 320 < self.bearing < 640:
                    self.case = 1
                if self.distance == -1 or self.bearing == -1:
                    self.state="search"
                elif 1.4 < self.distance < 1.6:
                    thetaG = (math.pi/4)+self.theta
                    h = abs(self.distance) * math.sqrt(2)
                    if thetaG <= -math.pi:
                        thetaG = (7*math.pi/4) - abs(self.theta)
                    self.kickX = (2*abs(self.distance)*math.cos(self.theta))+self.x
                    self.kickY = (2*abs(self.distance)*math.sin(self.theta))+self.y
                    self.goalX = (h*math.cos(thetaG))+self.x
                    self.goalY = (h*math.sin(thetaG))+self.y
                    self.state="toGoal"

            elif self.state == "toGoal" :
                print("Moving to side Goal")
                bear, dist = self.get_vector(self.x, self.y, self.goalX, self.goalY)

                #determine if bear - theta is between pi and -pi and correct accordingly
                phi = bear - self.theta
                if phi > math.pi :
                    phi = phi - (2*math.pi)
                elif phi < -math.pi :
                    phi = phi + (2*math.pi)

                #use PID to get correct movement
                twist.angular.z = self.BGPID.get_output(phi)
                print(phi)
                twist.linear.x = self.DGPID.get_output(dist)
                print(self.DGPID.previous_error)
                print(self.BGPID.previous_error)
                if abs(self.DGPID.previous_error) <= .08 and abs(self.BGPID.previous_error) <= .2:
                    self.state = "toKick"

            elif self.state == "toKick" :
                print("Moving to Kick Position")
                bear, dist = self.get_vector(self.x, self.y, self.kickX, self.kickY)

                #determine if bear - theta is between pi and -pi and correct accordingly
                phi = bear - self.theta
                if phi > math.pi :
                    phi = phi - (2*math.pi)
                elif phi < -math.pi :
                    phi = phi + (2*math.pi)

                #use PID to get correct movement
                twist.angular.z = self.BGPID.get_output(phi)
                print(phi)
                twist.linear.x = self.DGPID.get_output(dist)
                if abs(self.DGPID.previous_error) <= .08 and abs(self.BGPID.previous_error) <= .2:
                    self.state = "lineUp"

            elif self.state == "lineUp" :
                print("Lining Up")
                twist.linear.x = 0
                print(self.BPID.get_output(self.bearing))
                if self.bearing != -1 and self.distance != -1:
                    twist.angular.z = self.BPID.get_output(self.bearing)
                else:
                    twist.angular.z = -1

                    if -.19 < self.BPID.get_output(self.bearing) < .19:
                        self.state="kick"
                        self.time= time.time() + 1.5

            elif self.state == "kick":
                print("KICK!")
                twist.angular.z = 0.0
                twist.linear.x = 1.5
                if self.time < time.time():
                    self.state="search"

            self.pub.publish(twist)
            rate.sleep()
rospy.init_node('dodgeball')
robot = Robot()
robot.run()
