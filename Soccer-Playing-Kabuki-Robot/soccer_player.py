#!/usr/bin/env python

from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ball_finder.msg import BallLocation
from kobuki_msgs.msg import BumperEvent
import rospy
import roslib
import time
import math
import tf2_ros

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
        #Subscribers
        rospy.Subscriber('/odom', Odometry, self.handle_pose)
        rospy.Subscriber('/ball_detector/ball_location', BallLocation, self.handleLocation)
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumped)
        #Publishers
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        #PIDs
        self.BPID = PID(320, .005, 0, .003, 1)
        self.DPID = PID(1.3, -1, -.2, -.3, .5)
        self.BGPID = PID(0, -1.25, 0, -.06, 2)
        self.DGPID = PID(0, -.4, 0, -.1, .5)
        #self.DRPID = PID(1, -.4, 0, -.1, .3)
        #Set Variables
        self.kick = .75
        self.int = .75
        #Variables
        self.time = self.stoptime = self.retime = time.time()
        self.bearing = self.distance = -1
        self.tmp0X = self.tmp0Y = self.tmp1X = self.tmp1Y = self.interX = self.interY = 0
        self.goalX = self.goalY = self.kickX = self.kickY = self.goalToBall = 0
        self.x = self.y = self.theta = self.goal_x = self.goal_y = self.ballX = self.ballY = 0
        self.slow = self.case = 0
        self.state = "locateGoal"
        self.last_state = "search"
        self.blind = "no"
        #Other
        self.listener = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.listener)

    def bumped(self, msg):
        if msg.state == 1:
            print("BUMPED")
            if self.state != "backup":
                self.last_state = self.state
            if self.last_state == "kick":
                self.state = self.last_state
            else:
                self.state = "backup"
            self.time = time.time() + 1.5

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
        self.retime += 60

        while not rospy.is_shutdown():
            # if time.time() > self.retime:
            #     self.state = "reGoal"
            #     self.retime += 60
            #Locate Ball
            if self.distance != -1 and self.bearing != -1:
                if self.distance < .7 and self.blind == "no":
                    self.stoptime = time.time() + 2
                    self.blind = "yes"
                if self.blind == "yes" and self.stoptime < time.time():
                    blind = "no"
                if self.blind == "no":
                    self.ballX = self.distance*math.cos(self.theta)+self.x
                    self.ballY = self.distance*math.sin(self.theta)+self.y

            #Locate AR tag
            try:
                #trans = self.listener.lookup_transform('odom', 'ar_marker_1', rospy.Time())
                trans = self.listener.lookup_transform('odom', 'ar_marker_0', rospy.Time())
                self.goal_x = trans.transform.translation.x
                self.goal_y = trans.transform.translation.y
            except tf2_ros.LookupException:
                pass
            except tf2_ros.ConnectivityException:
                pass
            except tf2_ros.ExtrapolationException:
                pass

            #what side was the ball on last?
            if 0 < self.bearing < 320:
                self.case = 0
            elif 320 < self.bearing < 640:
                self.case = 1

            print("Goal: " + str(self.goal_x) + "," + str(self.goal_y) + " Ball: " + str(self.ballX) + "," + str(self.ballY) + " Kick: " + str(self.kickX) + "," + str(self.kickY))

            #Initial State to find AR tag
            if self.state == "locateGoal":
                print("Searching for Goal")
                if self.goal_x == 0 or self.goal_y == 0:
                    twist.linear.x = 0
                    twist.angular.z = 0
                else:
                    print("Goal Found")
                    self.state = "search"

            #Start of States
            if self.state == "search":
                print("Searching for Ball")
                if slow > 0:
                     twist.linear.x = slow -.4
                else:
                    twist.linear.x = 0
                if self.case == 0:
                    twist.angular.z = 1
                elif self.case == 1:
                    twist.angular.z = -1
                if self.distance != -1 and self.bearing != -1:
                    self.ballX = self.distance*math.cos(self.theta)+self.x
                    self.ballY = self.distance*math.sin(self.theta)+self.y
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
                slow = self.DPID.get_output(self.distance)
                if self.distance == -1 or self.bearing == -1:
                    self.state="search"
                elif 1.1 < self.distance < 1.5:
                    angle, length = self.get_vector(self.ballX, self.ballY, self.goal_x, self.goal_y)
                    eh, length0 = self.get_vector(self.x, self.y, self.goal_x, self.goal_y)
                    #Calculate Kick position
                    eh, distance = self.get_vector(self.goal_x, self.goal_y, self.ballX, self.ballY)
                    self.kickX = self.goal_x+((distance+self.kick)*(self.ballX-self.goal_x)/distance)
                    self.kickY = self.goal_y+((distance+self.kick)*(self.ballY-self.goal_y)/distance)
                    if length < length0:
                        self.state = "toKick"
                    else:
                        #Calculate inter goal
                        self.tmp0X = self.goal_x + math.sqrt(length**2+self.int**2)*math.cos((math.atan(self.int/length)+(angle+math.pi)))
                        self.tmp0Y = self.goal_y + math.sqrt(length**2+self.int**2)*math.sin((math.atan(self.int/length)+(angle+math.pi)))
                        self.tmp1X = self.goal_x + math.sqrt(length**2+self.int**2)*math.cos((math.atan(-self.int/length)+(angle+math.pi)))
                        self.tmp1Y = self.goal_y + math.sqrt(length**2+self.int**2)*math.sin((math.atan(-self.int/length)+(angle+math.pi)))
                        eh, dist0 = self.get_vector(self.x, self.y, self.tmp0X, self.tmp0Y)
                        eh, dist1 = self.get_vector(self.x, self.y, self.tmp1X, self.tmp1Y)
                        if dist0 < dist1:
                            self.interX = self.tmp0X
                            self.interY = self.tmp0Y
                        else:
                            self.interX = self.tmp1X
                            self.interY = self.tmp1Y
                        self.state="toInter"

            elif self.state == "toInter" :
                print("Moving to side Goal")
                bear, dist = self.get_vector(self.x, self.y, self.interX, self.interY)

                #determine if bear - theta is between pi and -pi and correct accordingly
                phi = bear - self.theta
                if phi > math.pi :
                    phi = phi - (2*math.pi)
                elif phi < -math.pi :
                    phi = phi + (2*math.pi)

                #use PID to get correct movement
                print(dist)
                twist.angular.z = self.BGPID.get_output(phi)
                twist.linear.x = self.DGPID.get_output(dist)
                if abs(self.DGPID.previous_error) <= .5:
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
                print(dist)
                twist.linear.x = self.DGPID.get_output(dist)
                if abs(self.DGPID.previous_error) <= .3:
                    self.state = "lineUp"

            elif self.state == "lineUp" :
                print("Lining Up")
                twist.linear.x = 0
                print(self.BPID.get_output(self.bearing))

                if self.bearing != -1 and self.distance != -1:
                    twist.angular.z = self.BPID.get_output(phi)
                else:
                    if self.case == 0:
                        twist.angular.z = 1.5
                    elif self.case == 1:
                        twist.angular.z = -1.5
                if 200 <= self.bearing <= 440:
                    self.state="kick"
                    twist.angular.z = 0
                    self.time= time.time() + 2

            elif self.state == "kick":
                print("KICKING: " + str(self.time-time.time()))
                print(self.time-time.time())
                twist.angular.z = self.BPID.get_output(self.bearing)
                twist.linear.x = 1.5
                if self.time < time.time():
                     self.state="search"

            elif self.state == "backup":
                print("Backing Up: " + str(self.time-time.time()))
                twist.angular.z = 0
                twist.linear.x = -.3
                if self.time < time.time():
                    self.state = "search"

            # elif self.state == "reGoal":
            #     print("Reseting Goal")
            #     bear, dist = self.get_vector(self.x, self.y, self.goal_x, self.goal_y)
            #
            #     #determine if bear - theta is between pi and -pi and correct accordingly
            #     phi = bear - self.theta
            #     if phi > math.pi :
            #         phi = phi - (2*math.pi)
            #     elif phi < -math.pi :
            #         phi = phi + (2*math.pi)
            #
            #     #use PID to get correct movement
            #     twist.angular.z = self.BGPID.get_output(phi)
            #     twist.linear.x = self.DRPID.get_output(dist)
            #     if .8 < self.distance < 1.2:
            #         self.state = "search"

            self.pub.publish(twist)
            rate.sleep()
rospy.init_node('soccer_player')
robot = Robot()
robot.run()
