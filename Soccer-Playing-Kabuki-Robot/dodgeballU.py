#!/usr/bin/env python

import roslib
roslib.load_manifest('beginner_tutorials')
import rospy
import math
from beginner_tutorials.msg import BallLocation
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class PID:

	#Initialize the PID, with the Goal being the intended location on the image
	#kp being the proportional error, ki being the integral error, kd being the derivative error
	def __init__(self, goal, kp, ki, kd, max1):
		self.goal = goal
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.max1 = max1
		self.previous_error = 0
		self.integral = 0

	#does the math to see what we actually need to return in order to correctly correct the motion
	#that we want it to do so it is smoother
	def get_output(self, measurement):
		error = self.goal - measurement
		self.integral = self.integral + error
		self.derivative = error - self.previous_error
		output = self.kp*error + self.ki*self.integral + self.kd*self.derivative
		output = max(output, -self.max1)
		output = min(output, self.max1)
		self.previous_error = error
		return output

class Dodgeball:

	#subscribing to the msgs our ball_detector program publishes and then says that we are gonna publish
	#twist messages to the velocity topic so we can move the robot around
	#Also, initializes all of the variables we need to keep track of to 0
	def __init__(self):
		rospy.Subscriber('/ball_detector/ball_location', BallLocation, self.location)
		rospy.Subscriber('/odom', Odometry, self.handle_pose)
		self.velopub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
		self.state = 'search'
		self.bearing = 0
		self.distance = 0
		self.distancePID = PID(1.5, -1, -.2, -.4, .5)
		self.bearingPID = PID(320, 0.005, 0, 0.003, 1)
		self.rangePID = PID(0, -.4, 0, -.1, .5)
		self.thetaPID = PID(0, -2.5, 0, -.1, 1)
		self.timing = 0
		self.midgoalX = 0
		self.midgoalY = 0
		self.goalX = 0
		self.goalY = 0
		self.timing = 0

	def handle_pose(self, msg):
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
		(_, _, self.theta) = euler_from_quaternion(q)


	#sets the location and distance of each run through to the constantly updating messages
	#that are getting published to distance and bearing from ball_detector
	#Also, if the distance and bearing are not 0 (off of the screen) then go find it
	def location(self, msg):
		self.distance = msg.distance
		self.bearing = msg.bearing
		if self.bearing !=-1 and self.distance != -1 and self.state == 'search':
			self.state = 'approach'

	def get_vectorD(self, from_x, from_y, to_x, to_y):
		return math.sqrt((from_x - to_x)**2 + (from_y - to_y)**2)

	def get_vectorT(self, from_x, from_y, to_x, to_y):
		return math.atan2(to_y - from_y, to_x - from_x)

	def start(self):
		rate = rospy.Rate(10)
		twist = Twist()
		while not rospy.is_shutdown():
			if self.state == 'search':		#If we are searching, turn around in circles
				print("Searching")
				twist.angular.z = 1
				twist.linear.x = 0
			elif self.state == 'approach':		#If we see the ball, go forward w/ changes to the PID going through
				if self.bearing == -1:
					self.state = 'search'
				print("Approaching")
				twist.angular.z = self.bearingPID.get_output(self.bearing)
				twist.linear.x = self.distancePID.get_output(self.distance)
				if 1.4 < self.distance < 1.7 and self.distance != -1:
					#self.iX = self.x
					#self.iY = self.y
					self.iTheta = self.theta + 2*math.pi
					self.midgoalX = 1.5*math.sqrt(2)*math.cos(self.iTheta + math.pi/4) + self.x
					self.midgoalY = 1.5*math.sqrt(2)*math.sin(self.iTheta + math.pi/4) + self.y
					self.goalX = 3*math.sqrt(2)*math.cos(self.theta) + self.x
					self.goalY = 3*math.sqrt(2)*math.sin(self.theta) + self.y
					self.state = 'midpoint'
			elif self.state == 'midpoint':
				distanceTO = self.get_vectorD(self.x, self.y, self.midgoalX, self.midgoalY)
				angleTO = self.get_vectorT(self.x, self.y, self.midgoalX, self.midgoalY)
				print("Going to midpoint")
				print(self.x)
				print(self.y)
				print(angleTO)
				twist.angular.z = self.thetaPID.get_output(abs(angleTO) - abs(self.theta))
				twist.linear.x = self.rangePID.get_output(distanceTO)
				if 0 < abs(distanceTO) < 0.5:
					self.state = 'toGoal'
			elif self.state == 'toGoal':
				distanceTO = self.get_vectorD(self.x, self.y, self.goalX, self.goalY)
				angleTO = self.get_vectorT(self.x, self.y, self.goalX, self.goalY) % math.pi
				print("Going for goal!!!!")
				print(self.x)
				print(self.y)
				print(angleTO)
				twist.angular.z = self.thetaPID.get_output(angleTO - self.theta)
				twist.linear.x = self.rangePID.get_output(distanceTO)
				if 0 < abs(distanceTO) < 0.5:
					self.state = 'lineup'
			elif self.state == 'lineup':
				print("Calibrating, I guess...")
				twist.linear.x = 0
				twist.angular.z = self.bearingPID.get_output(self.bearing)
				if 310 < self.bearing < 330:
					self.timing = rospy.get_time()
					self.state = 'kick'
			elif self.state == 'kick':
				print("Kicking")
				twist.angular.z = 0
				twist.linear.x = 1.5
				if rospy.get_time() >= self.timing +3:
					self.state = 'search'
			self.velopub.publish(twist)
			rate.sleep()

rospy.init_node('dodgeballU')
dodgeball = Dodgeball()
dodgeball.start()
