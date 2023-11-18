#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

import numpy as np
import math


class PurePursuit(Node):
	IN_FILE = '/sim_ws/src/pure_pursuit/waypoints/waypoints.csv'
	WAYPOINTS = []

	STRAIGHT_AHEAD_SPEED = 4.0
	STRAIGHT_AHEAD_THRESHOLD = 10 * np.pi / 180
	WIDE_TURN_SPEED = 2.0
	WIDE_TURN_THRESHOLD = 20 * np.pi / 180
	SHARP_TURN_SPEED = 0.75
	VELOCITY = 0.0
	WHEELBASE = 0.3

	a_angle = 40 * np.pi/180      # tunable angle
	b_angle = 90 * np.pi/180      # perpendicular angle

	def __init__(self):
		super().__init__('pure_pursuit_node')

		# self.odom_sub = self.create_subscription(
		# 	PoseStamped,
		# 	'/pf/viz/inferred_pose',
		# 	self.pose_callback,
		# 	10
		# )

		self.odom_sub = self.create_subscription (
			Odometry,
			'/ego_racecar/odom',
			self.pose_callback,
			10
		)

		self.drive_pub = self.create_publisher(
			AckermannDriveStamped,
			'/drive',
			10
		)

		self.marker_pub_current = self.create_publisher(
			MarkerArray,
			'visualization_marker_current',
			1
		)

		self.marker_pub_lookahead = self.create_publisher(
			MarkerArray,
			'visualization_marker_lookahead',
			1
		)

		# Read waypoints from csv file
		data = np.loadtxt(self.IN_FILE, delimiter=',', skiprows=1)

		x_values = data[:, 0]  # x
		y_values = data[:, 1]  # y
		rotation_values = data[:, 2]  # rotation

		# Set precision in printing
		np.set_printoptions(precision=3, suppress=True)

		# Make 2D Array
		self.WAYPOINTS = np.column_stack((x_values, y_values))

	def pose_callback(self, pose_msg):
		# TODO: find the current waypoint to track using methods mentioned in lecture
		# Step 1: Find current waypoint
		# Step 2: Go towards way point towards car angle pointed to it.
		# Step 3: Repeat\
		current_position = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])

		# Calculate the vehicle's heading vector
		orientation = pose_msg.pose.pose.orientation
		_, _, yaw = self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
		heading_vector = np.array([np.cos(yaw), np.sin(yaw)])

		# Calculate vectors from the vehicle to each waypoint
		waypoint_vectors = self.WAYPOINTS[:, :2] - current_position[:2]

		# Calculate dot products between waypoint vectors and vehicle's heading vector
		dot_products = np.dot(waypoint_vectors, heading_vector)

		# Filter out waypoints that are behind the vehicle
		valid_indices = dot_products >= 0
		valid_waypoints = self.WAYPOINTS[valid_indices]

		if len(valid_waypoints) == 0:
			# If no valid waypoints, stop the vehicle or take appropriate action
			return

		# Find the index of the closest waypoint among valid waypoints
		distances = np.linalg.norm(valid_waypoints[:, :2] - current_position[:2], axis=1)
		closest_waypoint_index = np.argmin(distances)

		# Get the coordinates of the closest waypoint
		closest_waypoint = valid_waypoints[closest_waypoint_index]

		print(current_position)
		print(closest_waypoint)

		# TODO: transform goal point to vehicle frame of reference
		# ???
		print("Closest Waypoint[0]", closest_waypoint[0])
		print("Closest Waypoint[1]", closest_waypoint[1])
		print("Current Position[0]", current_position[0])
		print("Current Position[1]", current_position[1])
		rel_x = closest_waypoint[0] - current_position[0] # x
		rel_y = closest_waypoint[1] - current_position[1] # y
		L = math.sqrt(rel_y**2 + rel_x**2)  # euclidean dist

		print("Euclidean distance:", L)
		print("rel_x:", rel_x)
		print("rel_y:", rel_y)

		# TODO: calculate curvature/steering angle
		# y = 2|y| / L^2  <- Formula for steering angle
		# L = Lookahead distance
		# Y = y length on axis
		radius = (L ** 2) / (2.0 * np.abs(rel_y))
		steering_angle = math.atan((self.WHEELBASE*(1/radius))/self.VELOCITY)
		print("Steering Angle:", steering_angle * 180/np.pi)
		if (rel_y < 0):
			steering_angle *= -1.0


		desired_yaw = np.arctan2(closest_waypoint[1] - current_position[1], closest_waypoint[0] - current_position[0])
		error = desired_yaw - yaw
		Kp = 1.0
		proportional_control = Kp * error

		steering_angle += proportional_control

		distance_to_waypoint = np.linalg.norm(closest_waypoint - current_position[:2])
		if distance_to_waypoint < 0.5:  # You can adjust this threshold as needed
			steering_angle = 0.0

		# TODO: publish drive message, don't forget to limit the steering angle.
		# clamp
		if(steering_angle < self.WIDE_TURN_THRESHOLD * -1):  # if <-20, set to -20
			steering_angle = self.WIDE_TURN_THRESHOLD * -1
		elif steering_angle > self.WIDE_TURN_THRESHOLD:  # if >20, set to 20
			steering_angle = self.WIDE_TURN_THRESHOLD

		if(np.abs(steering_angle) < self.STRAIGHT_AHEAD_THRESHOLD): # < 10 degree speed
			self.VELOCITY = self.STRAIGHT_AHEAD_SPEED # 0-10 degree speed
		elif np.abs(steering_angle) < self.WIDE_TURN_THRESHOLD: # < 20 degree speed
			self.VELOCITY = self.WIDE_TURN_SPEED # 10-20 degree speed
		else: # anything thats not < 20
			self.VELOCITY = self.SHARP_TURN_SPEED # > 20 degree speed

		# self.VELOCITY = 2.0

		drive_msg = AckermannDriveStamped()
		drive_msg.header = pose_msg.header
		drive_msg.drive.steering_angle = steering_angle
		drive_msg.drive.speed = self.VELOCITY
		self.drive_pub.publish(drive_msg)

		# Publish markers
		self.publish_marker(closest_waypoint, (0.0, 1.0, 0.0))

	def euler_from_quaternion(self, x, y, z, w):
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		roll_x = np.arctan2(t0, t1)

		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch_y = np.arcsin(t2)

		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		yaw_z = np.arctan2(t3, t4)

		return roll_x, pitch_y, yaw_z

	def publish_marker(self, position, color):
		marker = Marker()
		marker.header.frame_id = 'map'
		marker.type = Marker.SPHERE
		marker.action = Marker.ADD
		marker.pose.position.x = position[0]
		marker.pose.position.y = position[1]
		marker.pose.position.z = 0.0
		marker.pose.orientation.w = 1.0
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = 1.0
		marker.color.r = color[0]
		marker.color.g = color[1]
		marker.color.b = color[2]

		self.marker_pub_current.publish(MarkerArray(markers=[marker]))

def main(args=None):
	rclpy.init(args=args)
	print("PurePursuit Initialized")
	
	pure_pursuit_node = PurePursuit()
	rclpy.spin(pure_pursuit_node)

	pure_pursuit_node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
