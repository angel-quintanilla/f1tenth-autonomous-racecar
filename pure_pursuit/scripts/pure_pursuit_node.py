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

	STRAIGHT_AHEAD_SPEED = 2.0
	STRAIGHT_AHEAD_THRESHOLD = 10 * np.pi / 180
	WIDE_TURN_SPEED = 1.0
	WIDE_TURN_THRESHOLD = 20 * np.pi / 180
	SHARP_TURN_SPEED = 0.75
	CURRENT_SPEED = 0.0
	PREV_POS_X = 0.0
	PREV_POS_Y = 0.0
	
	STEERING_ANGLE = 0.0
	KP = 1.0
	
	def __init__(self):
		super().__init__('pure_pursuit_node')

		# self.odom_sub = self.create_subscription(
		#     PoseStamped,
		#     '/pf/viz/inferred_pose',
		#     self.pose_callback,
		#     10
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

		# Read waypoints from csv file
		data = np.loadtxt(self.IN_FILE, delimiter=',', skiprows=1)

		x_values = data[:, 0]  # x array
		y_values = data[:, 1]  # y array

		# Set precision in printing
		np.set_printoptions(precision=3, suppress=True)

		# Make 2D Array
		self.WAYPOINTS = np.column_stack((x_values, y_values))

	def pose_callback(self, pose_msg):
		# TODO: find the current waypoint to track using methods mentioned in lecture
		car_position = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]
		car_orientation = np.arctan2(pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w)
		car_rotation = (car_orientation * 180 / np.pi) * 2  # Convert to degrees
		
		closest_waypoint = self.get_waypoints(car_position, car_orientation)

		# TODO: transform goal point to vehicle frame of reference
		closest_waypoint_vehicle_frame = closest_waypoint - car_position

		# TODO: calculate curvature/steering angle
		rel_x = closest_waypoint_vehicle_frame[0]
		rel_y = closest_waypoint_vehicle_frame[1]

		# Calculate the rotated coordinates
		rotation_matrix = np.array([
			[np.cos(car_orientation), np.sin(car_orientation)],
			[-np.sin(car_orientation), np.cos(car_orientation)]
		])

		rotated_coordinates = np.dot(rotation_matrix, np.array([rel_x, rel_y]))
		rotated_rel_x, rotated_rel_y = rotated_coordinates

		L = math.sqrt(rotated_rel_y**2 + rotated_rel_x**2)

		radius = (L ** 2) / (2.0 * rotated_rel_y)  # L is lookahead distance, rel_y is y value from vehicle frame of reference
		self.STEERING_ANGLE = 1 / radius

		# TODO: publish drive message, don't forget to limit the steering angle.
		print("Euclidean distance:", L)
		print("rel_x:", rel_x)
		print("rel_y:", rel_y)
		print("rotated_rel_x", rotated_rel_x)
		print("rotated_rel_y", rotated_rel_y)
		print("Car Rotation:", car_rotation)
		print("Steering Angle:", self.STEERING_ANGLE * 180 / np.pi)
		
		# Clamp the steering angle
		self.STEERING_ANGLE = max(-20 * np.pi / 180, min(self.STEERING_ANGLE, 20 * np.pi / 180))

		# Dynamic speed
		# if (np.abs(self.STEERING_ANGLE) < self.STRAIGHT_AHEAD_THRESHOLD):  # < 10 degrees speed
		#     self.CURRENT_SPEED = self.STRAIGHT_AHEAD_SPEED  # 0-10 degrees speed
		# elif np.abs(self.STEERING_ANGLE) < self.WIDE_TURN_THRESHOLD:  # < 20 degrees speed
		#     self.CURRENT_SPEED = self.WIDE_TURN_SPEED  # 10-20 degrees speed
		# else:  # Anything that's not < 20
		#     self.CURRENT_SPEED = self.SHARP_TURN_SPEED  # > 20 degrees speed

		self.CURRENT_SPEED = 2.0

		# self.publish_drive(pose_msg)

		# Publish markers for visual waypoint validation
		self.publish_marker(closest_waypoint, (0.0, 0.0, 1.0))

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

	def publish_drive(self, pose_msg):
		drive_msg = AckermannDriveStamped()
		drive_msg.header = pose_msg.header
		drive_msg.drive.steering_angle = self.STEERING_ANGLE
		drive_msg.drive.speed = self.CURRENT_SPEED
		self.drive_pub.publish(drive_msg)

	def get_waypoints(self, position, orientation):
		# Euclidean distance between all current position and all waypoints
		distance_to_all_waypoints = np.sqrt(np.sum((self.WAYPOINTS - position)**2, axis=1))

		# Get the index of the smallest distance
		closest_waypoint_index = np.argmin(distance_to_all_waypoints)

		# If the waypoint at this index is behind the vehicle
		waypoint_to_vehicle = self.WAYPOINTS[closest_waypoint_index] - position
		
		relative_angle = np.arctan2(waypoint_to_vehicle[1], waypoint_to_vehicle[0]) - orientation

		# Do dot multiplication on axis x, negative val = behind, 0 = side
		# !!! This isn't perfectly working
		if (np.dot(waypoint_to_vehicle, np.array([1.0, 0.0])) <= 0 or np.abs(relative_angle) > np.pi / 2):
			closest_waypoint_index += 1  # Go to the next waypoint

		# When the vehicle is too close to the waypoint, choose next waypoint
		if (np.linalg.norm(waypoint_to_vehicle) < 0.6):
			closest_waypoint_index += 1  # Go to the next waypoint
		
		# Makes sure that if last waypoint, gets to set first
		closest_waypoint_index %= len(self.WAYPOINTS)  
		return self.WAYPOINTS[closest_waypoint_index]

def main(args=None):
	rclpy.init(args=args)
	print("PurePursuit Initialized")
	
	pure_pursuit_node = PurePursuit()
	rclpy.spin(pure_pursuit_node)

	pure_pursuit_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
