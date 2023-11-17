# make this ROS2 fellas :) 
import rclpy
from rclpy.node import Node

import numpy as np
import time

from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

class WaypointLogger(Node): 
	home = expanduser('~')
	file = open(strftime('/sim_ws/src/pure_pursuit/waypoints/waypoints',gmtime())+'.csv', 'w')
	waypoint_time = 1 # 1 second
	
	def __init__(self):
		super().__init__('waypoints_logger')
		# TODO: create ROS subscribers and publishers
		self.odom_sub = self.create_subscription (
			Odometry,
			'/ego_racecar/odom',
			self.save_waypoint,
			10
		)

	def save_waypoint(self, data):
		quaternion = np.array([data.pose.pose.orientation.x, 
							data.pose.pose.orientation.y, 
							data.pose.pose.orientation.z, 
							data.pose.pose.orientation.w])

		euler = euler_from_quaternion(quaternion)
		speed = LA.norm(np.array([data.twist.twist.linear.x, 
								data.twist.twist.linear.y, 
								data.twist.twist.linear.z]),2)
		if data.twist.twist.linear.x >0.:
			print(data.twist.twist.linear.x)

		self.file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x, data.pose.pose.position.y, euler[2], speed))
		time.sleep(self.waypoint_time)


def main(args=None):
	rclpy.init(args=args)
	print("Waypoint Logger Initialized")

	waypoint_logger_node = WaypointLogger()
	rclpy.spin(waypoint_logger_node)

	waypoint_logger_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()