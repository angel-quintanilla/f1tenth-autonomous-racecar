#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        self.ebrake = False
        # TODO: create ROS subscribers and publishers.
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.odom_sub = self.create_subscription (
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        self.laser_sub = self.create_subscription (
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        # Created vector with the linear twist, set speed to the first value
        VelocityVect = odom_msg.twist.twist.linear
        self.speed = VelocityVect.x

    def scan_callback(self, scan_msg):
        for x in range(len(scan_msg.ranges)):
        # TODO: calculate TTC
            # x is an index
            currentAngle = x * scan_msg.angle_increment + scan_msg.angle_min # current angle in radians

            distance = scan_msg.ranges[x] # gives us current distance
            
            derivateRange = self.speed * np.cos(currentAngle)
            if(derivateRange <= 0):
                derivateRange = TTC = 1_000_000; 
            else :
                TTC = distance / derivateRange; 
                TTC*=-1;
            # print(TTC)

        # TODO: publish command to brake
            if (TTC < 1.5 or self.ebrake):
                self.ebrake = True
                msg = AckermannDriveStamped()
                msg.drive.speed = 0.0
                self.drive_pub.publish(msg)
                self.get_logger().info(f' TTC: {TTC}\tPublishingto /drive, Initiating Emergency Brakes to avoid collision')
                return
            pass
        

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()