import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import time # added time

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.laser_sub = self.create_subscription (
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        # TODO: set PID gains
        self.kp = 2.5
        self.ki = 1
        self.kd = 2

        # TODO: store history
        self.integral =     0.0
        self.prev_error =   0.0
        self.error =        0.0

        # TODO: store any necessary values you think you'll need
        self.D_distance =   1.0                # desired distance
        self.a_angle =      40 * np.pi/180      # tunable angle
        self.b_angle =      90 * np.pi/180      # perpendicular angle

        self.STRAIGHT_AHEAD_SPEED = 3.0
        self.STRAIGHT_AHEAD_THRESHOLD = 10*np.pi/180
        self.WIDE_TURN_SPEED = 2.0
        self.WIDE_TURN_THRESHOLD = 20*np.pi/180
        self.SHARP_TURN_SPEED = 1.0
        self.current_time = time.time()
        self.current_speed = 0.0

        self.prev_time = 0.0
    

    

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        #TODO: implement
        # gets the distance 
        range = range_data.ranges[int((angle + range_data.angle_min)/range_data.angle_increment)]
        if (range < range_data.range_min):
            range = range_data.range_min
        
        if (range > range_data.range_max):
            range = range_data.range_max

        return range

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error "term"
        """
        #TODO:implement
        theta = self.b_angle - self.a_angle

        a_angle_distance = self.get_range(range_data, self.a_angle)
        b_angle_distance = self.get_range(range_data, self.b_angle)

        alpha = np.arctan(((a_angle_distance*(np.cos(theta)))-b_angle_distance)
                          /(a_angle_distance*np.sin(theta)))

        Distance_t = b_angle_distance * np.cos(alpha)

        L = self.current_speed
        # L = 1.5

        Distance_t_plus_1 = (Distance_t + (L * np.sin(alpha)))

        error_term = dist - Distance_t_plus_1

        self.error = error_term

        return error_term

    def pid_control(self, error):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller

        self.integral+=((self.prev_error+error)*(self.current_time - self.prev_time)/2)

        angle = -((self.kp * error) + 
                 (self.ki*self.integral) +
                 (self.kd * ((self.prev_error - error)/(self.current_time-self.prev_time))))

        # clamp based on anything greater than a wider turn
        if(angle < self.WIDE_TURN_THRESHOLD*-1):
            angle = self.WIDE_TURN_THRESHOLD*-1
        elif angle > self.WIDE_TURN_THRESHOLD:
            angle = self.WIDE_TURN_THRESHOLD
        


        #give speeds dependant on our angle 
        if(np.abs(angle) < self.STRAIGHT_AHEAD_THRESHOLD):
            self.current_speed = self.STRAIGHT_AHEAD_SPEED
        elif np.abs(angle) < self.WIDE_TURN_THRESHOLD:
            self.current_speed = self.WIDE_TURN_SPEED
        else:
            self.current_speed = self.SHARP_TURN_SPEED
        # self.current_speed = 1.5

        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.drive.speed = self.current_speed
        drive_msg.drive.steering_angle = angle
        self.drive_pub.publish(drive_msg)

        return





    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """

        # updating time variables
        self.prev_time = self.current_time
        self.current_time = time.time()

        self.prev_error = self.error
        self.error = self.get_error(msg, self.D_distance) # TODO: replace with error calculated by get_error()


        self.pid_control(self.error) # TODO: actuate the car with PID
        return 







def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()