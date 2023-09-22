import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        
        self.laser_sub = self.create_subscription (
            LaserScan,
            lidarscan_topic,
            self.lidar_callback,
            10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )

        self.window = 5
        self.depth  = 3.0
        self.car_radius = 0.5/2

        self.min_index = 0
        self.max_index = 0

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        for i, val in enumerate(ranges):
            mean_of_values = 0
            check_end = False

            # for the next 5 indices
            for j in range(self.window):
                # if out of bounds, break
                if (i+j >= len(ranges)):
                    check_end = True
                    break

                # add value to mean
                mean_of_values += ranges[i+j]

            # if not end of loop
            if (check_end == False):
                # change value at i to mean over window
                ranges[i] = mean_of_values/self.window

            # if high value
            if (ranges[i] > self.depth):
                # set high value to max allowed
                ranges[i] = 0

            # set all values outside of min/max range to 0
            #UGLY not needed since we always check from min to max range index
            if (i > self.max_index):
                ranges[i] = 0
            if (i < self.min_index):
                ranges[i] = 0

        return ranges

    def find_max_gap(self, free_space_ranges):
        """Return the start index & end index of the max gap in free_space_ranges"""
        # max_start  = -1
        # max_end = -1
        # current_start = self.min_index
        # max_gap = 0
        # current_gap = 0

        # for i in range(self.min_index, self.max_index+1):
        #     val = free_space_ranges[i]
        #     if val > 0:
        #         current_gap += 1
        #     else:
        #         if current_gap > max_gap:
        #             max_gap = current_gap
        #             max_start = current_start
        #             max_end = i
        #         current_start = i
        #         current_gap = 0
 
        # if current_gap > max_gap:
        #     max_gap = current_gap
        #     max_start = current_start
        #     max_end = self.max_index

        # return max_start, max_end
        
        start_i = self.min_index
        start_j = self.min_index
        in_range = False
        temp_i = 0
        temp_j = 0

        # for the range of 90 to -90
        for n in range(self.min_index, self.max_index+1):
            # if val 0, and temp range > start range (bigger gap)
            if(free_space_ranges[n] == 0 and temp_j-temp_i > start_j-start_i):
                # update start indices with bigger gap
                start_i = temp_i
                start_j = temp_j
                in_range = False # reset in_range
            elif not(in_range):
                temp_i = n # update temp index i
                in_range = True # true bool
            else:
                temp_j = n # update temp index j

        if (temp_j-temp_i > start_j-start_i):
            start_i = temp_i
            start_j = temp_j

        # return start and end indices of max gap
        return start_i, start_j
    
    def find_best_point(self, start_i, end_i, ranges):
        """ Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and return index"""
        furthest_point = start_i
        for x in range(start_i, end_i+1):
            if ranges[x] >= ranges[furthest_point]:
                furthest_point = x

        if (ranges[furthest_point] < 0.5):
            furthest_point = 1080/2


        return furthest_point

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges

        # only do this if statement once, *initialize*
        if not(self.min_index and self.max_index):
            # initialize angle at perpendicular right
            self.min_index = int((45*(np.pi/180)) / data.angle_increment) 
            # initialize angle at perpendicular left
            self.max_index = int(self.min_index + (np.pi / data.angle_increment))

        proc_ranges = self.preprocess_lidar(ranges)

        closest_index = 0
        #Find closest point to LiDAR
            # Loop through entire valid range of indices
        for x in range(self.min_index, self.max_index):
            # if the value at new index is less than closest index value
            if proc_ranges[x] < proc_ranges[closest_index]:
                closest_index = x # set closest_index to new index

        #Eliminate all points inside 'bubble' (set them to zero)
        # distance_single_radian = np.tan(1) * proc_ranges[closest_index] # this gets x
        # distance_needed_in_radians = self.car_radius/distance_single_radian
        # number_of_indices = int(distance_needed_in_radians/data.angle_increment)
        number_of_indices = int(self.car_radius/data.angle_increment)
        for n in range(closest_index-number_of_indices, closest_index+number_of_indices+1):
            if 0 <= n < len(proc_ranges):
                # print((n * data.angle_increment + data.angle_min)*180/np.pi)
                proc_ranges[n] = 0


        print(proc_ranges)

        #Find max length gap 
        start_i, start_j = self.find_max_gap(proc_ranges)
        # print(start_i)
        # print((start_i * data.angle_increment + data.angle_min)*180/np.pi)
        # print(start_j)
        # print((start_j * data.angle_increment + data.angle_min)*180/np.pi)


        #Find the best point in the gap
        best_point = self.find_best_point(start_i, start_j, proc_ranges)

        # print(best_point)
        # print((best_point * data.angle_increment + data.angle_min)*180/np.pi)

        # print(proc_ranges[best_point])
        # print()

        exit()

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.3
        drive_msg.drive.steering_angle = best_point*data.angle_increment + data.angle_min
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()