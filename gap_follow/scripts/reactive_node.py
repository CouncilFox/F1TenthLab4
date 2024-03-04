#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

import json

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
        self.lidarscan_topic = '/scan'
        self.drive_topic = '/drive'

        self.subscription = self.create_subscription(LaserScan, self.lidarscan_topic, self.lidar_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        
        self.safety_distance = 0.25  # meters

        self.file_written = False

        self.look_distance = 3

    def preprocess_lidar_with_threshold(self, ranges):
        # Convert the ranges to a NumPy array for easier manipulation
        proc_ranges = np.array(ranges)
        
        # Set values lower than look_distance to 1 (obstacle) and higher to 0 (free space)
        binary_ranges = np.where(proc_ranges <= self.look_distance, 1, 0)
        
        return binary_ranges


    def find_max_gap(self, binary_ranges):
        # Adding zeros at the beginning and end to ensure detection of gaps at edges
        padded_ranges = np.insert(binary_ranges, [1, len(binary_ranges)], 1)
        print([padded_ranges])
        
        # Calculate changes to find transitions
        changes = np.diff(padded_ranges)
        
        # Start of a gap is marked by a transition from 1 to 0 (-1 in diff)
        start_indices = np.where(changes == -1)[0]
        print(start_indices)
        # End of a gap is marked by a transition from 0 to 1 (1 in diff)
        end_indices = np.where(changes == 1)[0]
        print(end_indices)
        
        # Ensure at least one gap is found
        if len(start_indices) == 0 or len(end_indices) == 0 or len(start_indices) != len(end_indices):
            return 0, 0  # Default to no gap if none found or if there's a mismatch
        
        # Calculate gap lengths and find the index of the largest gap
        gap_lengths = end_indices - start_indices
        max_gap_index = np.argmax(gap_lengths)
        
        # Return the start and end indices of the largest gap
        return start_indices[max_gap_index], end_indices[max_gap_index]

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        # Naive approach: Choose the furthest point within the range
        best_point_index = np.argmax(ranges[start_i:end_i+1]) + start_i
        return best_point_index
    

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        # # Convert the ROS LaserScan ranges data to a Python list

        ranges = np.array(data.ranges)
        proc_ranges = self.preprocess_lidar_with_threshold(ranges)

        # # Find closest point to LiDAR
        closest_point = np.argmin(proc_ranges)
        print(f'Closest Point: {closest_point}')
        # Set points inside 'bubble' (safety distance) to zero
        ranges[max(0, closest_point - int(self.safety_distance)):min(len(ranges), closest_point + int(self.safety_distance))] = 1
        
        # Find max length gap
        start_i, end_i = self.find_max_gap(proc_ranges)
        print(f'Start i: {start_i} | End i: {end_i}')

        # Find the best point in the gap
        best_point_index = self.find_best_point(start_i, end_i, ranges)

        # Calculate the angle of the best point relative to the vehicle
        angle = best_point_index * data.angle_increment + data.angle_min 
        print((data.angle_max - data.angle_min) / data.angle_increment)

            # Scale down speed based on the turn angle in radians
        max_speed = 2.0  # Maximum speed
        abs_angle_radians = abs(angle)  # Absolute steering angle in radians

        # Simple formula to scale down speed as angle increases. Adjust the scaling factor as needed for your application.
        # This example reduces speed linearly as the absolute steering angle increases, using a scaling factor for radians.
        scaling_factor = 1.0  # Adjust based on your vehicle's sensitivity to steering angles
        speed = max_speed * (1 - abs_angle_radians / scaling_factor)
        
        # Ensure speed does not fall below a minimum threshold
        min_speed = 0.5
        speed = max(speed, min_speed)

        # Publish Drive message with adjusted speed
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed  # Adjusted speed based on the turn angle
        drive_msg.drive.steering_angle = angle * 1.0  # Multiplying by -1 if needed based on your vehicle's steering setup
        print(f'Steering angle is: {drive_msg.drive.steering_angle}, Speed: {drive_msg.drive.speed}')
        self.publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()