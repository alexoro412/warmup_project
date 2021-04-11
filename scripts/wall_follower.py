#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan

class WallFollower():
    """
    Pilots a robot to follow a wall
    """
    def __init__(self):
        """
        Initialize the node and publisher
        """
        rospy.init_node('wall_follower')
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Parameters for following
        self.desired_distance = 0.7
        
    def set_velocity(self, x, z):
        """
        Set angular and linear velocity
        """
        vel_msg = Twist()
        vel_msg.linear.x = x 
        vel_msg.angular.z = z
        self.vel_publisher.publish(vel_msg)

    def process_scan(self, scan_data):
        """
        Read scan data, and determine
        how to move so as to follow the wall
        """
        # For a distance, read the minimum distance
        # among a set of angles
        front_distance = min(scan_data.ranges[-10:-1] + scan_data.ranges[0:10])
        angular_vel = 0
        linear_vel = 0
        if front_distance < self.desired_distance:
            # Starting turning when too close to the wall
            angular_vel = 0.6 
            linear_vel = 0
        else:
            # Not too close in front, go forward
            linear_vel = 0.4
        self.set_velocity(linear_vel, angular_vel)

    def run(self):
        """
        Sets up the scan subscriber and spins
        """
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        rospy.spin()       

if __name__ == '__main__':
    WallFollower().run()
