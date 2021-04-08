#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan

class PersonFollower():
    """
    Pilots a robot to drive in a square
    """
    def __init__(self):
        """
        Initialize the node and publisher
        """
        rospy.init_node('person_follower')
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
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
        Based on scan data, use proportional
        control to turn towards the person.
        When pointing in roughly the right 
        direction, start to move towards
        the person
        """

        # get how far off angle is
        # turn appropriately
        closest = 4
        person_angle = None
        for i, dist in enumerate(scan_data.ranges):
            if dist < closest:
                closest = dist
                person_angle = i

        # Do nothing if it can't find the robot
        if person_angle is None:
            self.set_velocity(0, 0)
            return
  
        # Change angles from 0 to 360 degrees
        # to -180 to 180 degrees
        if person_angle >= 180:
            person_angle -= 360
        angular_vel = 0.03 * person_angle

        # move towards the person if the 
        # robot is roughly pointing at them,
        # but only move if we're not too close
        linear_vel = 0
        if abs(person_angle) <= 20 and closest > 0.4:
            linear_vel = 0.3 * closest

        # Don't wiggle when too close to the person
        if closest <= 0.4 and abs(person_angle) <= 10:
            angular_vel = 0

        self.set_velocity(linear_vel, angular_vel)

      

    def run(self):
        """
        Loops through a list of movements and executes
        them to drive in a square
        """
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        rospy.spin()       

if __name__ == '__main__':
    PersonFollower().run()
