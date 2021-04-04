#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 

class MoveState():
    """
    This encapsulates a single movement that the
    robot will perform (linear/angular velocity),
    and how long it will perform that movement.
    """
    def __init__(self, x, z, duration):
        self.x = x 
        self.z = z
        self.duration = duration

class SquareDriver():
    """
    Pilots a robot to drive in a square
    """
    def __init__(self):
        """
        Initialize the node and publisher
        """
        rospy.init_node('square_driver')
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
    def set_velocity(self, x, z):
        """
        Set angular and linear velocity
        """
        vel_msg = Twist()
        vel_msg.linear.x = x 
        vel_msg.angular.z = z
        self.vel_publisher.publish(vel_msg)

    def run(self):
        """
        Loops through a list of movements and executes
        them to drive in a square
        """
        rospy.sleep(1)
        states = [MoveState(0.2,0,4), MoveState(0,0,0.2), MoveState(0,0.3,5)]
        current_state_i = 0
        current_state = states[current_state_i]

        while not rospy.is_shutdown():
            self.set_velocity(current_state.x, current_state.z)
            # wait for this movement to finish
            rospy.sleep(current_state.duration)
            current_state_i += 1
            if current_state_i >= len(states):
                current_state_i = 0
            current_state = states[current_state_i]         

if __name__ == '__main__':
    SquareDriver().run()