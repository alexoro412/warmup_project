#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 

class MoveState():
    def __init__(self, x, z, duration):
        self.x = x 
        self.z = z
        self.duration = duration

class SquareDriver():
    def __init__(self):
        rospy.init_node('square_driver')
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
    def set_velocity(self, x, z):
        vel_msg = Twist()
        vel_msg.linear.x = x 
        vel_msg.angular.z = z 
        self.vel_publisher.publish(vel_msg)

    def run(self):
        r = rospy.Rate(100)
        r.sleep()
        states = [MoveState(0,0,1), MoveState(0.3,0,3), MoveState(0,0,1), MoveState(0.1,0.27,5)]
        last_transition = rospy.Time.now()
        current_state_i = 0
        current_state = states[current_state_i]
        self.set_velocity(current_state.x, current_state.z)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if (current_time - last_transition).secs > current_state.duration:
                current_state_i += 1
                if current_state_i >= len(states):
                    current_state_i = 0
                last_transition = current_time
                current_state = states[current_state_i]
                self.set_velocity(current_state.x, current_state.z)
            r.sleep()

if __name__ == '__main__':
    SquareDriver().run()