#!/usr/bin/env python
import rospy
import sys

import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class UsiAngryTurtle:
    def __init__(self, linear=0.2, angular=0.0):
        """init"""
        rospy.init_node('usi_angry_turtle', anonymous=True)

        # Publish to the topic '/turtleX/cmd_vel'
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.turtlesim_pose_callback)
        
        self.set_linear = linear
        self.set_angular = angular
        # publish at this rate
        self.rate = rospy.Rate(10)

    def turtlesim_pose_callback(self, data):
        """A new turltesim Pose has arrived. See turtlesim Pose msg definition."""
        
        rospy.loginfo("Pose received (%f, %f, %f) (%f, %f)" % (data.x, data.y, data.theta, data.linear_velocity, data.angular_velocity))

    def basic_move(self):
        """Moves the turtle"""
        vel_msg = Twist()
        vel_msg.linear.x = self.set_linear #0.2 # m/s
        vel_msg.angular.z = self.set_angular #0.0 # rad/s

        # single message
        #self.velocity_publisher.publish(vel_msg)
        
        # several messages at a rate
        while not rospy.is_shutdown():
            # Publishing vel_msg
            self.velocity_publisher.publish(vel_msg)
            # .. at the desired rate.
            self.rate.sleep()

        # waiting until shutdown flag (e.g. ctrl+c)
        rospy.spin()


if __name__ == '__main__':
    param_linear = 0.5
    param_angular = 0.2

    controller = UsiAngryTurtle(param_linear, param_angular)

    controller.basic_move()

