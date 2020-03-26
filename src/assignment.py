#!/usr/bin/env python
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import turtlesim.srv
import std_srvs.srv
from math import pow, atan2, sqrt, cos, sin


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        rospy.wait_for_service('clear')
        self.clear = rospy.ServiceProxy('clear', std_srvs.srv.Empty())

        rospy.wait_for_service('reset')
        self.reset = rospy.ServiceProxy('reset', std_srvs.srv.Empty())

        rospy.wait_for_service('kill')
        self.kill = rospy.ServiceProxy('kill', turtlesim.srv.Kill)

        rospy.wait_for_service('spawn')
        self.spawn = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)


        self.reset()

        rospy.wait_for_service('/turtle1/set_pen')
        self.set_pen_turtle1 = rospy.ServiceProxy('/turtle1/set_pen', turtlesim.srv.SetPen)
        self.set_pen_turtle1(0, 0, 0, 0, 1) # Turn off pen turtle1

        self.spawn(2.0, 2.0, 0.5, 'turtle_main')

        rospy.wait_for_service('/turtle_main/set_pen')
        self.set_pen = rospy.ServiceProxy('/turtle_main/set_pen', turtlesim.srv.SetPen)

        self.velocity_publisher = rospy.Publisher('/turtle_main/cmd_vel', Twist, queue_size=10)
        self.pose_main_subscriber = rospy.Subscriber('/turtle_main/pose', Pose, self.update_pose_main)
        self.pose_turtle1_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose_turtle1)

        # self.must_stop = False
        self.pose_main = Pose()
        self.pose_turtle1 = Pose()

        self.state = None

        self.rate = rospy.Rate(10)
        self.rate.sleep()



    def update_pose_main(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose_main = data

    def update_pose_turtle1(self, data):    
        self.pose_turtle1 = data



    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose_main.x), 2) +
                    pow((goal_pose.y - self.pose_main.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose_main.y, goal_pose.x - self.pose_main.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * np.arctan2(
                                np.sin(self.steering_angle(goal_pose) - self.pose_main.theta), 
                                np.cos(self.steering_angle(goal_pose) - self.pose_main.theta))
        # return constant * (self.steering_angle(goal_pose) - self.pose_main.theta)


    def predict_pose(self, initial_pose):
        predict_pose = Pose()
        m = initial_pose.linear_velocity * self.euclidean_distance(initial_pose) * 0.75
        predict_pose.x = m * cos(initial_pose.theta) + initial_pose.x
        predict_pose.y = m * sin(initial_pose.theta) + initial_pose.y
        predict_pose.theta = initial_pose.theta
        return predict_pose

    def move2goal(self, x, y):
        """Moves the turtle to the goal."""

        if self.state == "returning":
            return # skip instructions below and exit this method immediatly (skipping this move2goal)

        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = x
        goal_pose.y = y

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.3

        vel_msg = Twist()

        while not rospy.is_shutdown() and self.euclidean_distance(goal_pose) >= distance_tolerance:

            if (self.euclidean_distance(self.pose_turtle1) < 2):
                rospy.loginfo("turtle_main is angry")
                self.state = "angry"

                self.set_pen(0, 0, 0, 0, 1) # Turn off pen 
                
                # following the offender
                pr_pose = Pose()
                while not rospy.is_shutdown() and self.euclidean_distance(self.pose_turtle1) >= distance_tolerance:
                    pr_pose = self.predict_pose(self.pose_turtle1)

                    vel_msg.linear.x = self.linear_vel(pr_pose) # Linear velocity in the x-axis.
                    vel_msg.angular.z = self.angular_vel(pr_pose) # Angular velocity in the z-axis.
                    self.velocity_publisher.publish(vel_msg) # Publishing our vel_msg
                    self.rate.sleep() # Publish at the desired rate.

                # offender has been caught
                
                rospy.loginfo("turtle1 has been caught")

                self.state = "returning"

                self.kill("turtle1") # kill the offender 
                rospy.loginfo("turtle1 has been killed")

                self.pose_turtle1.x = -5 # reset turtle1 x
                self.pose_turtle1.y = -5 # reset turtle1 y
                
                self.spawn(2.0, 2.0, 0.5, 'turtle1')
                self.set_pen_turtle1(0, 0, 0, 0, 1)

                break


            # Porportional controller (https://en.wikipedia.org/wiki/Proportional_control)
            vel_msg.linear.x = self.linear_vel(goal_pose) # Linear velocity in the x-axis.
            vel_msg.angular.z = self.angular_vel(goal_pose) # Angular velocity in the z-axis.

            self.velocity_publisher.publish(vel_msg) # Publishing our vel_msg
            self.rate.sleep() # Publish at the desired rate.


        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)




if __name__ == '__main__':
    try:

        color_red = 255
        color_green = 255
        color_blue = 255
        width = 4

        x = TurtleBot()
    

        while not rospy.is_shutdown():
            x.state = "writing"
            rospy.loginfo("turtle_main start to write")

            x.clear()

            x.set_pen(color_red,color_green,color_blue,width,1) # pen off
            x.move2goal(1, 9)
            x.move2goal(1, 8)

            # U
            x.set_pen(color_red,color_green,color_blue,width,0) # pen on
            x.move2goal(1, 4)
            x.move2goal(2, 3)
            x.move2goal(3, 4)
            x.move2goal(3, 8)
            
            x.set_pen(color_red,color_green,color_blue,width,1) # pen off
            x.move2goal(6.5, 8)

            #S
            x.set_pen(color_red,color_green,color_blue,width,0) # pen on
            x.move2goal(5, 8)
            x.move2goal(4, 7)
            x.move2goal(5.5, 5.5)
            x.move2goal(6.5, 4.5)
            x.move2goal(4, 3)

            x.set_pen(color_red,color_green,color_blue,width,1) # pen off
            x.move2goal(8, 3)

            # I
            x.set_pen(color_red,color_green,color_blue,width,0) # pen on
            x.move2goal(8, 4) 
            x.move2goal(8, 8)
        
        # If we press control + C, the node will stop.
        #rospy.spin()

    except rospy.ROSInterruptException:
        pass