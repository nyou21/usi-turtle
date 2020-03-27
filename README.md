# Usi Turtle


## **Nadia Younis** - Usi Università della Svizzera Italiana - Assignment 1 Robotics 2020

The code runs in a ROS environment that uses Ubuntu, in which TurtleSim has to be installed. The program will be executed, by compiling the catkin package, having the name usi_angry_turtle and then executing the following commands inside the terminal:

- roscore
- rosrun turtlesim turtlesim_node
- rosrun turtlesim turtle_teleop_key 
- rosrun usi_angry_turtle assignment.py


## Details

I have done all the points specified in the Assignment.

The goal of this assigment is to be able to maneuver a turtle (`turtle_main`), whose main purpose is to write the word **"USI"** on the screen, while a number `n` of turtles (in this case only one `turtle1`), move arbitrarily in the available space.
Regarding `turtle1` a remote control is available, using the terminal specifically created for it.
The `turtle_main` will start chasing the` turtle1` the moment it approaches. The latter will be killed, and spawned again on the screen, while the `turtle_main` will start writing again from the beginning.
The program never stops, which is why it is important to do it manually when you are satisfied with the execution.
Furthermore, while the `turtle_main` chases the` turtle1`, it does not target her directly as in a real chase, but foresees the position of the `turtle1` (a few meters).

