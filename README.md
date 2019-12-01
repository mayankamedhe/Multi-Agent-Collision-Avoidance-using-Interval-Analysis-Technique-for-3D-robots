Multi-Agent Collision Avoidance using Interval Analysis Technique for 3D robots
===============================================================================

Abstract
--------

Path planning of multi-agent systems is a fast emerging field in robotics. The main motivation behind this problem is that a collection of many mobile robots can collaborate to perform complex tasks which are used in many industrial applications like Military Logistic Planning, coverage, motion planning. These agents can be static or dynamic. In this technique, intervals are defined according to the robot's maximum change in velocity. They are 3D sectors representing the pose of a robot in a fixed time-interval. An inclusion test is performed which gives the interval of overlap between them which is used to get the free interval.

Running
-------

Assuming a properly setup ROS, catkin workspace, and Gazebo, this should be all that is necessary:

`roslaunch simple_create create_world.launch`
