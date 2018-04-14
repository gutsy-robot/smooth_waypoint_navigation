# smooth_waypoint_navigation
In this project, I have included a script for the waypoint navigation in a turtlebot_stage_sim(http://wiki.ros.org/turtlebot_stage).

Move action interface already provides functions to do waypoint navigation. However, I wanted to navigate through multiple points without stopping at the intermediate points.

Therefore, I just created a script which subscribes to the amcl_pose topic and pushes the next waypoint goal a little early, before the robot has reached the intermediate goal point.
