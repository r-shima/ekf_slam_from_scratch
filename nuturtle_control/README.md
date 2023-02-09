# Nuturtle Control
This package enables control of the turtlebot in both the simulation and the real world.
## Launch File
`ros2 launch nuturtle_control start_robot.launch.xml` allows the user to send cmd_vel commands to the turtlebot, receive odometry, and visualize everything in rviz. The arguments are the following:
* `cmd_src` - Lets the user set it to 'circle' to start the circle node, 'teleop' to start the teleop_twist_keyboard, and 'none' to start nothing
* `robot` - Lets the user set it to 'nusim' to start the nusim simulator, 'localhost' to run the nodes directly from the turtlebot3, and 'none' to not launch any additional nodes
* `use_rviz` - Determines whether or not to launch rviz with a configuration that enables seeing the robot model, tf frames, and the odometry