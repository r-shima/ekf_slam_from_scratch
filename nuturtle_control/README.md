# Nuturtle Control
This package enables control of the turtlebot in both the simulation and the real world.
## Launch File
`ros2 launch nuturtle_control start_robot.launch.xml` allows the user to send cmd_vel commands to the turtlebot, receive odometry, and visualize everything in RViz. The arguments are the following:
* `cmd_src` - Lets the user set it to 'circle' to start the circle node, 'teleop' to start the teleop_twist_keyboard, and 'none' to start nothing
* `robot` - Lets the user set it to 'nusim' to start the nusim simulator, 'localhost' to run the nodes directly from the turtlebot3, and 'none' to not launch any additional nodes
* `use_rviz` - Determines whether or not to launch RViz with a configuration that enables seeing the robot model, tf frames, and the odometry
## Parameters
The parameters in `nuturtle_description/config/diff_params.yaml` can be used to change the simulator settings. Please see the `nuturtle_description` package for a list of parameters.
# Physical Testing
In the videos shown below, the turtlebot moves in a circle (clockwise and counterclockwise) and stops at its initial configuration in RViz and the real world.

[Alt-Text](https://github.com/r-shima/ekf_slam_from_scratch/assets/113070827/403629fc-f195-41a7-9f64-c17434092542)

[Alt-Text](https://github.com/r-shima/ekf_slam_from_scratch/assets/113070827/90efad86-9014-4fc0-8beb-4c0fba5f30f4)

The final location of the turtlebot according to odometry:

    position:
      x: 0.01961198662609796
      y: 0.01032656152978903
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.057767832257636216
      w: 0.9983300444022776