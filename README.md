# ME495 Sensing, Navigation and Machine Learning For Robotics
* Rintaroh Shima
* Winter 2023
# Package List
This repository consists of several ROS packages
- nuturtle_description - Displays multiple turtlebot3 models in RViz, each appearing with a different color. Allows you to change the physical properties of the robot by editing a yaml file.
- nusim - Provides a simulated environment with three cylindrical obstacles for a red turtlebot. Includes services to teleport the robot to a desired location and reset itself.
- nuturtle_control - Enables control of the turtlebot in both the simulation and the real world
# Custom Library
- turtlelib - A library for performing 2D rigid body transformations and other turtlebot-related math
# Physical Testing
In the videos shown below, the turtlebot moves in a circle (clockwise and counterclockwise) and stops at its initial configuration.

[Alt-Text](https://user-images.githubusercontent.com/113070827/217684866-9861df1e-4646-47e0-be22-85ffd615b72a.mp4)

[Alt-Text](https://user-images.githubusercontent.com/113070827/217684918-e293c4c9-c029-4dd2-afe4-30719885b758.webm)

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