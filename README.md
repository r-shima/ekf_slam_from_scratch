# Extended Kalman Filter (EKF) SLAM from Scratch
* Rintaroh Shima
* Winter 2023
# Package List
This repository consists of several ROS packages
- [nuturtle_description](https://github.com/r-shima/ekf_slam_from_scratch/tree/main/nuturtle_description) - Displays multiple turtlebot3 models in RViz, each appearing with a different color. Allows you to change the physical properties of the robot by editing a yaml file.
- [nusim](https://github.com/r-shima/ekf_slam_from_scratch/tree/main/nusim) - Provides a simulated environment with three cylindrical obstacles for a red turtlebot. Includes services to teleport the robot to a desired location and reset itself.
- [nuturtle_control](https://github.com/r-shima/ekf_slam_from_scratch/tree/main/nuturtle_control) - Enables control of the turtlebot in both the simulation and the real world
- [nuslam](https://github.com/r-shima/ekf_slam_from_scratch/tree/main/nuslam) - Performs Extended Kalman Filter SLAM with known and unknown data associations on the turtlebot
# Custom Library
- [turtlelib](https://github.com/r-shima/ekf_slam_from_scratch/tree/main/turtlelib) - A library for performing 2D rigid body transformations and other turtlebot-related math
# Videos
In the videos, three robots are displayed:
- Red - Ground truth
- Blue - Odometry
- Green - EKF SLAM estimate

Cylindrical obstacles are also displayed:
- Red - Ground truth
- Blue - Detected landmarks after performing clustering and circle fitting (shown only in the second video)
- Green - Obstacles as detected by EKF SLAM

Below is a video of the robot moving in a closed path in RViz with several landmarks (known data association):

[Alt-Text](https://user-images.githubusercontent.com/113070827/228314355-bcffdd45-5ceb-44ab-8ae1-e4523d558d46.mp4)

Below is a video of the robot moving in a closed path using unknown data association in RViz:

[Alt-Text](https://github.com/r-shima/ekf_slam_from_scratch/assets/113070827/2b6d4d40-4459-43d6-8ea6-a5668cea72b8)