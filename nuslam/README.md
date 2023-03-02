# Nuslam
This package implements Extended Kalman Filter SLAM, assuming a known data association. The algorithm computes the locations of the robots and the locations of the landmarks relative to a map frame.
## Launch File
`ros2 launch nuslam slam.launch.xml` allows the user to run and visualize the SLAM algorithm. The arguments are the following:
* `cmd_src` - Lets the user set it to 'circle' to start the circle node, 'teleop' to start the teleop_twist_keyboard, and 'none' to start nothing
* `robot` - Lets the user set it to 'nusim' to start the nusim simulator, 'localhost' to run the nodes directly from the turtlebot3, and 'none' to not launch any additional nodes
* `use_rviz` - Determines whether or not to launch RViz with a configuration that enables seeing the robot model, tf frames, and the odometry
## Parameters
The parameters in `nuturtle_description/config/diff_params.yaml` can be used to change the simulator settings. Please see the `nuturtle_description` package for a list of parameters.
## Screenshot
![](images/slam.png)
## Video
Below is a video of the robot moving in a closed path in RViz with several landmarks.
[Alt-Text](https://user-images.githubusercontent.com/113070827/222328189-ac7f5df1-a594-42dd-954b-b6a3e2f50934.webm)