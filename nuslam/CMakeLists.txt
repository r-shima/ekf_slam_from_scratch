cmake_minimum_required(VERSION 3.8)
project(nuslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nuturtle_control REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Armadillo)

add_executable(slam src/slam.cpp)

ament_target_dependencies(slam
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  nav_msgs
  sensor_msgs
  nuturtle_control
  visualization_msgs)

include_directories(${ARMADILLO_INCLUDE_DIRS})

target_link_libraries(slam turtlelib::turtlelib ${ARMADILLO_LIBRARIES})

install(TARGETS
  slam
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/nuslam/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
