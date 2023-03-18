/// \file
/// \brief This node detects landmarks and publishes their relative locations
///
/// PARAMETERS:
///     obstacles.r (double): the radius of the obstacles
/// PUBLISHES:
///     /clusters (visualization_msgs::msg::MarkerArray): cylindrical markers that act as
///                                                       clusters
///     /circles (visualization_msgs::msg::MarkerArray): cylindrical markers to help visualize
///                                                      detected circles
/// SUBSCRIBES:
///     /laser_scan (sensor_msgs::msg::LaserScan): laser points used to simulate the lidar

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlelib/circle_fitting.hpp"

/// \brief Performs clustering and circle fitting to detect landmarks
class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks")
  {
    // Initializes variables for the parameter, publishers, and subscriber
    declare_parameter("obstacles.r", 0.038);
    obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<double>();

    clusters_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/clusters", 10);
    circles_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/circles", 10);
    laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/laser_scan", 10, std::bind(
        &Landmarks::laser_scan_callback, this,
        std::placeholders::_1));

    threshold_ = 0.05;
    in_cluster_ = false;
    wrap_around_ = false;
  }

private:
  /// \brief Callback function for the subscriber that subscribes to sensor_msgs/msg/LaserScan.
  /// Clusters points into groups corresponding to individual landmarks, performs circle fitting,
  /// and classifies the clusters of points
  ///
  /// \param msg - LaserScan object
  /// \returns none
  void laser_scan_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    std::vector<std::vector<turtlelib::Vector2D>> cluster_list;

    for (size_t i = 0; i < msg.ranges.size(); i++) {
      std::vector<turtlelib::Vector2D> cluster;
      if (msg.ranges.at(i) > 0.0) {
        in_cluster_ = true;
        int num = 0;

        while (in_cluster_ == true) {
          turtlelib::Vector2D point1, point2;
          double range = msg.ranges.at((i + num) % 360);
          double theta = static_cast<double>((i + num) % 360);
          double next_range = msg.ranges.at((i + 1 + num) % 360);
          double next_theta = static_cast<double>((i + 1 + num) % 360);

          point1.x = range * cos(theta * (turtlelib::PI / 180.0));
          point1.y = range * sin(theta * (turtlelib::PI / 180.0));
          point2.x = next_range * cos(next_theta * (turtlelib::PI / 180.0));
          point2.y = next_range * sin(next_theta * (turtlelib::PI / 180.0));
          double distance = calculate_distance(point1.x, point1.y, point2.x, point2.y);

          if (num == 0) {
            cluster.push_back(point1);
          }

          if (distance < threshold_) {
            cluster.push_back(point2);
            num++;
          } else if (num >= 4) {
            in_cluster_ = false;
            cluster_list.push_back(cluster);
            i = i + num - 1;
          } else {
            in_cluster_ = false;
          }

          if (i + 1 + num >= 360) {
            wrap_around_ = true;
          }
        }
      }
    }

    if (wrap_around_ == true) {
      if (cluster_list.front().back().x == cluster_list.back().back().x &&
        cluster_list.front().back().y == cluster_list.back().back().y)
      {
        cluster_list.at(0) = cluster_list.back();
        cluster_list.pop_back();
      }
    }

    add_clusters(cluster_list);
    detect_circle(cluster_list);
  }

  /// \brief calculates the distance between two points
  ///
  /// \param x1 - the x-coordinate of the first point
  /// \param y1 - the y-coordinate of the first point
  /// \param x2 - the x-coordinate of the second point
  /// \param y2 - the y-coordinate of the second point
  /// \returns the distance between two points
  double calculate_distance(double x1, double y1, double x2, double y2)
  {
    double distance = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    return distance;
  }

  /// \brief Creates markers for the clusters, adds them to a marker array, and publishes it
  ///
  /// \param cluster_list - the vector of clusters
  /// \returns none
  void add_clusters(std::vector<std::vector<turtlelib::Vector2D>> cluster_list)
  {
    visualization_msgs::msg::MarkerArray cluster_array;
    for (size_t i = 0; i < cluster_list.size(); i++) {
      double x_mean = 0.0;
      double y_mean = 0.0;
      double num_of_points = 0.0;
      for (size_t j = 0; j < cluster_list.at(i).size(); j++) {
        x_mean += cluster_list.at(i).at(j).x;
        y_mean += cluster_list.at(i).at(j).y;
        num_of_points++;
      }

      visualization_msgs::msg::Marker cluster_marker;
      cluster_marker.header.frame_id = "green/base_footprint";
      cluster_marker.header.stamp = get_clock()->now();
      cluster_marker.id = i;
      cluster_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      cluster_marker.action = visualization_msgs::msg::Marker::ADD;
      cluster_marker.pose.position.x = x_mean / num_of_points;
      cluster_marker.pose.position.y = y_mean / num_of_points;
      cluster_marker.pose.position.z = 0.125;
      cluster_marker.scale.x = 2.0 * obstacles_r_;
      cluster_marker.scale.y = 2.0 * obstacles_r_;
      cluster_marker.scale.z = 0.25;
      cluster_marker.color.r = 0.2;
      cluster_marker.color.g = 0.2;
      cluster_marker.color.b = 0.2;
      cluster_marker.color.a = 1.0;
      cluster_array.markers.push_back(cluster_marker);
    }
    clusters_pub_->publish(cluster_array);
  }

  /// \brief Runs the circle fitting algorithm and classifies the clusters of points
  /// \param cluster_list - the vector of clusters
  /// \returns none
  void detect_circle(std::vector<std::vector<turtlelib::Vector2D>> cluster_list)
  {
    std::vector<turtlelib::Circle> detected_circle_list;
    for (size_t i = 0; i < cluster_list.size(); i++) {
      turtlelib::Circle circle = turtlelib::fit_circle(cluster_list.at(i));
      if (circle.r > 0.01 && circle.r < 0.1) {
        detected_circle_list.push_back(circle);
      }
    }
    add_detected_circle(detected_circle_list);
  }

  /// \brief Creates markers for the detected circles, adds them to a marker array, and publishes it
  /// \param detected_circle_list - the vector of detected circles
  /// \returns none
  void add_detected_circle(std::vector<turtlelib::Circle> detected_circle_list)
  {
    visualization_msgs::msg::MarkerArray circle_array;
    for (size_t i = 0; i < detected_circle_list.size(); i++) {
      visualization_msgs::msg::Marker circle_marker;
      circle_marker.header.frame_id = "green/base_footprint";
      circle_marker.header.stamp = get_clock()->now();
      circle_marker.id = i;
      circle_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      circle_marker.action = visualization_msgs::msg::Marker::ADD;
      circle_marker.pose.position.x = detected_circle_list.at(i).x;
      circle_marker.pose.position.y = detected_circle_list.at(i).y;
      circle_marker.pose.position.z = 0.125;
      circle_marker.scale.x = 2.0 * detected_circle_list.at(i).r;
      circle_marker.scale.y = 2.0 * detected_circle_list.at(i).r;
      circle_marker.scale.z = 0.25;
      circle_marker.color.r = 0.0;
      circle_marker.color.g = 0.0;
      circle_marker.color.b = 1.0;
      circle_marker.color.a = 1.0;
      circle_array.markers.push_back(circle_marker);
    }
    circles_pub_->publish(circle_array);
  }

  // Declare private variables
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clusters_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr circles_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

  double obstacles_r_;
  double threshold_;
  bool in_cluster_;
  bool wrap_around_;
};

/// \brief The main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
