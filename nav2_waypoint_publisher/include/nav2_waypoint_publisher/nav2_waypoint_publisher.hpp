#ifndef WAYPOINT_PUBLISHER_CORE_HPP_
#define WAYPOINT_PUBLISHER_CORE_HPP_

#include <cmath>
#include <stdexcept>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class WayPointPublisher : public rclcpp::Node
{
public:
  WayPointPublisher();

private:
  std::vector<std::string> getCSVLine(std::string& input, char delimiter);
  void publishWaypointsFromCSV(std::string csv_file);
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_pub_;
  // rclcpp::Clock ros_clock(RCL_ROS_TIME);
  int id_;
  std::string csv_file_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_goal_handle_;


};

#endif