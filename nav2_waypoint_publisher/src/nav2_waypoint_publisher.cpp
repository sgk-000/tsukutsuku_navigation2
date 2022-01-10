#include "nav2_waypoint_publisher/nav2_waypoint_publisher.hpp"  // include local header

WayPointPublisher::WayPointPublisher() :
  rclcpp::Node("nav2_waypoint_publisher"), id_(0)
{
  csv_file_ = declare_parameter<std::string>("csv_file", "sample.csv");
  waypoint_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", rclcpp::QoS{10});
  nav_through_poses_action_client_  = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, "navigate_through_poses");

  auto is_action_server_ready =
  nav_through_poses_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      this->get_logger(), "navigate_through_poses action server is not available."
      " Is the initial pose set?");
    return;
  }
  publishWaypointsFromCSV(csv_file_);
}

std::vector<std::string> WayPointPublisher::getCSVLine(std::string& input, char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

void WayPointPublisher::publishWaypointsFromCSV(std::string csv_file)
{
    std::ifstream ifs(csv_file);
    std::string line;
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker origin_marker;
    origin_marker.header.frame_id = "map";
    origin_marker.header.stamp = this->now();
    origin_marker.ns = "waypoints";
    origin_marker.id = id_++;
    origin_marker.type = visualization_msgs::msg::Marker::ARROW;
    origin_marker.action = visualization_msgs::msg::Marker::ADD;
    origin_marker.lifetime = rclcpp::Duration::from_seconds(3600);
    origin_marker.scale.x = 0.5;
    origin_marker.scale.y = 0.5;
    origin_marker.scale.z = 0.5;
    origin_marker.color.r = 0.0f;
    origin_marker.color.g = 1.0f;
    origin_marker.color.b = 0.0f;
    origin_marker.color.a = 1.0f;

    nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal;

    while (getline(ifs, line)) {
        std::vector<std::string> strvec = getCSVLine(line, ',');
        geometry_msgs::msg::PoseStamped goal_msg;
        visualization_msgs::msg::Marker marker;
        marker = origin_marker;
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "map";

        goal_msg.pose.position.x = std::stod(strvec.at(0));
        goal_msg.pose.position.y = std::stod(strvec.at(1));
        goal_msg.pose.orientation.x = std::stod(strvec.at(3));
        goal_msg.pose.orientation.y = std::stod(strvec.at(4));
        goal_msg.pose.orientation.w = std::stod(strvec.at(5));
        goal_msg.pose.orientation.z = std::stod(strvec.at(6));

        marker.pose.position.x = std::stod(strvec.at(0));
        marker.pose.position.y = std::stod(strvec.at(1));
        marker.pose.position.z = std::stod(strvec.at(2));
        marker.pose.orientation.x = std::stod(strvec.at(3));
        marker.pose.orientation.y = std::stod(strvec.at(4));
        marker.pose.orientation.z = std::stod(strvec.at(5));
        marker.pose.orientation.w = std::stod(strvec.at(6));
        std::cout << "x: " << std::stod(strvec.at(0)) << std::endl;
        std::cout << "y: " << std::stod(strvec.at(1)) << std::endl;
        std::cout << "z: " << std::stod(strvec.at(2)) << std::endl;
        std::cout << "x: " << std::stod(strvec.at(3)) << std::endl;
        std::cout << "y: " << std::stod(strvec.at(4)) << std::endl;
        std::cout << "z: " << std::stod(strvec.at(5)) << std::endl;
        std::cout << "w: " << std::stod(strvec.at(6)) << std::endl;
        nav_through_poses_goal.poses.push_back(goal_msg);
        marker_array.markers.push_back(marker);
    }

    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto) {
        nav_through_poses_goal_handle_.reset();
      };
  
    std::chrono::milliseconds server_timeout(30);
    auto future_goal_handle =
      nav_through_poses_action_client_->async_send_goal(nav_through_poses_goal, send_goal_options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle, server_timeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
      return;
    }
  
    // Get the goal handle and save so that we can check on completion in the timer callback
    nav_through_poses_goal_handle_ = future_goal_handle.get();
    if (!nav_through_poses_goal_handle_) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      return;
    }
    RCLCPP_INFO(
    this->get_logger(), "Sending a path of %zu waypoints:",
    nav_through_poses_goal.poses.size());
    waypoint_pub_->publish(marker_array);

}
