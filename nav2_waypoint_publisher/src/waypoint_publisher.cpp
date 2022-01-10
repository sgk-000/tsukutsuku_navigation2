#include "waypoint_publisher/waypoint_publisher.hpp"  // include local header

WayPointPublisher::WayPointPublisher() :
  Node("waypoint_publisher"), id_(0)
{
  csv_file_ = declare_parameter<std::string>("csv_file", "sample.csv");
  waypoint_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", rclcpp::QoS{10});
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

    while (getline(ifs, line)) {
        std::vector<std::string> strvec = getCSVLine(line, ',');
        visualization_msgs::msg::Marker marker;
        marker = origin_marker;
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
        marker_array.markers.push_back(marker);
    }
        std::cout << "publish" << std::endl;
        waypoint_pub_->publish(marker_array);
}
