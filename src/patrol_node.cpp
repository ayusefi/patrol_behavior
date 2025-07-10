#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PatrolNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  PatrolNode() : Node("patrol_node"), current_waypoint_idx_(0)
  {
    client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");
    RCLCPP_INFO(this->get_logger(), "PatrolNode started. Waiting for Nav2 action server...");
    load_waypoints();
    wait_for_server_and_start();
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  size_t current_waypoint_idx_;

  void load_waypoints()
  {
    std::string yaml_path = this->declare_parameter<std::string>("waypoints_yaml", "waypoints.yaml");
    try {
      YAML::Node config = YAML::LoadFile(yaml_path);
      for (const auto & wp : config["waypoints"]) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = wp["frame_id"].as<std::string>("map");
        pose.pose.position.x = wp["x"].as<double>();
        pose.pose.position.y = wp["y"].as<double>();
        pose.pose.position.z = wp["z"].as<double>(0.0);
        pose.pose.orientation.x = wp["qx"].as<double>(0.0);
        pose.pose.orientation.y = wp["qy"].as<double>(0.0);
        pose.pose.orientation.z = wp["qz"].as<double>(0.0);
        pose.pose.orientation.w = wp["qw"].as<double>(1.0);
        waypoints_.push_back(pose);
      }
      RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints.", waypoints_.size());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints: %s", e.what());
    }
  }

  void wait_for_server_and_start()
  {
    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available after waiting");
      rclcpp::shutdown();
      return;
    }
    send_next_waypoint();
  }

  void send_next_waypoint()
  {
    if (current_waypoint_idx_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "Patrol complete. Restarting patrol.");
      current_waypoint_idx_ = 0;
    }
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = waypoints_[current_waypoint_idx_];
    goal_msg.pose.header.stamp = this->now();
    RCLCPP_INFO(this->get_logger(), "Sending waypoint %zu", current_waypoint_idx_ + 1);
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", current_waypoint_idx_ + 1);
        current_waypoint_idx_++;
        send_next_waypoint();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to reach waypoint %zu", current_waypoint_idx_ + 1);
        // Optionally retry or skip
        current_waypoint_idx_++;
        send_next_waypoint();
      }
    };
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
