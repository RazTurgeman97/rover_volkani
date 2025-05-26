#include <memory>
#include <string>
#include <map>
#include <vector>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class GreenhouseExperimentNode : public rclcpp::Node
{
public:
  GreenhouseExperimentNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("greenhouse_experiment_node", options),
    tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    // 1) Declare & read parameters
    this->declare_parameter<std::string>("plants_config", "config/indoor_configs/plants.yaml");
    this->declare_parameter<int>("experiment_type", 1);
    this->get_parameter("plants_config", plants_config_rel_);
    this->get_parameter("experiment_type", experiment_type_);

    // 2) Load plant coordinates from YAML
    auto pkg_share = ament_index_cpp::get_package_share_directory("roverrobotics_driver");
    std::string config_path = pkg_share + "/" + plants_config_rel_;
    RCLCPP_INFO(this->get_logger(), "Loading plant config from: %s", config_path.c_str());
    loadPlantConfig(config_path);

    // 3) Create Nav2 action client
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 'navigate_to_pose' action server...");
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Connected to Nav2 action server.");

    // 4) Execute chosen experiment
    run_experiment();
  }

private:
  // Load the plant positions from the YAML file into plant_poses_
  void loadPlantConfig(const std::string & path)
  {
    YAML::Node cfg = YAML::LoadFile(path);
    for (auto it = cfg["plants"].begin(); it != cfg["plants"].end(); ++it) {
      int id = std::stoi(it->first.as<std::string>());
      double x = it->second["x"].as<double>();
      double y = it->second["y"].as<double>();

      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.header.stamp = get_clock()->now();
      ps.pose.position.x = x;
      ps.pose.position.y = y;
      ps.pose.position.z = 0.0;
      // face forward along +X
      ps.pose.orientation.z = 0.0;
      ps.pose.orientation.w = 1.0;

      plant_poses_[id] = ps;
      RCLCPP_INFO(this->get_logger(),
        "  Loaded Plant %d → (%.3f, %.3f)", id, x, y);
    }
  }

  // Helper: send a single NavigateToPose goal and wait for result + 3s pause
  void send_and_wait(int id)
  {
    auto it = plant_poses_.find(id);
    if (it == plant_poses_.end()) {
      RCLCPP_ERROR(this->get_logger(), "No pose for Plant %d", id);
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = it->second;
    goal_msg.pose.header.stamp = get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "→ Sending goal for Plant %d", id);
    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.result_callback = [this, id](const GoalHandleNavigate::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(),
          "Reached Plant %d successfully", id);
      } else {
        RCLCPP_WARN(this->get_logger(),
          "Failed to reach Plant %d (code %d)", id, (int)result.code);
      }
    };

    // send goal
    auto goal_handle_future = nav_client_->async_send_goal(goal_msg, options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
          goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),
        "send_goal failed for Plant %d", id);
      return;
    }
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(),
        "Goal was rejected for Plant %d", id);
      return;
    }

    // wait for result
    auto result_future = nav_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
          result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),
        "get_result failed for Plant %d", id);
      return;
    }

    // pause for inspection
    RCLCPP_INFO(this->get_logger(),
      "Inspecting Plant %d for 3 seconds...", id);
    std::this_thread::sleep_for(std::chrono::seconds(3));
  }

  // Main dispatcher for the experiments
  void run_experiment()
  {
    if (experiment_type_ == 1) {
      RCLCPP_INFO(this->get_logger(), "Experiment 1: Stop at every plant");
      for (int id = 1; id <= 15; ++id) {
        send_and_wait(id);
      }

    } else if (experiment_type_ == 2) {
      RCLCPP_INFO(this->get_logger(), "Experiment 2: Stop at every 2nd plant");
      for (int id = 1; id <= 15; id += 2) {
        send_and_wait(id);
      }

    } else {
      RCLCPP_WARN(this->get_logger(),
        "experiment_type %d not implemented yet", experiment_type_);
    }

    RCLCPP_INFO(this->get_logger(),
      "Experiment %d complete, shutting down.", experiment_type_);
    rclcpp::shutdown();
  }

private:
  std::string plants_config_rel_;
  int experiment_type_{1};

  std::map<int, geometry_msgs::msg::PoseStamped> plant_poses_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GreenhouseExperimentNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
