#ifndef GREENHOUSE_EXPERIMENT_NODE_HPP
#define GREENHOUSE_EXPERIMENT_NODE_HPP

#include <memory>
#include <string>
#include <vector>
#include <tuple>
#include <map>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <sstream>
#include <thread>
#include <mutex>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/int32_multi_array.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>

// Forward declaration
namespace fs = std::filesystem;

struct RowBoundary {
  double y_min;
  double y_max;
  double x_min;
  double x_max;
};

struct PlantData {
  int id;
  double x;
  double y;
  int row_number;
  int index_in_row;
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Point detected_position;
  double confidence;
  std::string plant_type;
};

// Use PlantData as PlantInfo to avoid conflicts
using PlantInfo = PlantData;

struct ResultEntry {
  int    plant_id;
  bool   success;
  double duration_s;
  double error_m;
  double error_x;
  double error_y;
  int    experiment_type;
  bool   imu_enabled;
  int    param_n_val;
  std::string predetermined_list_val;
  bool   is_realtime_target;
  int    realtime_trigger_id;
};

struct AnomalyAlert {
    std::string type;
    std::string severity;
    geometry_msgs::msg::Point location;
    std::string description;
};

struct EnvironmentalReading {
    double temperature;
    double humidity;
    double light_level;
    rclcpp::Time timestamp;
};

struct EnvironmentalBaseline {
    double avg_temperature;
    double avg_humidity;
    double avg_light;
    size_t sample_count;
};

struct AnomalyInvestigationResult {
    AnomalyAlert anomaly;
    rclcpp::Time investigation_time;
    EnvironmentalReading environmental_data;
    std::vector<PlantInfo> affected_plants;
    bool follow_up_required;
};

struct Detection {
    cv::Point2f world_position;
    float confidence;
    std::string classification;
    std::string health_assessment;
    cv::Rect bounding_box;
};

#endif // GREENHOUSE_EXPERIMENT_NODE_HPP