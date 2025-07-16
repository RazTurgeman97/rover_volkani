// src/roverrobotics_driver/src/greenhouse_experiment_node.cpp

#include <memory>
#include <string>
#include <vector>
#include <tuple>
#include <map>
#include <algorithm> // Required for std::sort
#include <cmath>
#include <chrono>
#include <sstream> // Required for std::stringstream
#include <thread>
#include <mutex>   // Required for std::mutex
#include <fstream>
#include <iomanip>
#include <filesystem>               // ← for directory iteration
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/int32_multi_array.hpp" // For experiment_type 5
#include "std_msgs/msg/string.hpp" // For dynamic plant detection
#include "std_srvs/srv/trigger.hpp"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "roverrobotics_driver/greenhouse_experiment_node.hpp"
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

using NavigateThroughPoses      = nav2_msgs::action::NavigateThroughPoses;
using G2P_Client               = rclcpp_action::Client<NavigateThroughPoses>;
using G2P_GoalHandle           = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

class GreenhouseExperimentNode : public rclcpp::Node
{
public:
  GreenhouseExperimentNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : Node("greenhouse_experiment_node", opts),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // --- 1) parameters
    declare_parameter<std::string>("plants_config","config/indoor_configs/plants.yaml");
    declare_parameter<int>("experiment_type",1);
    declare_parameter<int>("n",3); 
    declare_parameter<std::string>("infected_plants_list", ""); // For experiment_type 4, will be stored in predetermined_list_str_
    declare_parameter<bool>("use_imu", true); // For logging
    declare_parameter<double>("approach_dist",0.4);
    declare_parameter<double>("corridor_dist",0.8);
    // log_dir now under experiment_monitor/logs
    std::string default_log = 
      ament_index_cpp::get_package_share_directory("experiment_monitor") + "/logs";
    declare_parameter<std::string>("log_dir", default_log);

    // User confirmation parameter
    declare_parameter<bool>("require_confirmation", true);

    // Dynamic plant detection parameter
    declare_parameter<bool>("use_dynamic_detection", true);
    
    get_parameter("plants_config",    plants_config_rel_);
    get_parameter("experiment_type",  experiment_type_);
    get_parameter("n",                n_);
    get_parameter("infected_plants_list", predetermined_list_str_);
    get_parameter("use_imu", imu_enabled_);
    get_parameter("approach_dist",     approach_dist_);
    get_parameter("corridor_dist",     corridor_dist_);
    get_parameter("log_dir",           log_dir_);
    get_parameter("require_confirmation", require_confirmation_);
    get_parameter("use_dynamic_detection", use_dynamic_detection_);

    if (require_confirmation_) {
        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), "🤖 GREENHOUSE EXPERIMENT SYSTEM INITIALIZED");
        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), "Experiment Type: %d", experiment_type_);
        RCLCPP_INFO(get_logger(), "Use AI Perception: %s", use_dynamic_detection_ ? "YES" : "NO");
        RCLCPP_INFO(get_logger(), "IMU Enabled: %s", imu_enabled_ ? "YES" : "NO");
        RCLCPP_INFO(get_logger(), " ");
        RCLCPP_INFO(get_logger(), "📋 PRE-EXPERIMENT CHECKLIST:");
        RCLCPP_INFO(get_logger(), "   ✓ Robot is powered on and connected");
        RCLCPP_INFO(get_logger(), "   ✓ Navigation system is active"); 
        RCLCPP_INFO(get_logger(), "   ✓ Camera is working (check RViz)");
        RCLCPP_INFO(get_logger(), "   ✓ Robot is at starting position");
        RCLCPP_INFO(get_logger(), "   ✓ Greenhouse environment is clear");
        RCLCPP_INFO(get_logger(), " ");
        
        if (use_dynamic_detection_) {
            RCLCPP_INFO(get_logger(), "🔍 AI DETECTION MODE:");
            RCLCPP_INFO(get_logger(), "   ✓ YOLO model is loaded");
            RCLCPP_INFO(get_logger(), "   ✓ Plant detection pipeline is ready");
            RCLCPP_INFO(get_logger(), "   ✓ Adaptive navigation is enabled");
            RCLCPP_INFO(get_logger(), " ");
        }
        
        // Note: No longer calling wait_for_user_confirmation() here
        // The system will wait for service calls instead
    }

    // --- 2.0) Initialize subscriber for experiment 5
    if (experiment_type_ == 5) {
      infected_plants_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/infected_plant_notification", 10, // QoS depth 10
        std::bind(&GreenhouseExperimentNode::infected_plants_callback, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Experiment Type 5: Subscribed to /infected_plant_notification");
    }

    // --- 2) load plant positions
    auto pkg = ament_index_cpp::get_package_share_directory("roverrobotics_driver");
    std::string path = pkg + "/" + plants_config_rel_;
    RCLCPP_INFO(get_logger(),"Loading plant config: %s", path.c_str());
    YAML::Node cfg = YAML::LoadFile(path);

    // Load row boundaries
    RCLCPP_INFO(get_logger(),"Loading row boundaries:");
    for (auto const & row_node : cfg["row_boundaries"]) {
        std::string row_name = row_node.first.as<std::string>();
        RowBoundary boundary;
        boundary.y_min = row_node.second["y_min"].as<double>();
        boundary.y_max = row_node.second["y_max"].as<double>();
        boundary.x_min = row_node.second["x_min"].as<double>();
        boundary.x_max = row_node.second["x_max"].as<double>();
        row_boundaries_.push_back(boundary);
        RCLCPP_INFO(get_logger(),"  %s: y(%.2f, %.2f), x(%.2f, %.2f)",
                    row_name.c_str(), boundary.y_min, boundary.y_max, boundary.x_min, boundary.x_max);
    }

    // Load plant positions and initial data
    RCLCPP_INFO(get_logger(),"Loading plant data:");
    std::vector<PlantData> temp_plant_data_list;
    for (auto it : cfg["plants"]) {
      PlantData pd;
      pd.id = std::stoi(it.first.as<std::string>());
      pd.x = it.second["x"].as<double>();
      pd.y = it.second["y"].as<double>();
      pd.row_number = -1; // Placeholder, will be determined
      pd.index_in_row = -1; // Placeholder, will be determined

      pd.pose.header.frame_id = "map";
      pd.pose.header.stamp    = get_clock()->now();
      pd.pose.pose.position.x = pd.x;
      pd.pose.pose.position.y = pd.y;
      pd.pose.pose.orientation = to_quat(0.0);
      
      // Determine row number
      for (size_t i = 0; i < row_boundaries_.size(); ++i) {
        if (pd.y >= row_boundaries_[i].y_min && pd.y <= row_boundaries_[i].y_max) {
          pd.row_number = i + 1; // 1-indexed row number
          break;
        }
      }
      temp_plant_data_list.push_back(pd);
      RCLCPP_INFO(get_logger(),"  Plant %d @ (%.3f,%.3f) -> Row %d (initially)", pd.id, pd.x, pd.y, pd.row_number);
    }

    // Determine index_in_row for each plant
    for (size_t i = 1; i <= row_boundaries_.size(); ++i) { // Iterate through rows (1, 2, 3)
        std::vector<PlantData*> plants_in_row;
        for (auto& plant_data : temp_plant_data_list) {
            if (plant_data.row_number == static_cast<int>(i)) {
                plants_in_row.push_back(&plant_data);
            }
        }

        // Sort plants in the current row by x-coordinate
        std::sort(plants_in_row.begin(), plants_in_row.end(), [](const PlantData* a, const PlantData* b) {
            return a->x < b->x;
        });

        // Assign index_in_row
        for (size_t j = 0; j < plants_in_row.size(); ++j) {
            plants_in_row[j]->index_in_row = j;
        }
    }

    // Store plant data in the map
    for (const auto& pd : temp_plant_data_list) {
        plant_data_map_[pd.id] = pd;
        RCLCPP_INFO(get_logger(),"  Final Plant %d: Row %d, Index %d", pd.id, pd.row_number, pd.index_in_row);
    }


    // --- 2.1) set up directories
    fs::create_directories(log_dir_);
    experiment_base_dir_ = log_dir_ + "/experiment_" + std::to_string(experiment_type_);
    fs::create_directories(experiment_base_dir_);
    // next attempt index
    int attempt_idx = 1;
    for (auto & e : fs::directory_iterator(experiment_base_dir_)) {
      if (!e.is_directory()) continue;
      auto nm = e.path().filename().string();
      if (nm.rfind("attempt_",0)==0) {
        int idx = std::stoi(nm.substr(8));
        attempt_idx = std::max(attempt_idx, idx+1);
      }
    }
    attempt_dir_ = experiment_base_dir_ + "/attempt_" + std::to_string(attempt_idx);
    fs::create_directories(attempt_dir_);

    // --- 3) action client
    g2p_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
      this, "navigate_through_poses");
    RCLCPP_INFO(get_logger(),"Waiting for navigate_through_poses...");
    if (!g2p_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(),"Action server unavailable");
      return;
    }

    // --- 4) Setup based on confirmation requirement
    if (require_confirmation_) {
        // Create service for manual start
        start_experiment_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start_experiment",
            std::bind(&GreenhouseExperimentNode::start_experiment_callback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // Create status check service
        check_status_service_ = this->create_service<std_srvs::srv::Trigger>(
            "check_system_status",
            std::bind(&GreenhouseExperimentNode::check_status_callback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), "🤖 GREENHOUSE EXPERIMENT SYSTEM READY");
        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), "Experiment Type: %d", experiment_type_);
        RCLCPP_INFO(get_logger(), "Use AI Perception: %s", use_dynamic_detection_ ? "YES" : "NO");
        RCLCPP_INFO(get_logger(), "IMU Enabled: %s", imu_enabled_ ? "YES" : "NO");
        RCLCPP_INFO(get_logger(), " ");
        RCLCPP_INFO(get_logger(), "🎯 SYSTEM READY - WAITING FOR START COMMAND");
        RCLCPP_INFO(get_logger(), " ");
        RCLCPP_INFO(get_logger(), "To start the experiment, run:");
        RCLCPP_INFO(get_logger(), "  ros2 service call /start_experiment std_srvs/srv/Trigger");
        RCLCPP_INFO(get_logger(), " ");
        RCLCPP_INFO(get_logger(), "To check system status, run:");
        RCLCPP_INFO(get_logger(), "  ros2 service call /check_system_status std_srvs/srv/Trigger");
        RCLCPP_INFO(get_logger(), "==============================================");
        
        // DO NOT CALL run_experiment() here - wait for service call
        
    } else {
        // Auto-start if confirmation not required
        RCLCPP_INFO(get_logger(), "Auto-starting experiment (confirmation disabled)");
        // run_experiment();
    }
  } // End of constructor - DO NOT call run_experiment() after this

private:
  // yaw→quaternion
  geometry_msgs::msg::Quaternion to_quat(double yaw) {
    tf2::Quaternion q; q.setRPY(0,0,yaw);
    geometry_msgs::msg::Quaternion out;
    out.x=q.x(); out.y=q.y(); out.z=q.z(); out.w=q.w();
    return out;
  }

  // get freshest base_link→map pose
  geometry_msgs::msg::PoseStamped get_current_robot_pose() {
    geometry_msgs::msg::PoseStamped robot_pose;
    try {
        auto transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
        robot_pose.header = transform.header;
        robot_pose.pose.position.x = transform.transform.translation.x;
        robot_pose.pose.position.y = transform.transform.translation.y;
        robot_pose.pose.position.z = transform.transform.translation.z;
        robot_pose.pose.orientation = transform.transform.rotation;
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Failed to get current pose: %s", e.what());
        robot_pose.header.frame_id = "map";
        robot_pose.header.stamp = now();
    }
    return robot_pose;
  }

  // two‐step corridor+standoff, record & detect outliers later
  // Added is_rt_target and rt_trigger_id for experiment 5 logging
  void send_and_record(int id, double yaw, int side, bool is_rt_target = false, int rt_trigger_id = -1) {
    auto & plant_data = plant_data_map_.at(id); // Use .at() for bounds checking
    double px=plant_data.pose.pose.position.x, py=plant_data.pose.pose.position.y;
    // corridor
    double cx=px - std::cos(yaw)*corridor_dist_;
    double cy=py - std::sin(yaw)*corridor_dist_;
    // standoff
    double dx=-side*std::sin(yaw)*approach_dist_;
    double dy= side*std::cos(yaw)*approach_dist_;
    double sx=px+dx, sy=py+dy;

    std::vector<geometry_msgs::msg::PoseStamped> poses;
    for (auto [x,y] : {std::make_pair(cx,cy), std::make_pair(sx,sy)}) {
      auto p = plant_data.pose;
      p.pose.position.x = x;
      p.pose.position.y = y;
      p.pose.orientation = to_quat(yaw);
      poses.push_back(p);
    }

    NavigateThroughPoses::Goal goal; goal.poses = poses;
    auto t0 = now();
    bool succeeded=false;

    auto opts = G2P_Client::SendGoalOptions();
    opts.result_callback = [&](auto const & res){
      succeeded = (res.code==rclcpp_action::ResultCode::SUCCEEDED);
    };
    auto fut = g2p_client_->async_send_goal(goal, opts);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(),fut)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(),"send failed %d",id);
    } else if (!fut.get()) {
      RCLCPP_ERROR(get_logger(),"rejected %d",id);
    } else {
      auto rf = g2p_client_->async_get_result(fut.get());
      rclcpp::spin_until_future_complete(get_node_base_interface(),rf);
      succeeded = (rf.get().code==rclcpp_action::ResultCode::SUCCEEDED);
    }

    auto t1 = now();
    double dur = (t1-t0).seconds();

    auto actual = get_current_robot_pose();
    double ex = actual.pose.position.x - sx;
    double ey = actual.pose.position.y - sy;
    double err = std::hypot(ex,ey);

    RCLCPP_INFO(get_logger(),
      "Plant %d → %s in %.1fs (err=%.3fm [dx=%.3f,dy=%.3f])",
      id, succeeded?"OK":"FAIL", dur, err, ex, ey);

    ResultEntry entry;
    entry.plant_id = id;
    entry.success = succeeded;
    entry.duration_s = dur;
    entry.error_m = err;
    entry.error_x = ex;
    entry.error_y = ey;
    entry.experiment_type = experiment_type_;
    entry.imu_enabled = imu_enabled_;
    entry.param_n_val = (experiment_type_ == 3) ? n_ : -1;
    entry.predetermined_list_val = (experiment_type_ == 4) ? predetermined_list_str_ : "";
    entry.is_realtime_target = (experiment_type_ == 5) ? is_rt_target : false;
    entry.realtime_trigger_id = (experiment_type_ == 5) ? rt_trigger_id : -1;
    
    results_.push_back(entry);
    std::this_thread::sleep_for(std::chrono::seconds(3));
  }

  void send_home() {
    geometry_msgs::msg::PoseStamped home;
    home.header.frame_id="map";
    home.header.stamp = get_clock()->now();
    home.pose.position.x=0;
    home.pose.position.y=0;
    home.pose.orientation=to_quat(0);
    NavigateThroughPoses::Goal g; g.poses={home};
    auto fut = g2p_client_->async_send_goal(g,{});
    rclcpp::spin_until_future_complete(get_node_base_interface(),fut);
  }

  void run_experiment() {
    // snake order
    std::vector<std::tuple<int,double,int>> snake;
    // snake order - using plant IDs directly
    // Row 1 (plants 1-5), approach from +Y
    // Row 2 (plants 10-6), approach from -Y
    // Row 3 (plants 11-15), approach from +Y
    std::vector<std::tuple<int,double,int>> snake_pattern_visits;
    for(int i=1;i<=5;++i)  snake_pattern_visits.emplace_back(i,    0.0,+1);
    for(int i=10;i>=6;--i) snake_pattern_visits.emplace_back(i, M_PI,-1);
    for(int i=11;i<=15;++i)snake_pattern_visits.emplace_back(i,    0.0,+1);

    std::vector<std::tuple<int,double,int>> visits_to_make;
    switch(experiment_type_) {
      case 1: // All plants in snake order
        visits_to_make = snake_pattern_visits;
        break;
      case 2: // Every 2nd plant in snake order
        for(size_t i=0; i<snake_pattern_visits.size(); i+=2) {
          visits_to_make.push_back(snake_pattern_visits[i]);
        }
        break;
      // case 3 was (every 3rd plant hardcoded), now is Nth plant
      case 3: // Every Nth plant in snake order (n_ is a ROS param)
        RCLCPP_INFO(get_logger(), "Experiment Type 3: Visiting every %d-th plant in snake order.", n_);
        for(size_t i=0; i<snake_pattern_visits.size(); i+=std::max(1,n_)) {
          visits_to_make.push_back(snake_pattern_visits[i]);
        }
        break;
      case 4: // Pre-determined infected plants, but in snake pattern order
        RCLCPP_INFO(get_logger(), "Experiment Type 4: Visiting pre-determined infected plants (snake order): %s", predetermined_list_str_.c_str());
        {
          // Parse the infected plant IDs into a set for fast lookup
          std::set<int> infected_set;
          std::stringstream ss(predetermined_list_str_);
          std::string item;
          while (std::getline(ss, item, ',')) {
            try {
              infected_set.insert(std::stoi(item));
            } catch (const std::exception& e) {
              RCLCPP_ERROR(get_logger(), "Invalid plant ID in infected_plants_list: %s", item.c_str());
            }
          }
          // Visit only the infected plants, but in the same snake pattern order and with the same yaw/side as other experiments
          for (const auto& visit : snake_pattern_visits) {
            int plant_id = std::get<0>(visit);
            if (infected_set.count(plant_id)) {
              visits_to_make.push_back(visit);
            }
          }
        }
        break;
      default:
        RCLCPP_WARN(get_logger(),"Experiment type %d not implemented",experiment_type_);
    }

    if (require_confirmation_ && !visits_to_make.empty()) {
        // Replace empty string with space
        RCLCPP_INFO(get_logger(), " ");
        RCLCPP_INFO(get_logger(), "📋 EXPERIMENT PLAN:");
        RCLCPP_INFO(get_logger(), "   Total plants to visit: %zu", visits_to_make.size());
        for (size_t i = 0; i < visits_to_make.size(); ++i) {
            auto [id, yaw, side] = visits_to_make[i];
            RCLCPP_INFO(get_logger(), "   %zu. Plant %d (yaw: %.2f, side: %d)", 
                       i+1, id, yaw, side);
        }
        
        std::string input;
        std::cout << "\nPress ENTER to start visiting plants...";
        std::cin.ignore(); // Clear any remaining input
        std::getline(std::cin, input);
    }

    if (experiment_type_ == 5) {
        RCLCPP_INFO(get_logger(), "Experiment Type 5: Starting real-time processing loop.");
        while(rclcpp::ok()) {
            std::vector<int> plants_to_process;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (!real_time_infected_plants_queue_.empty()) {
                    // Process all plants currently in the queue in one go
                    plants_to_process.swap(real_time_infected_plants_queue_);
                }
            }

            if (!plants_to_process.empty()) {
                for (int infected_plant_id : plants_to_process) {
                    RCLCPP_INFO(get_logger(), "Processing real-time infected plant ID: %d", infected_plant_id);
                    if (!plant_data_map_.count(infected_plant_id)) {
                        RCLCPP_WARN(get_logger(), "Received infected plant ID %d not in map.", infected_plant_id);
                        continue;
                    }

                    PlantInfo p_info = get_plant_info(infected_plant_id);
                    double yaw = 0.0;
                    int side = 0;
                    determine_yaw_side_for_plant(p_info, yaw, side);
                    
                    if (side == 0) { // Side would be 0 if row_number was invalid
                        RCLCPP_WARN(get_logger(), "Could not determine yaw/side for plant %d (row %d). Skipping.", infected_plant_id, p_info.row_number);
                        continue;
                    }
                    // For Exp 5, primary target's trigger ID is itself
                    send_and_record(infected_plant_id, yaw, side, true, infected_plant_id);

                    // Navigate to neighbors
                    int current_row = p_info.row_number;
                    int current_idx = p_info.index_in_row;

                    std::vector<int> neighbor_ids;
                    // Previous neighbor
                    if (current_idx > 0) {
                        int prev_neighbor_id = get_plant_id_by_row_index(current_row, current_idx - 1);
                        if (prev_neighbor_id != -1) neighbor_ids.push_back(prev_neighbor_id);
                    }
                    // Next neighbor
                    // Need to know max index for the row. Assuming 5 plants per row (0-4)
                    // This should be made more robust if row sizes can vary significantly
                    int max_index_for_row = -1;
                    if (current_row == 1) max_index_for_row = 4; // plants 1-5
                    else if (current_row == 2) max_index_for_row = 4; // plants 6-10 (indices 0-4 for this row)
                    else if (current_row == 3) max_index_for_row = 4; // plants 11-15

                    if (current_idx < max_index_for_row) {
                         int next_neighbor_id = get_plant_id_by_row_index(current_row, current_idx + 1);
                         if (next_neighbor_id != -1) neighbor_ids.push_back(next_neighbor_id);
                    }

                    for (int neighbor_id : neighbor_ids) {
                        RCLCPP_INFO(get_logger(), "Processing neighbor %d of plant %d", neighbor_id, infected_plant_id);
                        PlantInfo neighbor_info = get_plant_info(neighbor_id);
                        determine_yaw_side_for_plant(neighbor_info, yaw, side);
                         if (side == 0) {
                            RCLCPP_WARN(get_logger(), "Could not determine yaw/side for neighbor %d (row %d). Skipping.", neighbor_id, neighbor_info.row_number);
                            continue;
                        }
                        // For Exp 5, neighbor's trigger ID is the primary infected plant
                        send_and_record(neighbor_id, yaw, side, false, infected_plant_id);
                    }
                }
            } else {
                // Short sleep if queue is empty to avoid busy spinning
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            // Check if shutdown is requested (e.g. Ctrl-C)
            if (!rclcpp::ok()) {
                break;
            }
        }
        RCLCPP_INFO(get_logger(), "Experiment Type 5: Real-time processing loop finished.");

    } else { // For experiment types 1, 2, 3, 4
        for (size_t i = 0; i < visits_to_make.size(); ++i) {
            auto [id, yaw, side] = visits_to_make[i];
            
            if (require_confirmation_) {
                // Replace empty string with space
                RCLCPP_INFO(get_logger(), " ");
                RCLCPP_INFO(get_logger(), "🎯 Next: Plant %d (%zu/%zu)", id, i+1, visits_to_make.size());
                std::string input;
                std::cout << "Press ENTER to visit plant " << id << " (or 'skip' to skip): ";
                std::getline(std::cin, input);
                
                if (input == "skip") {
                    RCLCPP_WARN(get_logger(), "⏭️  Skipping Plant %d", id);
                    continue;
                }
            }
            
            send_and_record(id, yaw, side);
        }
    }
    
    // write per‐attempt CSV
    auto attempt_csv = attempt_dir_ + "/results.csv";
    std::ofstream csv(attempt_csv);
    csv << "plant,success,duration_s,error_m,error_x,error_y,experiment_type,imu_enabled,param_n,predetermined_list,is_realtime_target,realtime_trigger_id\n";
    for (auto &r: results_) {
      csv << r.plant_id << ','
          << (r.success?1:0) << ','
          << std::fixed<<std::setprecision(3)<<r.duration_s<<','
          << std::fixed<<std::setprecision(3)<<r.error_m<<','
          << std::fixed<<std::setprecision(3)<<r.error_x<<','
          << std::fixed<<std::setprecision(3)<<r.error_y<<','
          << r.experiment_type << ','
          << (r.imu_enabled?1:0) << ','
          << r.param_n_val << ','
          << "\"" << r.predetermined_list_val << "\"" << ',' // Enclose string in quotes
          << (r.is_realtime_target?1:0) << ','
          << r.realtime_trigger_id << "\n";
    }
    RCLCPP_INFO(get_logger(),"Wrote attempt log → %s", attempt_csv.c_str());

    // outlier detection
    double sum=0, sum2=0; int N=results_.size();
    for (auto &r: results_) { sum+=r.duration_s; sum2+=r.duration_s*r.duration_s; }
    double mean = sum/N;
    double stdv = std::sqrt(sum2/N - mean*mean);
    double thresh = mean + 2*stdv;
    for (auto &r: results_) {
      if (r.duration_s > thresh) {
        RCLCPP_WARN(get_logger(),
          "DURATION OUTLIER plant %d: %.3fs > %.3fs (mean+2σ)",
          r.plant_id, r.duration_s, thresh);
      }
    }

    // update cross‐attempt comparison
    auto comp_csv = experiment_base_dir_ + "/comparison.csv";
    std::ofstream comp(comp_csv, std::ios::trunc);
    comp << "attempt,plant,success,duration_s,error_m,error_x,error_y,experiment_type,imu_enabled,param_n,predetermined_list,is_realtime_target,realtime_trigger_id\n";
    for (auto & e : fs::directory_iterator(experiment_base_dir_)) {
      if (!e.is_directory()) continue;
      std::string nm = e.path().filename().string();
      if (nm.rfind("attempt_",0)!=0) continue;
      int idx=std::stoi(nm.substr(8));
      auto f = e.path()/"results.csv";
      if (fs::exists(f)) { // Check if results.csv exists for this attempt
        std::ifstream ifs(f);
        std::string line; 
        if (std::getline(ifs,line)) { // Skip header of the individual attempt's results.csv
            while(std::getline(ifs,line)) {
              if (!line.empty()) { // Ensure line is not empty before prepending attempt index
                comp << idx << "," << line << "\n";
              }
            }
        }
      }
    }
    RCLCPP_INFO(get_logger(),"Updated comparison.csv → %s", comp_csv.c_str());

    send_home();
    RCLCPP_INFO(get_logger(),"Experiment done, returned home.");

    rclcpp::shutdown();
  }

  // members
  std::string plants_config_rel_, log_dir_, predetermined_list_str_; // Renamed from infected_plants_list_str_
  std::string experiment_base_dir_, attempt_dir_;
  bool   imu_enabled_; // Added
  int    experiment_type_{1}, n_{3};
  double approach_dist_{0.4}, corridor_dist_{0.8};

  std::vector<RowBoundary> row_boundaries_;
  std::map<int, PlantData> plant_data_map_; // Stores all plant data, including row and index
  G2P_Client::SharedPtr g2p_client_;
  tf2_ros::Buffer       tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<ResultEntry> results_;

  // For Experiment 5
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr infected_plants_sub_;
  std::vector<int> real_time_infected_plants_queue_;
  std::mutex queue_mutex_;

  // For dynamic plant detection
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dynamic_plants_sub_;
  bool use_dynamic_detection_;
  bool plants_loaded_dynamically_;
  
  // For environmental monitoring and anomaly detection
  std::queue<int> priority_inspection_queue_;
  bool experiment_active_ = true;
  std::vector<PlantInfo> historical_plant_data_;
  EnvironmentalBaseline environmental_baseline_;

  void infected_plants_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    for (int plant_id : msg->data) {
      RCLCPP_INFO(get_logger(), "Received infected plant notification: %d", plant_id);
      // Optional: check if plant_id is valid before adding
      if (plant_data_map_.count(plant_id)) {
        real_time_infected_plants_queue_.push_back(plant_id);
      } else {
        RCLCPP_WARN(get_logger(), "Received infected plant_id %d not in known plant map.", plant_id);
      }
    }
  }

  void dynamic_plants_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (!use_dynamic_detection_) return;
        
        try {
            // Parse the YAML string
            YAML::Node dynamic_config = YAML::Load(msg->data);
            
            // Clear existing plant data
            plant_data_map_.clear();
            row_boundaries_.clear();
            
            // Load row boundaries
            for (const auto& row_node : dynamic_config["row_boundaries"]) {
                std::string row_name = row_node.first.as<std::string>();
                RowBoundary boundary;
                boundary.y_min = row_node.second["y_min"].as<double>();
                boundary.y_max = row_node.second["y_max"].as<double>();
                boundary.x_min = row_node.second["x_min"].as<double>();
                boundary.x_max = row_node.second["x_max"].as<double>();
                row_boundaries_.push_back(boundary);
            }
            
            // Load plant data
            for (const auto& plant_node : dynamic_config["plants"]) {
                PlantData pd;
                pd.id = std::stoi(plant_node.first.as<std::string>());
                pd.x = plant_node.second["x"].as<double>();
                pd.y = plant_node.second["y"].as<double>();
                pd.row_number = plant_node.second["row"].as<int>();
                pd.index_in_row = plant_node.second["index_in_row"].as<int>();
                pd.confidence = plant_node.second["confidence"].as<double>();
                
                pd.pose.header.frame_id = "map";
                pd.pose.header.stamp = get_clock()->now();
                pd.pose.pose.position.x = pd.x;
                pd.pose.pose.position.y = pd.y;
                pd.pose.pose.orientation = to_quat(0.0);
                
                plant_data_map_[pd.id] = pd;
                
                RCLCPP_INFO(get_logger(), "Dynamic Plant %d: (%.3f, %.3f) Row %d, Index %d, Conf %.2f", 
                           pd.id, pd.x, pd.y, pd.row_number, pd.index_in_row, pd.confidence);
            }
            
            plants_loaded_dynamically_ = true;
            RCLCPP_INFO(get_logger(), "Loaded %zu plants dynamically", plant_data_map_.size());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to parse dynamic plant data: %s", e.what());
        }
    }

  // Plant Helper Utility
  PlantInfo get_plant_info(int plant_id) {
    if (plant_data_map_.count(plant_id)) {
        return plant_data_map_.at(plant_id);
    } else {
        RCLCPP_ERROR(get_logger(), "Plant with ID %d not found in get_plant_info.", plant_id);
        // Create a proper invalid PlantData object
        PlantData invalid_plant;
        invalid_plant.id = -1;
        invalid_plant.x = 0.0;
        invalid_plant.y = 0.0;
        invalid_plant.row_number = -1;
        invalid_plant.index_in_row = -1;
        invalid_plant.pose = geometry_msgs::msg::PoseStamped();
        invalid_plant.detected_position = geometry_msgs::msg::Point();
        invalid_plant.confidence = 0.0;
        invalid_plant.plant_type = "invalid";
        return invalid_plant;
    }
}

  // Helper to get plant ID by row and index within that row
  int get_plant_id_by_row_index(int row_num, int index_in_row) {
    for (const auto& pair : plant_data_map_) {
      const PlantData& pd = pair.second;
      if (pd.row_number == row_num && pd.index_in_row == index_in_row) {
        return pd.id;
      }
    }
    RCLCPP_WARN(get_logger(), "Plant not found for row %d, index %d", row_num, index_in_row);
    return -1; // Not found
  }
  
  // Helper to determine yaw and side for a given plant
  void determine_yaw_side_for_plant(const PlantInfo& p_info, double& yaw, int& side) {
      yaw = 0.0; // default
      side = 0;  // default (indicates error or unknown)
      if (p_info.row_number == 1) {
          yaw = 0.0;
          side = 1;
      } else if (p_info.row_number == 2) {
          yaw = M_PI;
          side = -1;
      } else if (p_info.row_number == 3) {
          yaw = 0.0;
          side = 1;
      } else {
          RCLCPP_WARN(get_logger(), "Plant ID %d has unknown row_number %d. Cannot determine yaw/side.", p_info.id, p_info.row_number);
      }
  }

  // Helper functions for enhanced experiment modes
  void send_disease_alert(int plant_id) {
      RCLCPP_WARN(get_logger(), "DISEASE ALERT: Plant %d requires immediate attention", plant_id);
      // Could integrate with external alert systems here
  }
  
  std::vector<PlantInfo> wait_for_ai_detections(std::chrono::seconds timeout) {
      // Placeholder - would wait for AI detection results
      std::vector<PlantInfo> detections;
      rclcpp::sleep_for(timeout);
      return detections;
  }
  
  void navigate_to_ai_detected_plant(const PlantInfo& plant_info) {
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.header.frame_id = "map";
      target_pose.header.stamp = now();
      target_pose.pose.position = plant_info.detected_position;
      target_pose.pose.orientation.w = 1.0;
      
      navigate_to_pose_with_retry(target_pose);
  }
  
  bool navigate_to_pose_with_retry(const geometry_msgs::msg::PoseStamped& target_pose, int max_retries = 3) {
      for (int attempt = 0; attempt < max_retries; ++attempt) {
          // Use your existing navigation code here
          std::vector<geometry_msgs::msg::PoseStamped> poses = {target_pose};
          
          NavigateThroughPoses::Goal goal;
          goal.poses = poses;
          
          auto future = g2p_client_->async_send_goal(goal);
          if (rclcpp::spin_until_future_complete(get_node_base_interface(), future) 
              == rclcpp::FutureReturnCode::SUCCESS) {
              
              auto goal_handle = future.get();
              if (goal_handle) {
                  auto result_future = g2p_client_->async_get_result(goal_handle);
                  if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future)
                      == rclcpp::FutureReturnCode::SUCCESS) {
                      
                      auto result = result_future.get();
                      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                          return true;
                      }
                  }
              }
          }
          
          RCLCPP_WARN(get_logger(), "Navigation attempt %d failed, retrying...", attempt + 1);
          rclcpp::sleep_for(std::chrono::seconds(2));
      }
      
      return false;
  }
  
  EnvironmentalBaseline establish_environmental_baseline() {
      EnvironmentalBaseline baseline;
      baseline.avg_temperature = 22.0; // Default values
      baseline.avg_humidity = 60.0;
      baseline.avg_light = 500.0;
      baseline.sample_count = 100;
      return baseline;
  }
  
  EnvironmentalReading get_current_environmental_data() {
      EnvironmentalReading reading;
      reading.temperature = 22.5; // Would read from actual sensors
      reading.humidity = 58.0;
      reading.light_level = 520.0;
      reading.timestamp = now();
      return reading;
  }
  
  geometry_msgs::msg::Point get_current_position() {
      auto pose = get_current_robot_pose();
      return pose.pose.position;
  }
  
  void send_anomaly_alert(const AnomalyAlert& alert) {
      RCLCPP_ERROR(get_logger(), "ANOMALY ALERT: %s - %s at (%.2f, %.2f)", 
                  alert.type.c_str(), alert.description.c_str(),
                  alert.location.x, alert.location.y);
  }
  
  std::vector<PlantInfo> find_plants_near_position(const geometry_msgs::msg::Point& position, double radius) {
      std::vector<PlantInfo> nearby_plants;
      
      for (const auto& [plant_id, plant_data] : plant_data_map_) {
          double distance = std::hypot(
              plant_data.pose.pose.position.x - position.x,
              plant_data.pose.pose.position.y - position.y
          );
          
          if (distance <= radius) {
              PlantInfo info;
              info.detected_position = plant_data.pose.pose.position;
              info.confidence = 1.0;
              info.plant_type = "existing_plant";
              nearby_plants.push_back(info);
          }
      }
      
      return nearby_plants;
  }
  
  void load_historical_monitoring_data() {
      // Load previous monitoring data from files
      RCLCPP_INFO(get_logger(), "Loading historical monitoring data...");
      // Implementation would load from CSV or database
  }
  
  void save_anomaly_investigation_result(const AnomalyInvestigationResult& result) {
      // Save investigation results to file
      RCLCPP_INFO(get_logger(), "Saving anomaly investigation result...");
      // Implementation would save to CSV or database
  }
  
  bool require_confirmation_;
  bool experiment_started_ = false;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_experiment_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr check_status_service_;
  
  
  void wait_for_user_confirmation() {
        std::string input;
        bool confirmed = false;
        
        while (!confirmed && rclcpp::ok()) {
            // Replace empty string with space
            RCLCPP_INFO(get_logger(), " ");
            RCLCPP_INFO(get_logger(), "🎯 READY TO START EXPERIMENT?");
            RCLCPP_INFO(get_logger(), "   Type 'start' to begin experiment");
            RCLCPP_INFO(get_logger(), "   Type 'check' to review system status");
            RCLCPP_INFO(get_logger(), "   Type 'abort' to cancel");
            // Replace empty string with space
            RCLCPP_INFO(get_logger(), " ");
            std::cout << "Enter command: ";
            std::cin >> input;
            
            // Convert to lowercase
            std::transform(input.begin(), input.end(), input.begin(), ::tolower);
            
            if (input == "start" || input == "s") {
                confirmed = true;
                // Replace empty string with space
                RCLCPP_INFO(get_logger(), " ");
                RCLCPP_INFO(get_logger(), "🚀 EXPERIMENT CONFIRMED - STARTING NOW!");
                RCLCPP_INFO(get_logger(), "==============================================");
                std::this_thread::sleep_for(std::chrono::seconds(2));
                
            } else if (input == "check" || input == "c") {
                perform_system_check();
                
            } else if (input == "abort" || input == "a") {
                // Replace empty string with space
                RCLCPP_WARN(get_logger(), " ");
                RCLCPP_WARN(get_logger(), "❌ EXPERIMENT ABORTED BY USER");
                RCLCPP_WARN(get_logger(), "==============================================");
                rclcpp::shutdown();
                return;
                
            } else {
                RCLCPP_WARN(get_logger(), "❓ Unknown command: '%s'. Please try again.", input.c_str());
            }
        }
    }
    
    void perform_system_check() {
        // Replace empty string with space
        RCLCPP_INFO(get_logger(), " ");
        RCLCPP_INFO(get_logger(), "🔍 PERFORMING SYSTEM CHECK...");
        RCLCPP_INFO(get_logger(), "================================");
        
        // Check action client
        bool nav_ready = g2p_client_->wait_for_action_server(std::chrono::seconds(1));
        RCLCPP_INFO(get_logger(), "   Navigation Server: %s", nav_ready ? "✅ READY" : "❌ NOT READY");
        
        // Check if we can get robot pose
        bool pose_available = false;
        try {
            auto pose = get_current_robot_pose();
            pose_available = true;
            RCLCPP_INFO(get_logger(), "   Robot Localization: ✅ READY (%.2f, %.2f)", 
                      pose.pose.position.x, pose.pose.position.y);
        } catch (const std::exception& e) {
            RCLCPP_INFO(get_logger(), "   Robot Localization: ❌ NOT READY");
        }
        
        // Check plant data
        if (use_dynamic_detection_) {
            RCLCPP_INFO(get_logger(), "   Plant Detection: %s", plants_loaded_dynamically_ ? "READY" : "WAITING");
            if (plants_loaded_dynamically_) {
                RCLCPP_INFO(get_logger(), "   Detected Plants: %zu", plant_data_map_.size());
            }
        } else {
            RCLCPP_INFO(get_logger(), "   Plant Configuration: LOADED (%zu plants)", plant_data_map_.size());
        }
        
        // Overall status
        bool system_ready = nav_ready && pose_available && 
                           (use_dynamic_detection_ ? plants_loaded_dynamically_ : !plant_data_map_.empty());
        
        RCLCPP_INFO(get_logger(), "   Overall Status: %s", system_ready ? "✅ SYSTEM READY" : "❌ SYSTEM NOT READY");
        RCLCPP_INFO(get_logger(), "================================");
    }
    
    void start_experiment_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
      (void)request; // Suppress unused parameter warning
    
      if (experiment_started_) {
          response->success = false;
          response->message = "Experiment already started or completed";
          return;
      }
      
      RCLCPP_INFO(get_logger(), "🚀 Starting experiment via service call...");
      experiment_started_ = true;
      
      // Run experiment in a separate thread to avoid blocking the service
      std::thread experiment_thread([this]() {
        run_experiment();
      });
      experiment_thread.detach();
      
      response->success = true;
      response->message = "Experiment started successfully";
  }

  void check_status_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    
    std::stringstream status_msg;
    
    // Check navigation server
    bool nav_ready = g2p_client_->wait_for_action_server(std::chrono::seconds(1));
    status_msg << "Navigation Server: " << (nav_ready ? "READY" : "NOT READY") << "\n";
    
    // Check robot pose
    bool pose_available = false;
    try {
        auto pose = get_current_robot_pose();
        pose_available = true;
        status_msg << "Robot Localization: READY (" << std::fixed << std::setprecision(2) 
                  << pose.pose.position.x << ", " << pose.pose.position.y << ")\n";
    } catch (const std::exception& e) {
        status_msg << "Robot Localization: NOT READY\n";
    }
    
    // Check plant data
    if (use_dynamic_detection_) {
        status_msg << "Plant Detection: " << (plants_loaded_dynamically_ ? "READY" : "WAITING") << "\n";
        if (plants_loaded_dynamically_) {
            status_msg << "Detected Plants: " << plant_data_map_.size() << "\n";
        }
    } else {
        status_msg << "Plant Configuration: LOADED (" << plant_data_map_.size() << " plants)\n";
    }
    
    // Overall status
    bool system_ready = nav_ready && pose_available && 
                       (use_dynamic_detection_ ? plants_loaded_dynamically_ : !plant_data_map_.empty());
    
    status_msg << "Overall Status: " << (system_ready ? "SYSTEM READY" : "SYSTEM NOT READY");
    
    response->success = system_ready;
    response->message = status_msg.str();
    
    RCLCPP_INFO(get_logger(), "System Status Check:\n%s", status_msg.str().c_str());
}
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GreenhouseExperimentNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}