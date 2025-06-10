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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/int32_multi_array.hpp" // For experiment_type 5

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

using NavigateThroughPoses      = nav2_msgs::action::NavigateThroughPoses;
using G2P_Client               = rclcpp_action::Client<NavigateThroughPoses>;
using G2P_GoalHandle           = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

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
  geometry_msgs::msg::PoseStamped pose; // Keep original pose for navigation
};

struct ResultEntry {
  int    plant_id;
  bool   success;
  double duration_s;
  double error_m;
  double error_x;
  double error_y;
  // New fields
  int    experiment_type;
  bool   imu_enabled;
  int    param_n_val; // Value of n for Nth plant exp, else -1
  std::string predetermined_list_val; // For Exp 4, else ""
  bool   is_realtime_target; // For Exp 5
  int    realtime_trigger_id; // For Exp 5, else -1
};

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

    get_parameter("plants_config",    plants_config_rel_);
    get_parameter("experiment_type",  experiment_type_);
    get_parameter("n",                n_);
    get_parameter("infected_plants_list", predetermined_list_str_); // Renamed for clarity in logging
    get_parameter("use_imu", imu_enabled_); // Added
    get_parameter("approach_dist",     approach_dist_);
    get_parameter("corridor_dist",     corridor_dist_);
    get_parameter("log_dir",           log_dir_);

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
      rclcpp::shutdown();
      return;
    }

    // --- 4) run
    run_experiment();
  }

private:
  // yaw→quaternion
  geometry_msgs::msg::Quaternion to_quat(double yaw) {
    tf2::Quaternion q; q.setRPY(0,0,yaw);
    geometry_msgs::msg::Quaternion out;
    out.x=q.x(); out.y=q.y(); out.z=q.z(); out.w=q.w();
    return out;
  }

  // get freshest base_link→map pose
  geometry_msgs::msg::PoseStamped current_pose() {
    geometry_msgs::msg::PoseStamped ps, out;
    ps.header.frame_id = "base_link";
    ps.header.stamp.sec = 0; ps.header.stamp.nanosec = 0;  // ask for latest
    ps.pose.orientation = to_quat(0.0);
    tf_buffer_.transform(ps, out, "map", std::chrono::milliseconds(100));
    return out;
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

    auto actual = current_pose();
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
      case 4: // Pre-determined infected plants
        RCLCPP_INFO(get_logger(), "Experiment Type 4: Visiting pre-determined infected plants list: %s", predetermined_list_str_.c_str());
        {
          std::vector<int> infected_plants;
          std::stringstream ss(predetermined_list_str_);
          std::string item;
          while (std::getline(ss, item, ',')) {
            try {
              infected_plants.push_back(std::stoi(item));
            } catch (const std::invalid_argument& ia) {
              RCLCPP_ERROR(get_logger(), "Invalid plant ID in infected_plants_list: %s", item.c_str());
            } catch (const std::out_of_range& oor) {
              RCLCPP_ERROR(get_logger(), "Plant ID out of range in infected_plants_list: %s", item.c_str());
            }
          }

          for (int plant_id : infected_plants) {
            if (plant_data_map_.count(plant_id)) {
              PlantInfo p_info = get_plant_info(plant_id);
              double yaw = 0.0;
              int side = 0;
              // Determine yaw and side based on row
              // Row 1 (plants 1-5): yaw = 0.0, side = +1
              // Row 2 (plants 6-10): yaw = M_PI, side = -1
              // Row 3 (plants 11-15): yaw = 0.0, side = +1
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
                RCLCPP_WARN(get_logger(), "Plant ID %d has unknown row_number %d. Cannot determine yaw/side.", plant_id, p_info.row_number);
                continue; // Skip this plant
              }
              visits_to_make.emplace_back(plant_id, yaw, side);
            } else {
              RCLCPP_WARN(get_logger(), "Infected plant ID %d not found in plant_data_map_.", plant_id);
            }
          }
        }
        break;
      default:
        RCLCPP_WARN(get_logger(),"Experiment type %d not implemented",experiment_type_);
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
        for (const auto& visit_params : visits_to_make) {
          auto [id,yaw,side] = visit_params;
          send_and_record(id,yaw,side);
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

  // Plant Helper Utility
  using PlantInfo = PlantData; 

  PlantInfo get_plant_info(int plant_id) {
    if (plant_data_map_.count(plant_id)) {
      return plant_data_map_.at(plant_id);
    } else {
      RCLCPP_ERROR(get_logger(), "Plant with ID %d not found in get_plant_info.", plant_id);
      return PlantData{-1, 0.0, 0.0, -1, -1, geometry_msgs::msg::PoseStamped()}; // Return invalid data
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

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GreenhouseExperimentNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}