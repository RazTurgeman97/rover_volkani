// src/roverrobotics_driver/src/greenhouse_experiment_node.cpp

#include <memory>
#include <string>
#include <vector>
#include <tuple>
#include <map>
#include <cmath>
#include <chrono>
#include <thread>
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

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

using NavigateThroughPoses      = nav2_msgs::action::NavigateThroughPoses;
using G2P_Client               = rclcpp_action::Client<NavigateThroughPoses>;
using G2P_GoalHandle           = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

struct ResultEntry {
  int    plant_id;
  bool   success;
  double duration_s;
  double error_m;
  double error_x;
  double error_y;
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
    declare_parameter<int>("n",4);
    declare_parameter<double>("approach_dist",0.4);
    declare_parameter<double>("corridor_dist",0.8);
    // log_dir now under experiment_monitor/logs
    std::string default_log = 
      ament_index_cpp::get_package_share_directory("experiment_monitor") + "/logs";
    declare_parameter<std::string>("log_dir", default_log);

    get_parameter("plants_config",    plants_config_rel_);
    get_parameter("experiment_type",  experiment_type_);
    get_parameter("n",                n_);
    get_parameter("approach_dist",     approach_dist_);
    get_parameter("corridor_dist",     corridor_dist_);
    get_parameter("log_dir",           log_dir_);

    // --- 2) load plant positions
    auto pkg = ament_index_cpp::get_package_share_directory("roverrobotics_driver");
    std::string path = pkg + "/" + plants_config_rel_;
    RCLCPP_INFO(get_logger(),"Loading plant config: %s", path.c_str());
    YAML::Node cfg = YAML::LoadFile(path);
    for (auto it : cfg["plants"]) {
      int id = std::stoi(it.first.as<std::string>());
      double x = it.second["x"].as<double>();
      double y = it.second["y"].as<double>();
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.header.stamp    = get_clock()->now();
      ps.pose.position.x = x;
      ps.pose.position.y = y;
      ps.pose.orientation = to_quat(0.0);
      plant_poses_[id] = ps;
      RCLCPP_INFO(get_logger(),"  Plant %d @ (%.3f,%.3f)", id, x, y);
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
  void send_and_record(int id, double yaw, int side) {
    auto & plant = plant_poses_[id];
    double px=plant.pose.position.x, py=plant.pose.position.y;
    // corridor
    double cx=px - std::cos(yaw)*corridor_dist_;
    double cy=py - std::sin(yaw)*corridor_dist_;
    // standoff
    double dx=-side*std::sin(yaw)*approach_dist_;
    double dy= side*std::cos(yaw)*approach_dist_;
    double sx=px+dx, sy=py+dy;

    std::vector<geometry_msgs::msg::PoseStamped> poses;
    for (auto [x,y] : {std::make_pair(cx,cy), std::make_pair(sx,sy)}) {
      auto p = plant;
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

    results_.push_back({ id, succeeded, dur, err, ex, ey });
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
    for(int i=1;i<=5;++i)  snake.emplace_back(i,    0.0,+1);
    for(int i=10;i>=6;--i)  snake.emplace_back(i, M_PI,-1);
    for(int i=11;i<=15;++i) snake.emplace_back(i,    0.0,+1);

    std::vector<size_t> idxs;
    switch(experiment_type_) {
      case 1: idxs.resize(snake.size()); std::iota(idxs.begin(),idxs.end(),0); break;
      case 2: for(size_t i=0;i<snake.size();i+=2) idxs.push_back(i); break;
      case 3: for(size_t i=0;i<snake.size();i+=3) idxs.push_back(i); break;
      case 4: for(size_t i=0;i<snake.size();i+=std::max(1,n_)) idxs.push_back(i); break;
      default: RCLCPP_WARN(get_logger(),"Exp%d not implemented",experiment_type_);
    }

    for (auto i : idxs) {
      auto [id,yaw,side] = snake[i];
      send_and_record(id,yaw,side);
    }

    // write per‐attempt CSV
    auto attempt_csv = attempt_dir_ + "/results.csv";
    std::ofstream csv(attempt_csv);
    csv << "plant,success,duration_s,error_m,error_x,error_y\n";
    for (auto &r: results_) {
      csv << r.plant_id << ','
          << (r.success?1:0) << ','
          << std::fixed<<std::setprecision(3)<<r.duration_s<<','
          << std::fixed<<std::setprecision(3)<<r.error_m<<','
          << std::fixed<<std::setprecision(3)<<r.error_x<<','
          << std::fixed<<std::setprecision(3)<<r.error_y<<"\n";
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
    comp << "attempt,plant,success,duration_s,error_m,error_x,error_y\n";
    for (auto & e : fs::directory_iterator(experiment_base_dir_)) {
      if (!e.is_directory()) continue;
      std::string nm = e.path().filename().string();
      if (nm.rfind("attempt_",0)!=0) continue;
      int idx=std::stoi(nm.substr(8));
      auto f = e.path()/"results.csv";
      std::ifstream ifs(f);
      std::string line; std::getline(ifs,line);
      while(std::getline(ifs,line)) comp<<idx<<","<<line<<"\n";
    }
    RCLCPP_INFO(get_logger(),"Updated comparison.csv → %s", comp_csv.c_str());

    send_home();
    RCLCPP_INFO(get_logger(),"Experiment done, returned home.");

    rclcpp::shutdown();
  }

  // members
  std::string plants_config_rel_, log_dir_;
  std::string experiment_base_dir_, attempt_dir_;
  int    experiment_type_{1}, n_{4};
  double approach_dist_{0.4}, corridor_dist_{0.8};

  std::map<int,geometry_msgs::msg::PoseStamped> plant_poses_;
  G2P_Client::SharedPtr g2p_client_;
  tf2_ros::Buffer       tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<ResultEntry> results_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GreenhouseExperimentNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
