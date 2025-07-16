#include "roverrobotics_driver/greenhouse_experiment_node.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cmath>

class AdaptiveGreenhouseNavigator : public rclcpp::Node {
public:
    AdaptiveGreenhouseNavigator() : Node("adaptive_greenhouse_navigator") {
        // Subscribe to real-time plant detections
        plant_detection_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
            "/detected_plants", 10,
            std::bind(&AdaptiveGreenhouseNavigator::plants_detected_callback, this, std::placeholders::_1));
            
        // Subscribe to diseased plant notifications
        diseased_plants_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "/infected_plant_notification", 10,
            std::bind(&AdaptiveGreenhouseNavigator::diseased_plants_callback, this, std::placeholders::_1));
            
        // Navigation action client
        nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
            this, "navigate_through_poses");
            
        // Timer for periodic replanning
        replan_timer_ = create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&AdaptiveGreenhouseNavigator::replan_if_needed, this));
            
        RCLCPP_INFO(get_logger(), "Adaptive Greenhouse Navigator initialized");
    }
    
private:
    // Current navigation state
    std::vector<geometry_msgs::msg::PoseStamped> planned_waypoints_;
    std::vector<int> priority_plants_; // Diseased plants get priority
    std::map<int, geometry_msgs::msg::Point> known_plants_;
    bool navigation_in_progress_ = false;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr plant_detection_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr diseased_plants_sub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr replan_timer_;
    
    void plants_detected_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        // Update known plants with new detections
        update_plant_database(msg);
        
        // If not currently navigating, start adaptive navigation
        if (!navigation_in_progress_) {
            start_adaptive_navigation();
        }
    }
    
    void diseased_plants_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        // Update priority list with diseased plants
        priority_plants_ = msg->data;
        
        // Replan to prioritize diseased plants
        replan_for_priority_plants();
    }
    
    void update_plant_database(const geometry_msgs::msg::PoseArray::SharedPtr detected_plants) {
        for (size_t i = 0; i < detected_plants->poses.size(); ++i) {
            int plant_id = static_cast<int>(i); // You might want a better ID system
            known_plants_[plant_id] = detected_plants->poses[i].position;
        }
        
        RCLCPP_INFO(get_logger(), "Updated plant database: %zu plants known", known_plants_.size());
    }
    
    void start_adaptive_navigation() {
        if (known_plants_.empty()) {
            RCLCPP_WARN(get_logger(), "No plants detected yet, waiting...");
            return;
        }
        
        // Generate optimal path through all known plants
        auto waypoints = generate_optimal_path();
        
        if (!waypoints.empty()) {
            execute_navigation_plan(waypoints);
        }
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> generate_optimal_path() {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        
        // Step 1: Prioritize diseased plants
        std::vector<int> ordered_plants;
        
        // Add diseased plants first
        for (int diseased_id : priority_plants_) {
            if (known_plants_.find(diseased_id) != known_plants_.end()) {
                ordered_plants.push_back(diseased_id);
            }
        }
        
        // Add remaining plants using traveling salesman approximation
        for (const auto& [plant_id, position] : known_plants_) {
            if (std::find(ordered_plants.begin(), ordered_plants.end(), plant_id) == ordered_plants.end()) {
                ordered_plants.push_back(plant_id);
            }
        }
        
        // Step 2: Generate waypoints with approach poses
        for (int plant_id : ordered_plants) {
            auto plant_pos = known_plants_[plant_id];
            
            // Create approach pose (offset for safety)
            geometry_msgs::msg::PoseStamped approach_waypoint;
            approach_waypoint.header.frame_id = "map";
            approach_waypoint.header.stamp = now();
            
            // Approach from the south (adjust based on your greenhouse layout)
            approach_waypoint.pose.position.x = plant_pos.x;
            approach_waypoint.pose.position.y = plant_pos.y - 0.5; // 0.5m south
            approach_waypoint.pose.position.z = 0.0;
            
            // Face the plant
            approach_waypoint.pose.orientation = create_quaternion_from_yaw(M_PI/2); // Face north
            
            waypoints.push_back(approach_waypoint);
        }
        
        return waypoints;
    }
    
    void execute_navigation_plan(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints) {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "Navigation action server not available");
            return;
        }
        
        nav2_msgs::action::NavigateThroughPoses::Goal goal;
        goal.poses = waypoints;
        goal.behavior_tree = ""; // Use default behavior tree
        
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
        
        send_goal_options.goal_response_callback = 
            [this](auto) { 
                navigation_in_progress_ = true;
                RCLCPP_INFO(get_logger(), "Adaptive navigation started");
            };
            
        send_goal_options.feedback_callback = 
            [this](auto, const auto& feedback) {
                RCLCPP_INFO(get_logger(), "Visiting waypoint at pose: [x: %f, y: %f, z: %f]", 
                            feedback->current_pose.pose.position.x, 
                            feedback->current_pose.pose.position.y, 
                            feedback->current_pose.pose.position.z);
                
                // Trigger plant inspection at each waypoint
                trigger_plant_inspection(feedback->current_pose);
            };
            
        send_goal_options.result_callback = 
            [this](const auto& result) {
                navigation_in_progress_ = false;
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(get_logger(), "Adaptive navigation completed successfully!");
                } else {
                    RCLCPP_WARN(get_logger(), "Navigation failed, replanning...");
                    // Could trigger replanning here
                }
            };
        
        planned_waypoints_ = waypoints;
        nav_client_->async_send_goal(goal, send_goal_options);
    }
    
    void replan_for_priority_plants() {
        if (navigation_in_progress_) {
            // Cancel current navigation
            nav_client_->async_cancel_all_goals();
            navigation_in_progress_ = false;
            
            // Replan with new priorities
            rclcpp::sleep_for(std::chrono::seconds(1)); // Brief pause
            start_adaptive_navigation();
        }
    }
    
    void replan_if_needed() {
        // Periodic check for new plants or changed priorities
        // This could also check for navigation failures and trigger recovery
        
        if (!navigation_in_progress_ && !known_plants_.empty()) {
            // Check if we have new plants since last planning
            static size_t last_known_count = 0;
            if (known_plants_.size() != last_known_count) {
                RCLCPP_INFO(get_logger(), "New plants detected, replanning...");
                start_adaptive_navigation();
                last_known_count = known_plants_.size();
            }
        }
    }
    
    void trigger_plant_inspection(const geometry_msgs::msg::PoseStamped& current_pose) {
        // This would integrate with your existing experiment node
        // Could publish a message or call a service to start inspection
        
        RCLCPP_INFO(get_logger(), "Triggering inspection at pose: [x: %f, y: %f, z: %f]", 
                    current_pose.pose.position.x, 
                    current_pose.pose.position.y, 
                    current_pose.pose.position.z);
        
        // Example: publish inspection command
        // inspection_trigger_pub_->publish(inspection_msg);
    }
    
    geometry_msgs::msg::Quaternion create_quaternion_from_yaw(double yaw) {
        geometry_msgs::msg::Quaternion q;
        q.x = 0.0;
        q.y = 0.0;
        q.z = sin(yaw / 2.0);
        q.w = cos(yaw / 2.0);
        return q;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdaptiveGreenhouseNavigator>());
    rclcpp::shutdown();
    return 0;
}