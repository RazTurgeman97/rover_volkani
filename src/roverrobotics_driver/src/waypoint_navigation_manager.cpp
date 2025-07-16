#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class WaypointNavigationManager : public rclcpp::Node {
private:
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using WaypointClient = rclcpp_action::Client<FollowWaypoints>;
    
    WaypointClient::SharedPtr waypoint_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr detected_plants_sub_;
    
public:
    WaypointNavigationManager() : Node("waypoint_navigation_manager") {
        // Create action client for waypoint following
        waypoint_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
        
        // Subscribe to detected plants from your perception node
        detected_plants_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
            "/detected_plants", 10,
            std::bind(&WaypointNavigationManager::detected_plants_callback, this, std::placeholders::_1));
    }
    
    void detected_plants_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        // Convert detected plants to waypoints automatically
        navigate_to_detected_plants(msg->poses);
    }
    
    void navigate_to_detected_plants(const std::vector<geometry_msgs::msg::Pose>& plant_poses) {
        if (!waypoint_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
            return;
        }
        
        // Create waypoint goal
        FollowWaypoints::Goal goal;
        
        for (const auto& plant_pose : plant_poses) {
            geometry_msgs::msg::PoseStamped stamped_pose;
            stamped_pose.header.frame_id = "map";
            stamped_pose.header.stamp = now();
            stamped_pose.pose = plant_pose;
            goal.poses.push_back(stamped_pose);
        }
        
        // Set up callback options
        auto send_goal_options = WaypointClient::SendGoalOptions();
        
        send_goal_options.result_callback = 
            [this](const WaypointClient::WrappedResult & result) {
                switch(result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(get_logger(), "Navigation succeeded!");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(get_logger(), "Navigation was aborted");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(get_logger(), "Navigation was canceled");
                        break;
                    default:
                        RCLCPP_ERROR(get_logger(), "Unknown result code");
                        break;
                }
            };
        
        waypoint_client_->async_send_goal(goal, send_goal_options);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNavigationManager>());
    rclcpp::shutdown();
    return 0;
}