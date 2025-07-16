#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>

class PlantProtectionLayer : public nav2_costmap_2d::Layer {
public:
    void onInitialize() override {
        // Subscribe to detected plants
        plant_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseArray>(
            "/detected_plants", 1, 
            std::bind(&PlantProtectionLayer::plantsCallback, this, std::placeholders::_1));
        
        // Parameters
        nh_->declare_parameter("protection_radius", 0.3);
        nh_->get_parameter("protection_radius", protection_radius_);
    }
    
    void updateBounds(double robot_x, double robot_y, double robot_yaw,
                     double* min_x, double* min_y, double* max_x, double* max_y) override {
        // Update bounds to include all detected plants
        for (const auto& plant : detected_plants_) {
            *min_x = std::min(*min_x, plant.x - protection_radius_);
            *max_x = std::max(*max_x, plant.x + protection_radius_);
            *min_y = std::min(*min_y, plant.y - protection_radius_);
            *max_y = std::max(*max_y, plant.y + protection_radius_);
        }
    }
    
    void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                    int min_i, int min_j, int max_i, int max_j) override {
        // Add high cost around detected plants to protect them
        for (const auto& plant : detected_plants_) {
            unsigned int mx, my;
            if (master_grid.worldToMap(plant.x, plant.y, mx, my)) {
                // Create protection zone around plant
                int radius_cells = protection_radius_ / master_grid.getResolution();
                
                for (int dx = -radius_cells; dx <= radius_cells; dx++) {
                    for (int dy = -radius_cells; dy <= radius_cells; dy++) {
                        if (dx*dx + dy*dy <= radius_cells*radius_cells) {
                            unsigned int cell_x = mx + dx;
                            unsigned int cell_y = my + dy;
                            
                            if (cell_x < master_grid.getSizeInCellsX() && 
                                cell_y < master_grid.getSizeInCellsY()) {
                                // Set high cost but not lethal (allows approach)
                                master_grid.setCost(cell_x, cell_y, 200);
                            }
                        }
                    }
                }
            }
        }
    }
    
private:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr plant_sub_;
    std::vector<geometry_msgs::msg::Point> detected_plants_;
    double protection_radius_;

    void plantsCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr& msg) {
        detected_plants_.clear();
        for (const auto& pose : msg->poses) {
            detected_plants_.push_back(pose.position);
        }
    }
};