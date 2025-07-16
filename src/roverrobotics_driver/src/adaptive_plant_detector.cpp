#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

struct DetectedPlant {
    int id;
    int row_number;
    int index_in_row;
    geometry_msgs::msg::Point position;
    double confidence;
    std::chrono::steady_clock::time_point first_detected;
    std::chrono::steady_clock::time_point last_updated;
    int detection_count;
    bool is_stable;
    double approach_yaw;
    int approach_side;
};

struct RowConfig {
    int id;
    double y_min, y_max;
    double x_min, x_max;
    int expected_plants;
    int numbering_start;
    double approach_yaw;
    int approach_side;
};

class AdaptivePlantDetector : public rclcpp::Node
{
public:
    AdaptivePlantDetector() : Node("adaptive_plant_detector"),
                              tf_buffer_(this->get_clock()),
                              tf_listener_(tf_buffer_)
    {
        // Load configuration
        load_detection_config();
        
        // Publishers
        detected_plants_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/detected_plants_yaml", 10);
        plant_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/plant_detection_markers", 10);
        plant_list_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/detected_plant_ids", 10);
            
        // Subscribers
        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/realsense/color/image_raw", 10,
            std::bind(&AdaptivePlantDetector::image_callback, this, std::placeholders::_1));
            
        // Timer for periodic plant map generation
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&AdaptivePlantDetector::publish_plant_map, this));
            
        RCLCPP_INFO(get_logger(), "Adaptive Plant Detector started");
    }

private:
    void load_detection_config() {
        auto pkg = ament_index_cpp::get_package_share_directory("roverrobotics_driver");
        std::string config_path = pkg + "/config/plant_detection_config.yaml";
        
        try {
            YAML::Node config = YAML::LoadFile(config_path);
            
            for (const auto& row_node : config["plant_detection"]["rows"]) {
                RowConfig row;
                row.id = row_node["id"].as<int>();
                auto y_range = row_node["y_range"].as<std::vector<double>>();
                auto x_range = row_node["x_range"].as<std::vector<double>>();
                row.y_min = y_range[0];
                row.y_max = y_range[1];
                row.x_min = x_range[0];
                row.x_max = x_range[1];
                row.expected_plants = row_node["expected_plants"].as<int>();
                row.numbering_start = row_node["numbering_start"].as<int>();
                row.approach_yaw = row_node["approach_yaw"].as<double>();
                row.approach_side = row_node["approach_side"].as<int>();
                
                row_configs_.push_back(row);
                RCLCPP_INFO(get_logger(), "Loaded row %d: y(%.2f-%.2f), plants %d-%d", 
                           row.id, row.y_min, row.y_max, row.numbering_start, 
                           row.numbering_start + row.expected_plants - 1);
            }
            
            auto detection_config = config["plant_detection"]["detection"];
            confidence_threshold_ = detection_config["confidence_threshold"].as<double>();
            clustering_distance_ = detection_config["clustering_distance"].as<double>();
            min_detections_per_plant_ = detection_config["min_detections_per_plant"].as<int>();
            stabilization_time_ = std::chrono::duration<double>(
                detection_config["stabilization_time"].as<double>());
                
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to load detection config: %s", e.what());
        }
    }
    
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // TODO: Implement YOLO detection here
        // For now, simulate detection with some example plants
        simulate_plant_detection();
    }
    
    void simulate_plant_detection() {
        // Simulate detecting plants at known positions for testing
        static bool simulated = false;
        if (!simulated) {
            // Row 1 plants
            add_detected_plant(1.0, 1.0, 0.8);
            add_detected_plant(2.0, 1.0, 0.9);
            add_detected_plant(3.0, 1.0, 0.85);
            add_detected_plant(4.0, 1.0, 0.75);
            add_detected_plant(5.0, 1.0, 0.8);
            
            // Row 2 plants  
            add_detected_plant(5.0, 2.3, 0.8);
            add_detected_plant(4.0, 2.3, 0.9);
            add_detected_plant(3.0, 2.3, 0.85);
            add_detected_plant(2.0, 2.3, 0.75);
            add_detected_plant(1.0, 2.3, 0.8);
            
            // Row 3 plants
            add_detected_plant(1.0, 4.0, 0.8);
            add_detected_plant(2.0, 4.0, 0.9);
            add_detected_plant(3.0, 4.0, 0.85);
            add_detected_plant(4.0, 4.0, 0.75);
            add_detected_plant(5.0, 4.0, 0.8);
            
            simulated = true;
            RCLCPP_INFO(get_logger(), "Simulated plant detection complete");
        }
    }
    
    void add_detected_plant(double x, double y, double confidence) {
        geometry_msgs::msg::Point position;
        position.x = x;
        position.y = y;
        position.z = 0.0;
        
        // Find which row this detection belongs to
        int row_id = -1;
        RowConfig* row_config = nullptr;
        for (auto& row : row_configs_) {
            if (y >= row.y_min && y <= row.y_max && x >= row.x_min && x <= row.x_max) {
                row_id = row.id;
                row_config = &row;
                break;
            }
        }
        
        if (row_config == nullptr) {
            RCLCPP_WARN(get_logger(), "Plant at (%.2f, %.2f) doesn't belong to any configured row", x, y);
            return;
        }
        
        // Check if this is close to an existing detection
        for (auto& plant : detected_plants_) {
            double distance = std::hypot(plant.position.x - x, plant.position.y - y);
            if (distance < clustering_distance_) {
                // Update existing plant
                plant.confidence = std::max(plant.confidence, confidence);
                plant.detection_count++;
                plant.last_updated = std::chrono::steady_clock::now();
                
                // Check if plant is now stable
                auto time_since_first = plant.last_updated - plant.first_detected;
                if (!plant.is_stable && time_since_first >= stabilization_time_ && 
                    plant.detection_count >= min_detections_per_plant_) {
                    plant.is_stable = true;
                    RCLCPP_INFO(get_logger(), "Plant %d stabilized at (%.2f, %.2f)", plant.id, plant.position.x, plant.position.y);
                }
                return;
            }
        }
        
        // Create new plant detection
        DetectedPlant plant;
        plant.row_number = row_id;
        plant.position = position;
        plant.confidence = confidence;
        plant.first_detected = std::chrono::steady_clock::now();
        plant.last_updated = plant.first_detected;
        plant.detection_count = 1;
        plant.is_stable = false;
        plant.approach_yaw = row_config->approach_yaw;
        plant.approach_side = row_config->approach_side;
        
        detected_plants_.push_back(plant);
        
        // Assign IDs based on row and position
        assign_plant_ids();
    }
    
    void assign_plant_ids() {
        // Group plants by row and sort by x-coordinate within each row
        for (auto& row_config : row_configs_) {
            std::vector<DetectedPlant*> plants_in_row;
            
            for (auto& plant : detected_plants_) {
                if (plant.row_number == row_config.id) {
                    plants_in_row.push_back(&plant);
                }
            }
            
            // Sort by x-coordinate (left to right for rows 1&3, right to left for row 2)
            if (row_config.id == 2) {
                // Row 2: sort right to left (descending x)
                std::sort(plants_in_row.begin(), plants_in_row.end(),
                         [](const DetectedPlant* a, const DetectedPlant* b) {
                             return a->position.x > b->position.x;
                         });
            } else {
                // Rows 1&3: sort left to right (ascending x)
                std::sort(plants_in_row.begin(), plants_in_row.end(),
                         [](const DetectedPlant* a, const DetectedPlant* b) {
                             return a->position.x < b->position.x;
                         });
            }
            
            // Assign IDs
            for (size_t i = 0; i < plants_in_row.size(); ++i) {
                plants_in_row[i]->id = row_config.numbering_start + i;
                plants_in_row[i]->index_in_row = i;
            }
        }
    }
    
    void publish_plant_map() {
        // Only publish stable plants
        std::vector<DetectedPlant> stable_plants;
        for (const auto& plant : detected_plants_) {
            if (plant.is_stable) {
                stable_plants.push_back(plant);
            }
        }
        
        if (stable_plants.empty()) {
            return;
        }
        
        // Generate YAML string for the experiment node
        std::stringstream yaml_stream;
        yaml_stream << "plants:\n";
        
        for (const auto& plant : stable_plants) {
            yaml_stream << "  " << plant.id << ":\n";
            yaml_stream << "    x: " << std::fixed << std::setprecision(3) << plant.position.x << "\n";
            yaml_stream << "    y: " << std::fixed << std::setprecision(3) << plant.position.y << "\n";
            yaml_stream << "    row: " << plant.row_number << "\n";
            yaml_stream << "    index_in_row: " << plant.index_in_row << "\n";
            yaml_stream << "    approach_yaw: " << plant.approach_yaw << "\n";
            yaml_stream << "    approach_side: " << plant.approach_side << "\n";
            yaml_stream << "    confidence: " << plant.confidence << "\n";
        }
        
        // Add row boundaries to the YAML
        yaml_stream << "row_boundaries:\n";
        for (const auto& row : row_configs_) {
            yaml_stream << "  row_" << row.id << ":\n";
            yaml_stream << "    y_min: " << row.y_min << "\n";
            yaml_stream << "    y_max: " << row.y_max << "\n";
            yaml_stream << "    x_min: " << row.x_min << "\n";
            yaml_stream << "    x_max: " << row.x_max << "\n";
        }
        
        std_msgs::msg::String yaml_msg;
        yaml_msg.data = yaml_stream.str();
        detected_plants_pub_->publish(yaml_msg);
        
        // Publish plant IDs list
        std_msgs::msg::Int32MultiArray plant_ids_msg;
        for (const auto& plant : stable_plants) {
            plant_ids_msg.data.push_back(plant.id);
        }
        plant_list_pub_->publish(plant_ids_msg);
        
        // Publish visualization markers
        publish_plant_markers(stable_plants);
        
        RCLCPP_INFO(get_logger(), "Published %zu stable plants", stable_plants.size());
    }
    
    void publish_plant_markers(const std::vector<DetectedPlant>& plants) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (size_t i = 0; i < plants.size(); ++i) {
            const auto& plant = plants[i];
            
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = now();
            marker.ns = "detected_plants";
            marker.id = plant.id;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position = plant.position;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.5;
            
            // Color based on row
            if (plant.row_number == 1) {
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; // Red
            } else if (plant.row_number == 2) {
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; // Green
            } else {
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; // Blue
            }
            marker.color.a = 0.7;
            
            marker_array.markers.push_back(marker);
            
            // Add text marker for plant ID
            visualization_msgs::msg::Marker text_marker;
            text_marker.header = marker.header;
            text_marker.ns = "plant_ids";
            text_marker.id = plant.id + 1000;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            text_marker.pose.position = plant.position;
            text_marker.pose.position.z += 0.6;
            text_marker.pose.orientation.w = 1.0;
            
            text_marker.scale.z = 0.3;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            
            text_marker.text = "Plant " + std::to_string(plant.id);
            
            marker_array.markers.push_back(text_marker);
        }
        
        plant_markers_pub_->publish(marker_array);
    }

    // Member variables
    std::vector<RowConfig> row_configs_;
    std::vector<DetectedPlant> detected_plants_;
    
    double confidence_threshold_;
    double clustering_distance_;
    int min_detections_per_plant_;
    std::chrono::duration<double> stabilization_time_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detected_plants_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plant_markers_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr plant_list_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdaptivePlantDetector>());
    rclcpp::shutdown();
    return 0;
}