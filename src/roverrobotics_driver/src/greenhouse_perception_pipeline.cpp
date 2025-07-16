#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

struct PlantDetection {
    cv::Point2f pixel_location;
    float confidence;
    std::string health_status;
    cv::Rect bounding_box;
};

struct PlantInfo {
    geometry_msgs::msg::Point world_position;
    std::string health_status;
    float confidence;
    rclcpp::Time last_seen;
    bool lidar_confirmed;
};

class GreenhousePerceptionPipeline : public rclcpp::Node {
public:
    GreenhousePerceptionPipeline() : Node("greenhouse_perception_pipeline") {
        // Initialize subscribers
        camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&GreenhousePerceptionPipeline::camera_callback, this, std::placeholders::_1));
        
        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&GreenhousePerceptionPipeline::lidar_callback, this, std::placeholders::_1));
        
        // Initialize publishers
        plants_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/detected_plants", 10);
        diseased_plants_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/diseased_plants", 10);
        
        initialize_models();
        
        RCLCPP_INFO(get_logger(), "Greenhouse Perception Pipeline initialized");
    }
    
private:
    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr plants_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr diseased_plants_pub_;
    
    // Data storage for sensor fusion
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    std::vector<PlantDetection> camera_detections_;
    std::vector<PlantInfo> fused_plants_;
    
    void initialize_models() {
        // Initialize AI models for plant detection and health assessment
        RCLCPP_INFO(get_logger(), "Initializing AI models...");
    }
    
    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat image = cv_ptr->image;
            
            // Detect plants in image
            camera_detections_ = detect_plants_in_image(msg);
            
            // Classify plant health
            classify_plant_health(camera_detections_);
            
            // Fuse with LiDAR data if available
            if (latest_scan_) {
                fuse_camera_lidar_data();
            }
            
            // Publish results
            publish_detection_results();
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Camera callback error: %s", e.what());
        }
    }
    
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
        
        // Use LiDAR for obstacle detection and plant height estimation
        process_lidar_data(msg);
    }
    
    std::vector<PlantDetection> detect_plants_in_image(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::vector<PlantDetection> detections;
        
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat image = cv_ptr->image;
            
            // Use traditional color-based detection
            auto traditional_detections = run_traditional_detection(image);
            
            // Convert to PlantDetection format
            for (const auto& point : traditional_detections) {
                PlantDetection detection;
                detection.pixel_location = point;
                detection.confidence = 0.8f;
                detection.health_status = "unknown";
                detection.bounding_box = cv::Rect(point.x - 25, point.y - 25, 50, 50);
                detections.push_back(detection);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Plant detection error: %s", e.what());
        }
        
        return detections;
    }
    
    void classify_plant_health(std::vector<PlantDetection>& detections) {
        // Classify health status of detected plants
        for (auto& detection : detections) {
            detection.health_status = classify_health_status(detection.bounding_box);
        }
    }
    
    void fuse_camera_lidar_data() {
        // Combine camera and LiDAR data for better plant localization
        fused_plants_.clear();
        
        for (const auto& camera_detection : camera_detections_) {
            PlantInfo plant_info;
            plant_info.world_position = pixel_to_world(camera_detection.pixel_location);
            plant_info.health_status = camera_detection.health_status;
            plant_info.confidence = camera_detection.confidence;
            plant_info.last_seen = now();
            plant_info.lidar_confirmed = false; // Would verify with LiDAR data
            
            fused_plants_.push_back(plant_info);
        }
    }
    
    void process_lidar_data(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        // Process LiDAR data for plant height estimation and obstacle detection
        // Implementation would analyze scan data
    }
    
    void publish_detection_results() {
        // Publish detected plants
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = now();
        pose_array.header.frame_id = "map";
        
        for (const auto& plant : fused_plants_) {
            geometry_msgs::msg::Pose pose;
            pose.position = plant.world_position;
            pose.orientation.w = 1.0;
            pose_array.poses.push_back(pose);
        }
        
        plants_pub_->publish(pose_array);
        
        // Publish diseased plants
        std_msgs::msg::Int32MultiArray diseased_msg;
        int plant_id = 0;
        for (const auto& plant : fused_plants_) {
            if (plant.health_status == "diseased" || plant.health_status == "pest_damage") {
                diseased_msg.data.push_back(plant_id);
            }
            plant_id++;
        }
        
        if (!diseased_msg.data.empty()) {
            diseased_plants_pub_->publish(diseased_msg);
        }
    }
    
    std::vector<cv::Point2f> run_traditional_detection(const cv::Mat& image) {
        std::vector<cv::Point2f> plant_locations;
        
        // Convert to HSV for color-based detection
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        
        // Define green color range for plants
        cv::Scalar lower_green(35, 50, 50);
        cv::Scalar upper_green(85, 255, 255);
        
        // Create mask for green regions
        cv::Mat mask;
        cv::inRange(hsv, lower_green, upper_green, mask);
        
        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // Extract centers
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 500) {
                cv::Moments moments = cv::moments(contour);
                if (moments.m00 != 0) {
                    cv::Point2f center(moments.m10 / moments.m00, moments.m01 / moments.m00);
                    plant_locations.push_back(center);
                }
            }
        }
        
        return plant_locations;
    }
    
    std::string classify_health_status(const cv::Rect& plant_region) {
        // Simple health classification based on color analysis
        // In practice, this would use a trained AI model
        return "healthy"; // Placeholder
    }
    
    geometry_msgs::msg::Point pixel_to_world(const cv::Point2f& pixel) {
        geometry_msgs::msg::Point world_point;
        // Simple conversion - replace with proper camera calibration
        world_point.x = pixel.x * 0.01;
        world_point.y = pixel.y * 0.01;
        world_point.z = 0.0;
        return world_point;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GreenhousePerceptionPipeline>());
    rclcpp::shutdown();
    return 0;
}