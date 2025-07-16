#ifndef ROVERROBOTICS_DRIVER__PLANT_DETECTION_NODE_HPP_
#define ROVERROBOTICS_DRIVER__PLANT_DETECTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Forward declarations
struct Detection {
    cv::Rect bounding_box;
    float confidence;
    int class_id;
    std::string class_name;
    std::string health_status;
};

struct PlantDetection {
    cv::Rect bounding_box;
    float confidence;
    std::string class_name;
    std::string health_status;
};

struct ROIRegion {
    double x_min, x_max, y_min, y_max;
};

struct PlantDetectionConfig {
    double min_detection_area = 500.0;
    double max_detection_area = 10000.0;
    bool color_filter_enabled = true;
    bool shape_filter_enabled = true;
    bool roi_filter_enabled = false;
    int hue_range[2] = {35, 85};
    int saturation_min = 50;
    int value_min = 50;
    double min_circularity = 0.4;
    std::vector<ROIRegion> roi_regions;
};

class PlantDetectionNode : public rclcpp::Node
{
public:
    PlantDetectionNode();
    ~PlantDetectionNode() = default;

private:
    // Core detection methods
    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    std::vector<Detection> detect_plants_yolo(const cv::Mat& image);
    std::vector<cv::Point2f> detect_plants_traditional(const cv::Mat& image);
    std::vector<PlantDetection> filterPlantDetections(
        const std::vector<cv::Rect>& raw_detections,
        const cv::Mat& image,
        const PlantDetectionConfig& config);
    
    // Plant validation methods
    bool isGreenPlant(const cv::Mat& roi, const PlantDetectionConfig& config);
    bool hasPlantShape(const cv::Mat& roi, const PlantDetectionConfig& config);
    bool isInPlantArea(const cv::Rect& detection, const PlantDetectionConfig& config);
    bool isInPlantRegion(const cv::Point2f& point, const cv::Size& image_size);
    float calculatePlantConfidence(const cv::Mat& image, const cv::Point2f& center);
    
    // Coordinate transformation and publishing
    geometry_msgs::msg::Point pixel_to_world_coordinate(
        const cv::Point2f& pixel, const std::string& camera_frame);
    void publish_detected_plants(const std::vector<geometry_msgs::msg::Point>& plant_positions);
    void publish_debug_visualization(const cv::Mat& image, 
                                   const std::vector<Detection>& detections);
    
    // Configuration
    void loadDetectionConfig(const std::string& config_file);
    
    // ROS2 components
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr plants_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    
    // TF2 components
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // YOLO components
    cv::dnn::Net yolo_net_;
    std::vector<std::string> class_names_;
    
    // Configuration parameters
    std::string model_path_;
    std::string camera_frame_;
    std::string map_frame_;
    double confidence_threshold_;
    double nms_threshold_;
    PlantDetectionConfig detection_config_;
};

#endif  // ROVERROBOTICS_DRIVER__PLANT_DETECTION_NODE_HPP_