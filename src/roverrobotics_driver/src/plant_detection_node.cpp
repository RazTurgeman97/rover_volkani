#include "roverrobotics_driver/plant_detection_node.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

PlantDetectionNode::PlantDetectionNode() 
: Node("plant_detection_node"), 
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_) {
    
    // Declare parameters for testing
    declare_parameter<std::string>("model_path", "");
    declare_parameter<std::string>("camera_topic", "/camera/color/image_raw");
    declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<double>("confidence_threshold", 0.5);
    declare_parameter<double>("nms_threshold", 0.4);
    declare_parameter<bool>("enable_yolo", true);
    declare_parameter<bool>("enable_traditional_detection", false);
    declare_parameter<bool>("publish_debug_image", true);
    
    // Get parameters
    get_parameter("model_path", model_path_);
    get_parameter("camera_frame", camera_frame_);
    get_parameter("map_frame", map_frame_);
    get_parameter("confidence_threshold", confidence_threshold_);
    get_parameter("nms_threshold", nms_threshold_);
    
    bool enable_yolo = get_parameter("enable_yolo").as_bool();
    bool publish_debug = get_parameter("publish_debug_image").as_bool();
    
    // Initialize publishers
    plants_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/detected_plants", 10);
    
    if (publish_debug) {
        debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/camera/debug/image_raw", 
            rclcpp::SensorDataQoS()
        );
        RCLCPP_INFO(get_logger(), "Debug image publisher enabled");
    }
    
    // Initialize subscriber
    std::string camera_topic;
    get_parameter("camera_topic", camera_topic);
    camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
        camera_topic, 
        rclcpp::SensorDataQoS(),
        std::bind(&PlantDetectionNode::camera_callback, this, std::placeholders::_1)
    );
    
    // Load YOLO model if enabled and path provided
    if (enable_yolo && !model_path_.empty()) {
        try {
            RCLCPP_INFO(get_logger(), "Loading YOLO model from: %s", model_path_.c_str());
            yolo_net_ = cv::dnn::readNetFromONNX(model_path_);
            if (yolo_net_.empty()) {
                RCLCPP_ERROR(get_logger(), "Failed to load YOLO model");
            } else {
                RCLCPP_INFO(get_logger(), "YOLO model loaded successfully");
                yolo_net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                yolo_net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error loading YOLO model: %s", e.what());
        }
    } else {
        RCLCPP_INFO(get_logger(), "YOLO disabled, using traditional detection");
    }
    
    // Initialize class names
    class_names_ = {"plant"};
    
    RCLCPP_INFO(get_logger(), "Plant Detection Node initialized for testing");
    RCLCPP_INFO(get_logger(), "Camera topic: %s", camera_topic.c_str());
    RCLCPP_INFO(get_logger(), "YOLO enabled: %s", enable_yolo ? "YES" : "NO");
}

void PlantDetectionNode::camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat image = cv_ptr->image;
        
        // Run detection
        std::vector<Detection> detections;
        if (!yolo_net_.empty()) {
            detections = detect_plants_yolo(image);
        } else {
            // Fallback to traditional detection
            auto plant_points = detect_plants_traditional(image);
            // Convert points to detections for consistency
            for (const auto& point : plant_points) {
                Detection det;
                // Make clusters 3x longer (height) and 2x wider (width)
                int cluster_width = 100;   // 2x wider (was 50)
                int cluster_height = 150;  // 3x longer (was 50)
                
                det.bounding_box = cv::Rect(
                    point.x - cluster_width/2, 
                    point.y - cluster_height/2, 
                    cluster_width, 
                    cluster_height
                );
                
                det.confidence = calculatePlantConfidence(image, point);
                det.class_name = "plant";
                det.health_status = "unknown";
                detections.push_back(det);
            }
        }
        
        // Convert detections to world coordinates and publish
        std::vector<geometry_msgs::msg::Point> world_positions;
        for (const auto& detection : detections) {
            cv::Point2f center(detection.bounding_box.x + detection.bounding_box.width/2.0f,
                             detection.bounding_box.y + detection.bounding_box.height/2.0f);
            auto world_pos = pixel_to_world_coordinate(center, camera_frame_);
            world_positions.push_back(world_pos);
        }
        
        publish_detected_plants(world_positions);
        publish_debug_visualization(image, detections);
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "CV bridge exception: %s", e.what());
    }
}

std::vector<Detection> PlantDetectionNode::detect_plants_yolo(const cv::Mat& image) {
    std::vector<Detection> detections;
    
    if (yolo_net_.empty()) {
        return detections;
    }
    
    try {
        // Prepare input blob
        cv::Mat blob;
        cv::dnn::blobFromImage(image, blob, 1/255.0, cv::Size(640, 640), cv::Scalar(0,0,0), true, false);
        yolo_net_.setInput(blob);
        
        // Run inference
        std::vector<cv::Mat> outputs;
        yolo_net_.forward(outputs, yolo_net_.getUnconnectedOutLayersNames());
        
        // Process outputs for YOLOv8/v11 format
        if (!outputs.empty()) {
            cv::Mat output = outputs[0];
            
            // YOLOv8 output shape: [1, 84, 8400] or [1, num_classes+4, num_detections]
            if (output.dims == 3) {
                output = output.reshape(1, output.size[1]); // Reshape to 2D
            }
            
            std::vector<int> class_ids;
            std::vector<float> confidences;
            std::vector<cv::Rect> boxes;
            
            // Transpose if needed (YOLOv8 format)
            if (output.cols > output.rows) {
                cv::transpose(output, output);
            }
            
            for (int i = 0; i < output.rows; ++i) {
                cv::Mat row = output.row(i);
                
                // Extract bbox coordinates (center_x, center_y, width, height)
                float center_x = row.at<float>(0);
                float center_y = row.at<float>(1);
                float width = row.at<float>(2);
                float height = row.at<float>(3);
                
                // Extract class scores
                cv::Mat scores = row.colRange(4, row.cols);
                cv::Point class_id_point;
                double max_class_score;
                cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id_point);
                
                if (max_class_score > confidence_threshold_) {
                    // Convert center coordinates to top-left coordinates
                    float x = (center_x - width / 2) * image.cols;
                    float y = (center_y - height / 2) * image.rows;
                    float w = width * image.cols;
                    float h = height * image.rows;
                    
                    boxes.push_back(cv::Rect(static_cast<int>(x), static_cast<int>(y), 
                                           static_cast<int>(w), static_cast<int>(h)));
                    confidences.push_back(static_cast<float>(max_class_score));
                    class_ids.push_back(class_id_point.x);
                }
            }
            
            // Apply Non-Maximum Suppression
            std::vector<int> nms_result;
            cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold_, nms_result);
            
            for (int idx : nms_result) {
                Detection detection;
                detection.bounding_box = boxes[idx];
                detection.confidence = confidences[idx];
                detection.class_id = class_ids[idx];
                detection.class_name = (class_ids[idx] < class_names_.size()) ? 
                                     class_names_[class_ids[idx]] : "unknown";
                detections.push_back(detection);
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "YOLO detection error: %s", e.what());
    }
    
    return detections;
}

std::vector<cv::Point2f> PlantDetectionNode::detect_plants_traditional(const cv::Mat& image) {
    std::vector<cv::Point2f> plant_points;
    
    // Convert to HSV for better plant detection
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    
    // Define green range for plants (not just any green)
    cv::Scalar lower_green(35, 40, 40);    // More restrictive
    cv::Scalar upper_green(85, 255, 255);
    
    // Create mask for plant-like green
    cv::Mat mask;
    cv::inRange(hsv, lower_green, upper_green, mask);
    
    // Morphological operations to clean up noise
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        
        // Filter by size - plants should be reasonably sized
        if (area > 1000 && area < 15000) {  // Adjust based on your plants
            // Check if it's roughly circular (plants from above)
            double perimeter = cv::arcLength(contour, true);
            double circularity = 4 * CV_PI * area / (perimeter * perimeter);
            
            if (circularity > 0.3) {  // Reasonably circular
                cv::Moments m = cv::moments(contour);
                if (m.m00 != 0) {
                    cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);
                    
                    // Check if it's in a plant area (not edges/walls)
                    if (isInPlantRegion(center, image.size())) {
                        plant_points.push_back(center);
                    }
                }
            }
        }
    }
    
    return plant_points;
}

bool PlantDetectionNode::isInPlantRegion(const cv::Point2f& point, const cv::Size& image_size) {
    // Avoid edges where non-plant objects might be
    int border = 50;
    return (point.x > border && point.x < image_size.width - border &&
            point.y > border && point.y < image_size.height - border);
}

geometry_msgs::msg::Point PlantDetectionNode::pixel_to_world_coordinate(
    const cv::Point2f& pixel, const std::string& camera_frame) {
    
    geometry_msgs::msg::Point world_point;
    
    try {
        // For now, use a simple projection assuming the camera is looking down
        // In a real system, you'd use camera calibration and depth information
        
        // Simple conversion (you should replace this with proper camera calibration)
        double pixel_size = 0.01; // meters per pixel (approximate)
        world_point.x = pixel.x * pixel_size;
        world_point.y = pixel.y * pixel_size;
        world_point.z = 0.0;
        
        // TODO: Transform from camera frame to map frame using TF2
        // geometry_msgs::msg::TransformStamped transform = 
        //     tf_buffer_.lookupTransform(map_frame_, camera_frame, tf2::TimePointZero);
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Failed to convert pixel to world coordinates: %s", e.what());
        world_point.x = 0.0;
        world_point.y = 0.0;
        world_point.z = 0.0;
    }
    
    return world_point;
}

void PlantDetectionNode::publish_detected_plants(const std::vector<geometry_msgs::msg::Point>& plant_positions) {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = now();
    pose_array.header.frame_id = map_frame_;
    
    for (const auto& position : plant_positions) {
        geometry_msgs::msg::Pose pose;
        pose.position = position;
        pose.orientation.w = 1.0; // Default orientation
        pose_array.poses.push_back(pose);
    }
    
    plants_pub_->publish(pose_array);
}

void PlantDetectionNode::publish_debug_visualization(const cv::Mat& image, 
                                                   const std::vector<Detection>& detections) {
    cv::Mat debug_image = image.clone();
    
    for (const auto& detection : detections) {
        cv::rectangle(debug_image, detection.bounding_box, cv::Scalar(0, 255, 0), 2);
        
        std::string label = detection.class_name + " " + 
                           std::to_string(static_cast<int>(detection.confidence * 100)) + "%";
        
        cv::putText(debug_image, label, 
                   cv::Point(detection.bounding_box.x, detection.bounding_box.y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }
    
    // Convert back to ROS message
    sensor_msgs::msg::Image::SharedPtr debug_msg = 
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_image).toImageMsg();
    debug_msg->header.stamp = now();
    debug_msg->header.frame_id = camera_frame_;
    
    debug_image_pub_->publish(*debug_msg);
}

std::vector<PlantDetection> PlantDetectionNode::filterPlantDetections(
    const std::vector<cv::Rect>& raw_detections,
    const cv::Mat& image,
    const PlantDetectionConfig& config)
{
    std::vector<PlantDetection> filtered_plants;
    
    for (const auto& detection : raw_detections) {
        // Extract ROI from image
        cv::Mat plant_roi = image(detection);
        
        // 1. Size filtering
        double detection_area = detection.width * detection.height;
        if (detection_area < config.min_detection_area || 
            detection_area > config.max_detection_area) {
            continue;
        }
        
        // 2. Color filtering for green plants
        if (config.color_filter_enabled) {
            if (!isGreenPlant(plant_roi, config)) {
                continue;
            }
        }
        
        // 3. Shape filtering
        if (config.shape_filter_enabled) {
            if (!hasPlantShape(plant_roi, config)) {
                continue;
            }
        }
        
        // 4. ROI filtering - only detect in plant areas
        if (config.roi_filter_enabled) {
            if (!isInPlantArea(detection, config)) {
                continue;
            }
        }
        
        // If it passes all filters, add to results
        PlantDetection plant;
        plant.bounding_box = detection;
        plant.confidence = 0.8f; // Default confidence since calculateConfidence is not implemented
        plant.class_name = "plant";
        plant.health_status = "unknown";
        filtered_plants.push_back(plant);
    }
    
    return filtered_plants;
}

bool PlantDetectionNode::isGreenPlant(const cv::Mat& roi, const PlantDetectionConfig& config)
{
    // Convert to HSV for better color filtering
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    
    // Define green color range
    cv::Scalar lower_green(config.hue_range[0], config.saturation_min, config.value_min);
    cv::Scalar upper_green(config.hue_range[1], 255, 255);
    
    // Create mask for green pixels
    cv::Mat mask;
    cv::inRange(hsv, lower_green, upper_green, mask);
    
    // Calculate percentage of green pixels
    double green_ratio = cv::countNonZero(mask) / (double)(roi.rows * roi.cols);
    
    // Plant should have significant green content
    return green_ratio > 0.3;  // At least 30% green
}

bool PlantDetectionNode::hasPlantShape(const cv::Mat& roi, const PlantDetectionConfig& config)
{
    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    if (contours.empty()) return false;
    
    // Get largest contour (should be the plant)
    auto largest_contour = *std::max_element(contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        });
    
    // Calculate circularity
    double area = cv::contourArea(largest_contour);
    double perimeter = cv::arcLength(largest_contour, true);
    double circularity = 4 * CV_PI * area / (perimeter * perimeter);
    
    // Plants should be reasonably circular from above
    return circularity > config.min_circularity;
}

bool PlantDetectionNode::isInPlantArea(const cv::Rect& detection, const PlantDetectionConfig& config)
{
    cv::Point2f center(detection.x + detection.width/2.0f, detection.y + detection.height/2.0f);
    geometry_msgs::msg::Point world_point = pixel_to_world_coordinate(center, camera_frame_);
    
    // Check if point is in any of the defined plant rows
    for (const auto& region : config.roi_regions) {
        if (world_point.x >= region.x_min && world_point.x <= region.x_max &&
            world_point.y >= region.y_min && world_point.y <= region.y_max) {
            return true;
        }
    }
    
    return false;
}

void PlantDetectionNode::loadDetectionConfig(const std::string& config_file) {
    // For now, use default config
    // TODO: Load from YAML file using yaml-cpp
    detection_config_.min_detection_area = 500.0;
    detection_config_.max_detection_area = 10000.0;
    detection_config_.color_filter_enabled = true;
    detection_config_.shape_filter_enabled = true;
    detection_config_.roi_filter_enabled = true;
    
    // Add ROI regions for your plant rows
    ROIRegion row1 = {0.5, 5.5, 0.5, 1.5};
    ROIRegion row2 = {0.5, 5.5, 1.8, 2.8};
    ROIRegion row3 = {0.5, 5.5, 3.5, 4.5};
    
    detection_config_.roi_regions = {row1, row2, row3};
}

float PlantDetectionNode::calculatePlantConfidence(const cv::Mat& image, const cv::Point2f& center) {
    // Extract ROI around detection
    cv::Rect roi(center.x - 25, center.y - 25, 50, 50);
    roi = roi & cv::Rect(0, 0, image.cols, image.rows);  // Ensure within bounds
    
    if (roi.area() == 0) return 0.0f;
    
    cv::Mat plant_roi = image(roi);
    cv::Mat hsv;
    cv::cvtColor(plant_roi, hsv, cv::COLOR_BGR2HSV);
    
    // Calculate green percentage
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(35, 40, 40), cv::Scalar(85, 255, 255), mask);
    float green_ratio = cv::countNonZero(mask) / (float)mask.total();
    
    // Calculate texture variation (plants have varied texture)
    cv::Mat gray;
    cv::cvtColor(plant_roi, gray, cv::COLOR_BGR2GRAY);
    cv::Scalar mean, stddev;
    cv::meanStdDev(gray, mean, stddev);
    float texture_score = std::min(stddev[0] / 50.0, 1.0);  // Normalize
    
    // Combine scores
    float confidence = (green_ratio * 0.7f + texture_score * 0.3f);
    return std::max(0.1f, std::min(0.95f, confidence));  // Clamp between 0.1-0.95
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlantDetectionNode>());
    rclcpp::shutdown();
    return 0;
}