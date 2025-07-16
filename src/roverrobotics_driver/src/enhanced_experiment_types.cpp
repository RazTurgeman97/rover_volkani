// Add these to your greenhouse_experiment_node.cpp

void GreenhouseExperimentNode::run_experiment() {
    switch (experiment_type_) {
        case 1: // Original: Basic sequential inspection
            run_basic_sequential_inspection();
            break;
            
        case 2: // Original: Random sampling
            run_random_sampling();
            break;
            
        case 6: // NEW: AI-driven adaptive inspection
            run_ai_adaptive_inspection();
            break;
            
        case 7: // NEW: Coverage-based systematic inspection
            run_coverage_based_inspection();
            break;
            
        case 8: // NEW: Anomaly detection and response
            run_anomaly_detection_mode();
            break;
            
        case 9: // NEW: Multi-robot coordination (future)
            run_multi_robot_coordination();
            break;
            
        case 10: // NEW: Temporal monitoring (longitudinal study)
            run_temporal_monitoring();
            break;
    }
}

void GreenhouseExperimentNode::run_ai_adaptive_inspection() {
    /*
     * AI-Driven Adaptive Inspection Mode
     * 
     * This mode uses computer vision and machine learning to:
     * 1. Detect plants in real-time using camera
     * 2. Classify plant health status automatically
     * 3. Prioritize diseased plants for immediate inspection
     * 4. Adapt navigation based on findings
     * 5. Learn from inspection results to improve detection
     * 
     * Benefits:
     * - No pre-programmed plant locations needed
     * - Responds to real-world conditions
     * - Prioritizes urgent cases (diseased plants)
     * - Continuously improves through learning
     */
    
    RCLCPP_INFO(get_logger(), "Starting AI-driven adaptive inspection...");
    
    // Subscribe to AI perception results
    auto ai_results_sub = create_subscription<std_msgs::msg::Int32MultiArray>(
        "/infected_plant_notification", 10,
        [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            // When AI detects diseased plants, immediately prioritize them
            for (int diseased_plant_id : msg->data) {
                RCLCPP_WARN(get_logger(), "URGENT: Disease detected at plant %d", diseased_plant_id);
                
                // Add to priority queue for immediate inspection
                priority_inspection_queue_.push(diseased_plant_id);
                
                // Could also send alert to human operators
                send_disease_alert(diseased_plant_id);
            }
        });
    
    // Main inspection loop
    while (rclcpp::ok() && experiment_active_) {
        // Check for priority inspections first
        if (!priority_inspection_queue_.empty()) {
            int urgent_plant = priority_inspection_queue_.front();
            priority_inspection_queue_.pop();
            
            RCLCPP_INFO(get_logger(), "Performing urgent inspection of plant %d", urgent_plant);
            perform_detailed_inspection(urgent_plant);
        }
        
        // Continue with regular AI-guided exploration
        auto newly_detected_plants = wait_for_ai_detections(std::chrono::seconds(5));
        
        for (const auto& plant_info : newly_detected_plants) {
            // Navigate to detected plant using adaptive navigation
            navigate_to_ai_detected_plant(plant_info);
            
            // Perform inspection and update AI model with results
            InspectionResult result = perform_automated_inspection(plant_info);
            update_ai_model_with_feedback(plant_info, result);
        }
        
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
}

void GreenhouseExperimentNode::run_coverage_based_inspection() {
    /*
     * Coverage-Based Systematic Inspection
     * 
     * This mode ensures complete coverage of the greenhouse by:
     * 1. Dividing greenhouse into grid cells
     * 2. Planning path to visit every cell (like a lawnmower pattern)
     * 3. Inspecting any plants found in each cell
     * 4. Maintaining map of covered vs uncovered areas
     * 5. Adapting path if obstacles are encountered
     * 
     * Benefits:
     * - Guarantees complete coverage
     * - No plants missed
     * - Systematic and repeatable
     * - Good for research requiring complete datasets
     */
    
    RCLCPP_INFO(get_logger(), "Starting coverage-based systematic inspection...");
    
    // Define greenhouse boundaries (you'd get these from your map)
    double min_x = -2.0, max_x = 8.0;  // Adjust based on your greenhouse
    double min_y = -1.0, max_y = 5.0;
    double cell_size = 0.5; // 50cm grid cells
    
    // Generate coverage path (lawnmower pattern)
    std::vector<geometry_msgs::msg::PoseStamped> coverage_path;
    
    bool left_to_right = true;
    for (double y = min_y; y <= max_y; y += cell_size) {
        if (left_to_right) {
            // Go from left to right
            for (double x = min_x; x <= max_x; x += cell_size) {
                coverage_path.push_back(create_pose_stamped(x, y, 0.0));
            }
        } else {
            // Go from right to left
            for (double x = max_x; x >= min_x; x -= cell_size) {
                coverage_path.push_back(create_pose_stamped(x, y, M_PI)); // Face opposite direction
            }
        }
        left_to_right = !left_to_right; // Alternate direction
    }
    
    RCLCPP_INFO(get_logger(), "Generated coverage path with %zu waypoints", coverage_path.size());
    
    // Execute coverage path
    for (size_t i = 0; i < coverage_path.size() && rclcpp::ok(); ++i) {
        RCLCPP_INFO(get_logger(), "Covering cell %zu/%zu", i+1, coverage_path.size());
        
        // Navigate to cell
        bool nav_success = navigate_to_pose_with_retry(coverage_path[i]);
        
        if (nav_success) {
            // Scan for plants in this cell
            std::vector<PlantInfo> plants_in_cell = scan_current_area_for_plants();
            
            // Inspect any plants found
            for (const auto& plant : plants_in_cell) {
                RCLCPP_INFO(get_logger(), "Found plant in cell, inspecting...");
                perform_detailed_inspection(plant.id);
                
                // Record coverage data
                CoverageData coverage_entry;
                coverage_entry.cell_x = coverage_path[i].pose.position.x;
                coverage_entry.cell_y = coverage_path[i].pose.position.y;
                coverage_entry.plants_found = plants_in_cell.size();
                coverage_entry.timestamp = this->now();
                coverage_results_.push_back(coverage_entry);
            }
        } else {
            RCLCPP_WARN(get_logger(), "Failed to reach cell %zu, marking as inaccessible", i);
            // Mark cell as inaccessible and continue
        }
        
        // Brief pause before next cell
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    
    RCLCPP_INFO(get_logger(), "Coverage inspection complete. Covered %zu cells", coverage_path.size());
}

void GreenhouseExperimentNode::run_anomaly_detection_mode() {
    /*
     * Anomaly Detection and Response Mode
     * 
     * This mode focuses on detecting unusual conditions:
     * 1. Monitor environmental sensors continuously
     * 2. Use statistical analysis to detect anomalies
     * 3. Investigate anomalous areas immediately
     * 4. Alert human operators to critical issues
     * 5. Build baseline of "normal" conditions
     * 
     * Benefits:
     * - Early detection of problems (pests, disease, equipment failure)
     * - Proactive rather than reactive approach
     * - Learns what's "normal" for your specific greenhouse
     * - Can detect subtle changes humans might miss
     */
    
    RCLCPP_INFO(get_logger(), "Starting anomaly detection mode...");
    
    // Initialize baseline environmental data
    EnvironmentalBaseline baseline = establish_environmental_baseline();
    
    // Set up anomaly detection thresholds
    double temperature_threshold = 2.0; // 2°C deviation
    double humidity_threshold = 10.0;   // 10% deviation
    double light_threshold = 15.0;      // 15% deviation
    
    while (rclcpp::ok() && experiment_active_) {
        // Collect current environmental data
        EnvironmentalReading current = get_current_environmental_data();
        
        // Check for anomalies
        std::vector<AnomalyAlert> anomalies;
        
        if (std::abs(current.temperature - baseline.avg_temperature) > temperature_threshold) {
            AnomalyAlert alert;
            alert.type = "TEMPERATURE_ANOMALY";
            alert.severity = "HIGH";
            alert.location = get_current_position();
            alert.description = "Temperature deviation: " + std::to_string(current.temperature) + 
                              "°C (baseline: " + std::to_string(baseline.avg_temperature) + "°C)";
            anomalies.push_back(alert);
        }
        
        if (std::abs(current.humidity - baseline.avg_humidity) > humidity_threshold) {
            AnomalyAlert alert;
            alert.type = "HUMIDITY_ANOMALY";
            alert.severity = "MEDIUM";
            alert.location = get_current_position();
            alert.description = "Humidity deviation: " + std::to_string(current.humidity) + 
                              "% (baseline: " + std::to_string(baseline.avg_humidity) + "%)";
            anomalies.push_back(alert);
        }
        
        // If anomalies detected, investigate immediately
        for (const auto& anomaly : anomalies) {
            RCLCPP_WARN(get_logger(), "ANOMALY DETECTED: %s - %s", 
                       anomaly.type.c_str(), anomaly.description.c_str());
            
            // Navigate to anomaly location for detailed investigation
            investigate_anomaly(anomaly);
            
            // Send alert to human operators
            send_anomaly_alert(anomaly);
            
            // Update baseline with new data (adaptive learning)
            update_baseline_with_new_data(current);
        }
        
        // Continue patrol pattern while monitoring
        patrol_greenhouse_for_anomalies();
        
        rclcpp::sleep_for(std::chrono::seconds(10)); // Check every 10 seconds
    }
}

void GreenhouseExperimentNode::run_multi_robot_coordination() {
    /*
     * Multi-Robot Coordination Mode (Future Implementation)
     * 
     * This mode coordinates multiple robots working together:
     * 1. Divide greenhouse into zones for each robot
     * 2. Share detection results between robots
     * 3. Coordinate to avoid collisions
     * 4. Dynamic task redistribution based on findings
     * 5. Collective intelligence and learning
     * 
     * Benefits:
     * - Faster coverage of large greenhouses
     * - Redundancy for critical tasks
     * - Specialization (one robot for imaging, one for sampling)
     * - Fault tolerance (if one robot fails, others continue)
     */
    
    RCLCPP_INFO(get_logger(), "Starting multi-robot coordination mode...");
    
    // This is a future implementation - placeholder for now
    RCLCPP_WARN(get_logger(), "Multi-robot coordination not yet implemented");
    RCLCPP_INFO(get_logger(), "This would coordinate with other robots in the greenhouse");
    
    // Example of what this would do:
    /*
    // Get list of active robots in greenhouse
    auto active_robots = discover_other_robots();
    
    // Negotiate zone assignments
    GreenhouseZone my_zone = negotiate_zone_assignment(active_robots);
    
    // Share my sensor data with other robots
    auto data_sharing_pub = create_publisher<custom_msgs::msg::SensorData>("/shared_sensor_data", 10);
    
    // Subscribe to other robots' findings
    auto shared_findings_sub = create_subscription<custom_msgs::msg::PlantFindings>(
        "/shared_findings", 10,
        [this](const custom_msgs::msg::PlantFindings::SharedPtr msg) {
            // Incorporate findings from other robots
            update_global_plant_map(msg);
        });
    
    // Execute coordinated inspection of my assigned zone
    inspect_assigned_zone(my_zone);
    */
}

void GreenhouseExperimentNode::run_temporal_monitoring() {
    /*
     * Temporal Monitoring Mode (Longitudinal Study)
     * 
     * This mode tracks changes over time:
     * 1. Repeatedly visit same plants at scheduled intervals
     * 2. Compare current state with historical data
     * 3. Track growth rates, health changes, etc.
     * 4. Build time-series datasets for research
     * 5. Predict future plant health based on trends
     * 
     * Benefits:
     * - Understand plant development over time
     * - Early detection of slow-developing issues
     * - Research into growth patterns and environmental effects
     * - Predictive analytics for crop management
     */
    
    RCLCPP_INFO(get_logger(), "Starting temporal monitoring mode...");
    
    // Load previous monitoring data if it exists
    load_historical_monitoring_data();
    
    // Define monitoring schedule (e.g., daily, weekly)
    std::chrono::hours monitoring_interval(24); // Monitor every 24 hours
    auto last_monitoring_time = std::chrono::steady_clock::now();
    
    while (rclcpp::ok() && experiment_active_) {
        auto current_time = std::chrono::steady_clock::now();
        
        // Check if it's time for next monitoring cycle
        if (current_time - last_monitoring_time >= monitoring_interval) {
            RCLCPP_INFO(get_logger(), "Starting scheduled monitoring cycle...");
            
            // Visit each plant and record temporal data
            for (const auto& [plant_id, plant_data] : plant_data_map_) {
                // Navigate to plant
                bool nav_success = navigate_to_plant(plant_id);
                
                if (nav_success) {
                    // Collect comprehensive data
                    TemporalDataPoint data_point;
                    data_point.plant_id = plant_id;
                    data_point.timestamp = this->now();
                    data_point.position = get_current_position();
                    
                    // Collect multi-modal data
                    data_point.rgb_image = capture_rgb_image();
                    data_point.depth_data = capture_depth_data();
                    data_point.environmental_data = get_local_environmental_data();
                    
                    // AI analysis for health metrics
                    data_point.health_score = analyze_plant_health(data_point.rgb_image);
                    data_point.growth_metrics = calculate_growth_metrics(data_point);
                    
                    // Compare with historical data
                    auto historical_trend = analyze_temporal_trend(plant_id, data_point);
                    
                    if (historical_trend.concerning_changes_detected) {
                        RCLCPP_WARN(get_logger(), "Concerning trend detected for plant %d: %s", 
                                   plant_id, historical_trend.description.c_str());
                        
                        // Flag for immediate attention
                        flag_plant_for_immediate_attention(plant_id, historical_trend);
                    }
                    
                    // Store data point
                    temporal_monitoring_data_[plant_id].push_back(data_point);
                    
                    RCLCPP_INFO(get_logger(), "Completed temporal monitoring of plant %d", plant_id);
                } else {
                    RCLCPP_WARN(get_logger(), "Failed to reach plant %d for temporal monitoring", plant_id);
                }
            }
            
            // Save all temporal data
            save_temporal_monitoring_data();
            
            // Generate temporal analysis report
            generate_temporal_analysis_report();
            
            last_monitoring_time = current_time;
            RCLCPP_INFO(get_logger(), "Monitoring cycle complete. Next cycle in %ld hours", 
                       std::chrono::duration_cast<std::chrono::hours>(monitoring_interval).count());
        }
        
        // Sleep until next check
        rclcpp::sleep_for(std::chrono::hours(1)); // Check every hour
    }
}

// Helper functions for enhanced experiment types

void GreenhouseExperimentNode::investigate_anomaly(const AnomalyAlert& anomaly) {
    /*
     * Detailed investigation of detected anomaly
     * Navigate to anomaly location and perform comprehensive analysis
     */
    
    RCLCPP_INFO(get_logger(), "Investigating anomaly at location (%.2f, %.2f)", 
               anomaly.location.x, anomaly.location.y);
    
    // Navigate to anomaly location
    geometry_msgs::msg::PoseStamped investigation_pose;
    investigation_pose.header.frame_id = "map";
    investigation_pose.pose.position = anomaly.location;
    investigation_pose.pose.orientation.w = 1.0;
    
    bool nav_success = navigate_to_pose_with_retry(investigation_pose);
    
    if (nav_success) {
        // Perform detailed sensor readings
        auto detailed_env_data = get_detailed_environmental_data();
        auto visual_inspection = capture_anomaly_investigation_images();
        
        // Look for nearby plants that might be affected
        auto nearby_plants = find_plants_near_position(anomaly.location, 1.0); // 1m radius
        
        for (const auto& plant : nearby_plants) {
            RCLCPP_INFO(get_logger(), "Checking plant %d near anomaly", plant.id);
            perform_detailed_inspection(plant.id);
        }
        
        // Record investigation results
        AnomalyInvestigationResult result;
        result.anomaly = anomaly;
        result.investigation_time = this->now();
        result.environmental_data = detailed_env_data;
        result.affected_plants = nearby_plants;
        result.follow_up_required = (anomaly.severity == "HIGH");
        
        save_anomaly_investigation_result(result);
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to reach anomaly location for investigation");
    }
}

std::vector<PlantInfo> GreenhouseExperimentNode::scan_current_area_for_plants() {
    /*
     * Use computer vision to detect plants in current field of view
     * This would integrate with your YOLO or other CV model
     */
    
    std::vector<PlantInfo> detected_plants;
    
    // This is where you'd integrate your computer vision model
    // For now, this is a placeholder
    RCLCPP_INFO(get_logger(), "Scanning current area for plants using computer vision...");
    
    // Capture current camera image
    auto current_image = capture_rgb_image();
    
    // Run AI detection (placeholder - you'd implement actual YOLO here)
    auto detections = run_plant_detection_ai(current_image);
    
    // Convert AI detections to plant info
    for (const auto& detection : detections) {
        PlantInfo plant_info;
        plant_info.detected_position = detection.world_position;
        plant_info.confidence = detection.confidence;
        plant_info.plant_type = detection.classification;
        plant_info.health_status = detection.health_assessment;
        
        detected_plants.push_back(plant_info);
    }
    
    RCLCPP_INFO(get_logger(), "Detected %zu plants in current area", detected_plants.size());
    return detected_plants;
}