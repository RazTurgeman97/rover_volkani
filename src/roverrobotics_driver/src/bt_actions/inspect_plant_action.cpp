#include <behaviortree_cpp_v3/action_node.h>

class InspectPlantAction : public BT::SyncActionNode {
public:
    InspectPlantAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}
        
    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("plant_id") };
    }
    
    BT::NodeStatus tick() override {
        // Get plant ID from behavior tree
        auto plant_id = getInput<int>("plant_id");
        if (!plant_id) {
            return BT::NodeStatus::FAILURE;
        }
        
        // Your inspection logic here
        bool inspection_success = perform_plant_inspection(plant_id.value());
        
        return inspection_success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    
private:
    bool perform_plant_inspection(int plant_id) {
        // Take photos, collect sensor data, etc.
        RCLCPP_INFO(rclcpp::get_logger("bt"), "Inspecting plant %d", plant_id);
        
        // Simulate inspection time
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        return true; // Return false if inspection failed
    }
};