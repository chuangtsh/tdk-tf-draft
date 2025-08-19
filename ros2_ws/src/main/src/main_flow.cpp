#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MainFlowNode : public rclcpp::Node
{
public:
    MainFlowNode()
    : Node("main_flow_node")
    {
        RCLCPP_INFO(this->get_logger(), "Main Flow Node started");

        // Create subscriber for integer commands
        command_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/robot_command", 10,
            std::bind(&MainFlowNode::command_callback, this, _1));

        // Create service client for lifecycle management
        lifecycle_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
            "/robot_tf_broadcaster_lifecycle/change_state");

        RCLCPP_INFO(this->get_logger(), "Waiting for lifecycle service...");
        
        // Wait for the lifecycle service to be available
        while (!lifecycle_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        
        RCLCPP_INFO(this->get_logger(), "Lifecycle service is available");
        RCLCPP_INFO(this->get_logger(), "Main Flow Node initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Command mapping:");
        RCLCPP_INFO(this->get_logger(), "  1 - Configure");
        RCLCPP_INFO(this->get_logger(), "  2 - Activate");
        RCLCPP_INFO(this->get_logger(), "  3 - Deactivate");
        RCLCPP_INFO(this->get_logger(), "  4 - Cleanup");
        RCLCPP_INFO(this->get_logger(), "  5 - Shutdown");
    }

private:
    void command_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received command: %d", msg->data);

        uint8_t transition_id;
        std::string transition_name;

        // Map integer commands to lifecycle transitions
        // Using proper lifecycle_msgs::msg::Transition constants
        switch (msg->data) {
            case 1:
                transition_id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
                transition_name = "configure";
                break;
            case 2:
                transition_id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
                transition_name = "activate";
                break;
            case 3:
                transition_id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
                transition_name = "deactivate";
                break;
            case 4:
                transition_id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;
                transition_name = "cleanup";
                break;
            case 5:
                transition_id = lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
                transition_name = "shutdown";
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown command: %d. Valid commands are 1-5", msg->data);
                return;
        }

        // Call the lifecycle service
        call_lifecycle_service(transition_id, transition_name);
    }

    void call_lifecycle_service(uint8_t transition_id, const std::string& transition_name)
    {
        // Check if service is still available
        if (!lifecycle_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Lifecycle service is not ready, skipping request...");
            return;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition_id;

        RCLCPP_INFO(this->get_logger(), "Calling lifecycle service for transition: %s (ID: %d)", 
                    transition_name.c_str(), transition_id);

        // Store transition name for callback
        current_transition_name_ = transition_name;

        // Call the service asynchronously with callback - mimicking your homework style
        auto result = lifecycle_client_->async_send_request(
            request, 
            std::bind(&MainFlowNode::response_callback, this, _1)
        );
    }

    void response_callback(rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future)
    {
        try {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Successfully executed %s transition", 
                           current_transition_name_.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to execute %s transition", 
                            current_transition_name_.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed for %s transition: %s", 
                        current_transition_name_.c_str(), e.what());
        }
    }

    // Subscriptions and service clients
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr command_subscription_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr lifecycle_client_;
    
    // Store current transition name for callback
    std::string current_transition_name_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MainFlowNode>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}