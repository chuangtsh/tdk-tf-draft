#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

constexpr int NUM_MISSIONS = 5;


class MainFlowNode : public rclcpp::Node {
public:
    MainFlowNode() : Node("main_flow_node")
    {
        RCLCPP_INFO(this->get_logger(), "Main Flow Node started");

        // Create subscriber for mission status
        command_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/mission_status", 10,
            std::bind(&MainFlowNode::command_callback, this, _1));

        // Initialize mission names array
        for (int i = 0; i < NUM_MISSIONS; ++i) {
            mission_names_[i] = "mission" + std::to_string(i + 1) + "_main";
        }
        
        // Create service clients for each mission's lifecycle management
        for (int i = 0; i < NUM_MISSIONS; ++i) {
            std::string service_name = "/" + mission_names_[i] + "/change_state";
            mission_clients_[i] = this->create_client<lifecycle_msgs::srv::ChangeState>(service_name);
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for mission lifecycle services...");
        
        // Wait for all mission lifecycle services to be available
        for (int i = 0; i < NUM_MISSIONS; ++i) {
            RCLCPP_INFO(this->get_logger(), "Waiting for %s lifecycle service...", mission_names_[i].c_str());
            while (!mission_clients_[i]->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "%s service not available, waiting again...", mission_names_[i].c_str());
            }
            RCLCPP_INFO(this->get_logger(), "%s lifecycle service is available", mission_names_[i].c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "All mission lifecycle services are available");
        RCLCPP_INFO(this->get_logger(), "Main Flow Node initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Mission status mapping:");
        for (int i = 0; i < NUM_MISSIONS; ++i) {
            RCLCPP_INFO(this->get_logger(), "  %ld - %s", i + 1, mission_names_[i].c_str());
        }
    }

private:

    void command_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Mission at: %d", msg->data);

        int mission_id = msg->data / 10;
        int piece_id = msg->data % 10;

        
        if ( piece_id == 9 ) {
            switch ( mission_id ) {
            case 0:
                // configure mission 1
                configure_mission(1);
                break;
            case 1:
                // configure mission 2
                configure_mission(2);
                break;
            case 2:
                // configure mission 3
                configure_mission(3);
                break;
            case 3:
                // configure mission 4
                configure_mission(4);
                break;
            case 4:
                // configure mission 5
                configure_mission(5);
                break;
            default:
                break;
            }
        }
        else if ( piece_id == 0 ) {
            switch ( mission_id ) {
            case 2:
                // clean up mission 1
                cleanup_mission(1); // mission 1 is at index 0
                break;
            case 3:
                // clean up mission 2
                cleanup_mission(2);
                break;
            case 4:
                // clean up mission 3
                cleanup_mission(3);
                break;
            case 5:
                // clean up mission 4
                cleanup_mission(4);
                break;
            default:
                break;
            }
        }


    }

    void configure_mission(int mission_index) {
        mission_index--; // Convert from 1-based to 0-based indexing
        
        if (mission_index < 0 || mission_index >= NUM_MISSIONS) {
            RCLCPP_ERROR(this->get_logger(), "Invalid mission index: %d", mission_index + 1);
            return;
        }

        const std::string& mission_name = mission_names_[mission_index];
        auto& client = mission_clients_[mission_index];

        // Check if service is ready
        if (!client->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "%s lifecycle service is not ready, skipping request...", 
                       mission_name.c_str());
            return;
        }

        // Create configure request
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;

        RCLCPP_INFO(this->get_logger(), "Configuring %s...", mission_name.c_str());

        // Store current mission info for callback
        current_mission_index_ = mission_index;

        // Call the service asynchronously
        auto result = client->async_send_request(
            request, 
            std::bind(&MainFlowNode::configure_response_callback, this, _1)
        );
    }

    void configure_response_callback(rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future)
    {
        const std::string& mission_name = mission_names_[current_mission_index_];

        try {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Successfully configured %s", mission_name.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to configure %s", mission_name.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed for %s configure: %s", 
                        mission_name.c_str(), e.what());
        }
    }

    void cleanup_mission(int mission_index)
    {
        mission_index--; // Convert from 1-based to 0-based indexing
        
        if (mission_index < 0 || mission_index >= NUM_MISSIONS) {
            RCLCPP_ERROR(this->get_logger(), "Invalid mission index: %d", mission_index + 1);
            return;
        }

        const std::string& mission_name = mission_names_[mission_index];
        auto& client = mission_clients_[mission_index];

        // Check if service is ready
        if (!client->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "%s lifecycle service is not ready, skipping request...", 
                       mission_name.c_str());
            return;
        }

        // Create cleanup request
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;

        RCLCPP_INFO(this->get_logger(), "Cleaning up %s...", mission_name.c_str());

        // Store current mission info for callback
        current_mission_index_ = mission_index;

        // Call the service asynchronously
        auto result = client->async_send_request(
            request,
            std::bind(&MainFlowNode::cleanup_response_callback, this, _1)
        );
    }

    void cleanup_response_callback(rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future)
    {
        const std::string& mission_name = mission_names_[current_mission_index_];

        try {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Successfully cleaned up %s", mission_name.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to clean up %s", mission_name.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed for %s cleanup: %s", 
                        mission_name.c_str(), e.what());
        }
    }



    // Subscriptions and service clients
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr command_subscription_;
    std::array<rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr, NUM_MISSIONS> mission_clients_;
    std::array<std::string, NUM_MISSIONS> mission_names_;
    
    // Store current mission index for callback
    int current_mission_index_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MainFlowNode>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
