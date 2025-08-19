#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

using std::placeholders::_1;
using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RobotTfBroadcasterLifecycle : public rclcpp_lifecycle::LifecycleNode
{
public:
    RobotTfBroadcasterLifecycle()
    : LifecycleNode("robot_tf_broadcaster_lifecycle")
    {
        RCLCPP_INFO(this->get_logger(), "Robot TF Broadcaster Lifecycle node created");
    }

    // Lifecycle callbacks
    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Configuring Robot TF Broadcaster Lifecycle node");

        // Declare parameters for static transforms
        // Camera static transform (arm_end -> camera)
        this->declare_parameter("camera_x", 0.1);
        this->declare_parameter("camera_y", 0.0);
        this->declare_parameter("camera_z", 0.05);
        this->declare_parameter("camera_roll", 0.0);
        this->declare_parameter("camera_pitch", 0.0);
        this->declare_parameter("camera_yaw", 0.0);
        
        // Map static transform (map -> odom) - robot starting position
        this->declare_parameter("odom_x", 0.0);
        this->declare_parameter("odom_y", 0.0);
        this->declare_parameter("odom_z", 0.0);
        this->declare_parameter("odom_roll", 0.0);
        this->declare_parameter("odom_pitch", 0.0);
        this->declare_parameter("odom_yaw", 0.0);

        // Initialize transform broadcasters
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        // Create subscriptions for dynamic transforms (but don't activate them yet)
        robot_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/odom", 10,
            std::bind(&RobotTfBroadcasterLifecycle::robot_pose_callback, this, _1));

        lift_base_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/lift_base", 10,
            std::bind(&RobotTfBroadcasterLifecycle::lift_base_callback, this, _1));

        arm_base_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/arm_base", 10,
            std::bind(&RobotTfBroadcasterLifecycle::arm_base_callback, this, _1));

        arm_end_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/arm_end", 10,
            std::bind(&RobotTfBroadcasterLifecycle::arm_end_callback, this, _1));

        // Create timer for publishing default transforms
        timer_ = this->create_wall_timer(
            100ms, std::bind(&RobotTfBroadcasterLifecycle::publish_static_transforms, this));

        RCLCPP_INFO(this->get_logger(), "Robot TF Broadcaster Lifecycle node configured successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Activating Robot TF Broadcaster Lifecycle node");

        // Reset received flags
        robot_pose_received_ = false;
        lift_base_received_ = false;
        arm_base_received_ = false;
        arm_end_received_ = false;

        // Publish static transforms
        this->publish_static_transforms();

        // Activate subscriptions if they support lifecycle (in this case, they're regular subscriptions)
        // The timer will start automatically when the node is activated

        RCLCPP_INFO(this->get_logger(), "Robot TF Broadcaster Lifecycle node activated successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Deactivating Robot TF Broadcaster Lifecycle node");

        // Stop processing but keep configuration
        // The timer and subscriptions will automatically stop processing

        RCLCPP_INFO(this->get_logger(), "Robot TF Broadcaster Lifecycle node deactivated successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up Robot TF Broadcaster Lifecycle node");

        // Reset all resources
        robot_pose_subscription_.reset();
        lift_base_subscription_.reset();
        arm_base_subscription_.reset();
        arm_end_subscription_.reset();
        timer_.reset();
        shutdown_timer_.reset();
        tf_broadcaster_.reset();
        static_tf_broadcaster_.reset();

        // Reset flags
        robot_pose_received_ = false;
        lift_base_received_ = false;
        arm_base_received_ = false;
        arm_end_received_ = false;

        RCLCPP_INFO(this->get_logger(), "Robot TF Broadcaster Lifecycle node cleaned up successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down Robot TF Broadcaster Lifecycle node");

        // Clean up all resources
        robot_pose_subscription_.reset();
        lift_base_subscription_.reset();
        arm_base_subscription_.reset();
        arm_end_subscription_.reset();
        timer_.reset();
        tf_broadcaster_.reset();
        static_tf_broadcaster_.reset();

        RCLCPP_INFO(this->get_logger(), "Robot TF Broadcaster Lifecycle node shut down successfully");
        
        // Create a timer to terminate the node after a short delay
        // This allows the shutdown response to be sent before termination
        shutdown_timer_ = this->create_wall_timer(
            100ms, std::bind(&RobotTfBroadcasterLifecycle::terminate_node, this));
        
        return CallbackReturn::SUCCESS;
    }

private:

    void robot_pose_callback(const std::shared_ptr<geometry_msgs::msg::Pose> msg)
    {
        // Only process if the node is active
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            return;
        }

        robot_pose_received_ = true;
        geometry_msgs::msg::TransformStamped t;

        // Set header
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        // Set translation
        t.transform.translation.x = msg->position.x;
        t.transform.translation.y = msg->position.y;
        t.transform.translation.z = msg->position.z;

        // Set rotation
        t.transform.rotation = msg->orientation;

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }

    void lift_base_callback(const std::shared_ptr<geometry_msgs::msg::Pose> msg)
    {
        // Only process if the node is active
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            return;
        }

        lift_base_received_ = true;
        geometry_msgs::msg::TransformStamped t;

        // Set header
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "base_link";
        t.child_frame_id = "lift_base";

        // Set translation
        t.transform.translation.x = msg->position.x;
        t.transform.translation.y = msg->position.y;
        t.transform.translation.z = msg->position.z;

        // Set rotation
        t.transform.rotation = msg->orientation;

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }

    void arm_base_callback(const std::shared_ptr<geometry_msgs::msg::Pose> msg)
    {
        // Only process if the node is active
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            return;
        }

        arm_base_received_ = true;
        geometry_msgs::msg::TransformStamped t;

        // Set header
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "lift_base";
        t.child_frame_id = "arm_base";

        // Set translation
        t.transform.translation.x = msg->position.x;
        t.transform.translation.y = msg->position.y;
        t.transform.translation.z = msg->position.z;

        // Set rotation
        t.transform.rotation = msg->orientation;

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }

    void arm_end_callback(const std::shared_ptr<geometry_msgs::msg::Pose> msg)
    {
        // Only process if the node is active
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            return;
        }

        arm_end_received_ = true;
        geometry_msgs::msg::TransformStamped t;

        // Set header
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "arm_base";
        t.child_frame_id = "arm_end";

        // Set translation
        t.transform.translation.x = msg->position.x;
        t.transform.translation.y = msg->position.y;
        t.transform.translation.z = msg->position.z;

        // Set rotation
        t.transform.rotation = msg->orientation;

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }

    void publish_static_transforms()
    {
        // Publish map -> odom static transform (robot starting position)
        geometry_msgs::msg::TransformStamped map_to_odom_transform;
        
        // Get odom parameters
        double odom_x = this->get_parameter("odom_x").as_double();
        double odom_y = this->get_parameter("odom_y").as_double();
        double odom_z = this->get_parameter("odom_z").as_double();
        double odom_roll = this->get_parameter("odom_roll").as_double();
        double odom_pitch = this->get_parameter("odom_pitch").as_double();
        double odom_yaw = this->get_parameter("odom_yaw").as_double();

        // Set header for map -> odom
        map_to_odom_transform.header.stamp = this->get_clock()->now();
        map_to_odom_transform.header.frame_id = "map";
        map_to_odom_transform.child_frame_id = "odom";

        // Set translation
        map_to_odom_transform.transform.translation.x = odom_x;
        map_to_odom_transform.transform.translation.y = odom_y;
        map_to_odom_transform.transform.translation.z = odom_z;

        // Convert RPY to quaternion
        tf2::Quaternion odom_q;
        odom_q.setRPY(odom_roll, odom_pitch, odom_yaw);
        map_to_odom_transform.transform.rotation.x = odom_q.x();
        map_to_odom_transform.transform.rotation.y = odom_q.y();
        map_to_odom_transform.transform.rotation.z = odom_q.z();
        map_to_odom_transform.transform.rotation.w = odom_q.w();

        // Send map -> odom static transform
        static_tf_broadcaster_->sendTransform(map_to_odom_transform);

        RCLCPP_INFO(this->get_logger(), 
            "Published static transform from map to odom: [%.3f, %.3f, %.3f]",
            odom_x, odom_y, odom_z);

        // Publish arm_end -> camera static transform
        geometry_msgs::msg::TransformStamped camera_transform;

        // Get camera parameters
        double camera_x = this->get_parameter("camera_x").as_double();
        double camera_y = this->get_parameter("camera_y").as_double();
        double camera_z = this->get_parameter("camera_z").as_double();
        double camera_roll = this->get_parameter("camera_roll").as_double();
        double camera_pitch = this->get_parameter("camera_pitch").as_double();
        double camera_yaw = this->get_parameter("camera_yaw").as_double();

        // Set header for arm_end -> camera
        camera_transform.header.stamp = this->get_clock()->now();
        camera_transform.header.frame_id = "arm_end";
        camera_transform.child_frame_id = "camera";

        // Set translation
        camera_transform.transform.translation.x = camera_x;
        camera_transform.transform.translation.y = camera_y;
        camera_transform.transform.translation.z = camera_z;

        // Convert RPY to quaternion
        tf2::Quaternion camera_q;
        camera_q.setRPY(camera_roll, camera_pitch, camera_yaw);
        camera_transform.transform.rotation.x = camera_q.x();
        camera_transform.transform.rotation.y = camera_q.y();
        camera_transform.transform.rotation.z = camera_q.z();
        camera_transform.transform.rotation.w = camera_q.w();

        // Send arm_end -> camera static transform
        static_tf_broadcaster_->sendTransform(camera_transform);

        RCLCPP_INFO(this->get_logger(), 
            "Published static transform from arm_end to camera: [%.3f, %.3f, %.3f]",
            camera_x, camera_y, camera_z);
    }

    void terminate_node()
    {
        RCLCPP_INFO(this->get_logger(), "Terminating node process...");
        rclcpp::shutdown();
        std::exit(0);
    }

    // Subscriptions and publishers
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr robot_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr lift_base_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr arm_base_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr arm_end_subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;
    
    // Flags to track if poses have been received
    bool robot_pose_received_ = false;
    bool lift_base_received_ = false;
    bool arm_base_received_ = false;
    bool arm_end_received_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RobotTfBroadcasterLifecycle>();
    rclcpp::spin(node->get_node_base_interface());
    
    rclcpp::shutdown();
    return 0;
}
