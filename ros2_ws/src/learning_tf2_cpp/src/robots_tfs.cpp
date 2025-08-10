#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RobotTfBroadcaster : public rclcpp::Node
{
public:
    RobotTfBroadcaster()
    : Node("robot_tf_broadcaster")
    {
        // Initialize transform broadcasters
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        // Declare parameters for static transforms
        // Camera static transform (arm_end -> camera)
        this->declare_parameter("camera_x", 0.1);
        this->declare_parameter("camera_y", 0.0);
        this->declare_parameter("camera_z", 0.05);
        this->declare_parameter("camera_roll", 0.0);
        this->declare_parameter("camera_pitch", 0.0);
        this->declare_parameter("camera_yaw", 0.0);
        
        // Map static transform (world -> map) - robot starting position
        this->declare_parameter("map_x", 0.0);
        this->declare_parameter("map_y", 0.0);
        this->declare_parameter("map_z", 0.0);
        this->declare_parameter("map_roll", 0.0);
        this->declare_parameter("map_pitch", 0.0);
        this->declare_parameter("map_yaw", 0.0);

        // Create subscriptions for dynamic transforms
        robot_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/odom", 10,
            std::bind(&RobotTfBroadcaster::robot_pose_callback, this, _1));

        lift_base_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/lift_base", 10,
            std::bind(&RobotTfBroadcaster::lift_base_callback, this, _1));

        arm_base_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/arm_base", 10,
            std::bind(&RobotTfBroadcaster::arm_base_callback, this, _1));

        arm_end_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/arm_end", 10,
            std::bind(&RobotTfBroadcaster::arm_end_callback, this, _1));

        // Create timer for publishing default transforms
        timer_ = this->create_wall_timer(
            100ms, std::bind(&RobotTfBroadcaster::publish_default_transforms, this));

        // Publish static transforms (world -> map and arm_end -> camera)
        this->publish_static_transforms();

        RCLCPP_INFO(this->get_logger(), "Robot TF Broadcaster node started");
    }

private:
    void publish_default_transforms()
    {
        // Always publish map -> robot_center transform (will be overridden by odom if available)
        if (!robot_pose_received_) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "map";
            t.child_frame_id = "robot_center";
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;
            t.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(t);
        }

        // Always publish robot_center -> lift_base transform (will be overridden if available)
        if (!lift_base_received_) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "robot_center";
            t.child_frame_id = "lift_base";
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.3;
            t.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(t);
        }

        // Always publish lift_base -> arm_base transform (will be overridden if available)
        if (!arm_base_received_) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "lift_base";
            t.child_frame_id = "arm_base";
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.2;
            t.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(t);
        }

        // Always publish arm_base -> arm_end transform (will be overridden if available)
        if (!arm_end_received_) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "arm_base";
            t.child_frame_id = "arm_end";
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.3;
            t.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(t);
        }
    }

    void lift_base_callback(const std::shared_ptr<geometry_msgs::msg::Pose> msg)
    {
        lift_base_received_ = true;
        geometry_msgs::msg::TransformStamped t;

        // Set header
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "robot_center";
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
    void robot_pose_callback(const std::shared_ptr<geometry_msgs::msg::Pose> msg)
    {
        robot_pose_received_ = true;
        geometry_msgs::msg::TransformStamped t;

        // Set header
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "robot_center";

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
        // Publish world -> map static transform (robot starting position)
        geometry_msgs::msg::TransformStamped world_to_map_transform;
        
        // Get map parameters
        double map_x = this->get_parameter("map_x").as_double();
        double map_y = this->get_parameter("map_y").as_double();
        double map_z = this->get_parameter("map_z").as_double();
        double map_roll = this->get_parameter("map_roll").as_double();
        double map_pitch = this->get_parameter("map_pitch").as_double();
        double map_yaw = this->get_parameter("map_yaw").as_double();

        // Set header for world -> map
        world_to_map_transform.header.stamp = this->get_clock()->now();
        world_to_map_transform.header.frame_id = "world";
        world_to_map_transform.child_frame_id = "map";

        // Set translation
        world_to_map_transform.transform.translation.x = map_x;
        world_to_map_transform.transform.translation.y = map_y;
        world_to_map_transform.transform.translation.z = map_z;

        // Convert RPY to quaternion
        tf2::Quaternion map_q;
        map_q.setRPY(map_roll, map_pitch, map_yaw);
        world_to_map_transform.transform.rotation.x = map_q.x();
        world_to_map_transform.transform.rotation.y = map_q.y();
        world_to_map_transform.transform.rotation.z = map_q.z();
        world_to_map_transform.transform.rotation.w = map_q.w();

        // Send world -> map static transform
        static_tf_broadcaster_->sendTransform(world_to_map_transform);

        RCLCPP_INFO(this->get_logger(), 
            "Published static transform from world to map: [%.3f, %.3f, %.3f]",
            map_x, map_y, map_z);

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

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr robot_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr lift_base_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr arm_base_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr arm_end_subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Flags to track if poses have been received
    bool robot_pose_received_ = false;
    bool lift_base_received_ = false;
    bool arm_base_received_ = false;
    bool arm_end_received_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
