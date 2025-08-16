#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

class CameraDisplacementPublisher : public rclcpp::Node
{
public:
    CameraDisplacementPublisher()
    : Node("camera_displacement_publisher")
    {
        // Initialize the transform buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create the publisher
        displacement_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
            "camera_displacement", 10);

        // Create timer to publish displacement periodically
        timer_ = this->create_wall_timer(
            100ms, std::bind(&CameraDisplacementPublisher::publish_displacement, this));

        RCLCPP_INFO(this->get_logger(), "Camera Displacement Publisher started");
        RCLCPP_INFO(this->get_logger(), "Publishing on topic: /camera_displacement");
    }

private:
    void publish_displacement()
    {
        try
        {
            // Look up the transform from camera to base_link
            geometry_msgs::msg::TransformStamped transform_stamped;
            
            // Check if transform is available
            if (tf_buffer_->canTransform("base_link", "camera", tf2::TimePointZero, tf2::durationFromSec(0.1)))
            {
                transform_stamped = tf_buffer_->lookupTransform(
                    "base_link", "camera", tf2::TimePointZero);

                // Create pose message
                geometry_msgs::msg::Pose displacement_pose;
                displacement_pose.position.x = transform_stamped.transform.translation.x;
                displacement_pose.position.y = transform_stamped.transform.translation.y;
                displacement_pose.position.z = transform_stamped.transform.translation.z;
                displacement_pose.orientation = transform_stamped.transform.rotation;

                // Publish the displacement
                displacement_publisher_->publish(displacement_pose);

                // Log occasionally (every 50 calls = ~5 seconds at 100ms rate)
                static int log_counter = 0;
                if (++log_counter >= 50)
                {
                    double distance = std::sqrt(
                        displacement_pose.position.x * displacement_pose.position.x +
                        displacement_pose.position.y * displacement_pose.position.y +
                        displacement_pose.position.z * displacement_pose.position.z);
                    
                    RCLCPP_INFO(this->get_logger(), 
                        "Publishing displacement - X: %.3f, Y: %.3f, Z: %.3f, Distance: %.3f", 
                        displacement_pose.position.x, displacement_pose.position.y, 
                        displacement_pose.position.z, distance);
                    log_counter = 0;
                }
            }
            else
            {
                // Only log warning occasionally to avoid spam
                static int warn_counter = 0;
                if (++warn_counter >= 50)
                {
                    RCLCPP_WARN(this->get_logger(), 
                        "Transform from camera to base_link not available");
                    warn_counter = 0;
                }
            }
        }
        catch (const tf2::TransformException & ex)
        {
            // Only log errors occasionally to avoid spam
            static int error_counter = 0;
            if (++error_counter >= 50)
            {
                RCLCPP_ERROR(this->get_logger(), 
                    "Transform lookup failed: %s", ex.what());
                error_counter = 0;
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr displacement_publisher_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraDisplacementPublisher>());
    rclcpp::shutdown();
    return 0;
}
