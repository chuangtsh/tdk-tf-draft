#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <thread>
#include <cstring>
#include <cmath>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

// Global flag for shutdown
volatile bool g_shutdown_requested = false;

// Signal handler
void sigint_handler(int sig)
{
    (void)sig;
    g_shutdown_requested = true;
}

class RobotSimulator : public rclcpp::Node
{
public:
    RobotSimulator()
    : Node("robot_simulator"), robot_yaw_(0.0), arm_end_pitch_(0.0)
    {
        // Create publishers for each pose topic
        odom_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/odom", 10);
        lift_base_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/lift_base", 10);
        arm_base_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/arm_base", 10);
        arm_end_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/arm_end", 10);

        // Initialize poses
        robot_pose_.position.x = 0.0;
        robot_pose_.position.y = 0.0;
        robot_pose_.position.z = 0.0;
        robot_pose_.orientation.w = 1.0;

        arm_base_pose_.position.x = 0.0;
        arm_base_pose_.position.y = 0.0;
        arm_base_pose_.position.z = 0.2;
        arm_base_pose_.orientation.w = 1.0;

        lift_base_pose_.position.x = 0.0;
        lift_base_pose_.position.y = 0.0;
        lift_base_pose_.position.z = 0.3;
        lift_base_pose_.orientation.w = 1.0;

        arm_end_pose_.position.x = 0.0;
        arm_end_pose_.position.y = 0.0;
        arm_end_pose_.position.z = 0.3;
        arm_end_pose_.orientation.w = 1.0;

        // Create timer for publishing
        timer_ = this->create_wall_timer(
            100ms, std::bind(&RobotSimulator::publish_poses, this));

        // Print instructions
        print_instructions();
    }

private:
    void print_instructions()
    {
        std::cout << "\n=== Robot Pose Simulator ===" << std::endl;
        std::cout << "Use the following keys to control poses:" << std::endl;
        std::cout << "\n--- Robot (world -> robot_center) ---" << std::endl;
        std::cout << "W/S: Move forward/backward (robot's local frame)" << std::endl;
        std::cout << "A/D: Move left/right (robot's local frame)" << std::endl;
        std::cout << "Q/E: Rotate left/right (Yaw)" << std::endl;
        std::cout << "\n--- Lift Base (robot_center -> lift_base) ---" << std::endl;
        std::cout << "Z/X: Move forward/backward (X)" << std::endl;
        std::cout << "C/V: Move left/right (Y)" << std::endl;
        std::cout << "\n--- Arm Base (lift_base -> arm_base) ---" << std::endl;
        std::cout << "U/O: Move up/down (Z) - cascade lift" << std::endl;
        std::cout << "\n--- Arm End (arm_base -> arm_end) ---" << std::endl;
        std::cout << "I/K: Move forward/backward (X)" << std::endl;
        std::cout << "J/L: Move left/right (Y)" << std::endl;
        std::cout << "T/G: Move up/down (Z)" << std::endl;
        std::cout << "R/Y: Rotate pitch up/down" << std::endl;
        std::cout << "\nPress Ctrl-C to exit" << std::endl;
        std::cout << "Step size: 0.1 units, 0.1 radians" << std::endl;
        std::cout << "===========================\n" << std::endl;
        printf("Robot: [%.2f, %.2f, %.2f] | Lift: [%.2f, %.2f, %.2f] | Arm Base: [%.2f, %.2f, %.2f] | Arm End: [%.2f, %.2f, %.2f]\n",
               robot_pose_.position.x, robot_pose_.position.y, robot_pose_.position.z,
               lift_base_pose_.position.x, lift_base_pose_.position.y, lift_base_pose_.position.z,
               arm_base_pose_.position.x, arm_base_pose_.position.y, arm_base_pose_.position.z,
               arm_end_pose_.position.x, arm_end_pose_.position.y, arm_end_pose_.position.z);
    }

    void publish_poses()
    {
        // Always publish current poses at 10Hz
        odom_publisher_->publish(robot_pose_);
        lift_base_publisher_->publish(lift_base_pose_);
        arm_base_publisher_->publish(arm_base_pose_);
        arm_end_publisher_->publish(arm_end_pose_);
    }

    void update_pose(char key)
    {
        const double step_size = 0.1;
        const double angle_step = 0.1;
        bool pose_changed = true;

        switch (key) {
            // Robot controls - movement relative to robot's current orientation
            case 'w': case 'W':
            {
                // Move forward in robot's local coordinate frame
                double cos_yaw = cos(robot_yaw_);
                double sin_yaw = sin(robot_yaw_);
                robot_pose_.position.x += step_size * cos_yaw;
                robot_pose_.position.y += step_size * sin_yaw;
                break;
            }
            case 's': case 'S':
            {
                // Move backward in robot's local coordinate frame
                double cos_yaw = cos(robot_yaw_);
                double sin_yaw = sin(robot_yaw_);
                robot_pose_.position.x -= step_size * cos_yaw;
                robot_pose_.position.y -= step_size * sin_yaw;
                break;
            }
            case 'a': case 'A':
            {
                // Move left in robot's local coordinate frame
                double cos_yaw = cos(robot_yaw_);
                double sin_yaw = sin(robot_yaw_);
                robot_pose_.position.x -= step_size * sin_yaw;
                robot_pose_.position.y += step_size * cos_yaw;
                break;
            }
            case 'd': case 'D':
            {
                // Move right in robot's local coordinate frame
                double cos_yaw = cos(robot_yaw_);
                double sin_yaw = sin(robot_yaw_);
                robot_pose_.position.x += step_size * sin_yaw;
                robot_pose_.position.y -= step_size * cos_yaw;
                break;
            }
            case 'q': case 'Q':
                robot_yaw_ += angle_step;
                set_quaternion_from_yaw(robot_pose_, robot_yaw_);
                break;
            case 'e': case 'E':
                robot_yaw_ -= angle_step;
                set_quaternion_from_yaw(robot_pose_, robot_yaw_);
                break;

            // Lift base controls - moves freely on XY plane relative to robot_center frame
            case 'z': case 'Z':
                lift_base_pose_.position.x += step_size;
                break;
            case 'x': case 'X':
                lift_base_pose_.position.x -= step_size;
                break;
            case 'c': case 'C':
                lift_base_pose_.position.y += step_size;
                break;
            case 'v': case 'V':
                lift_base_pose_.position.y -= step_size;
                break;

            // Arm base controls - cascade lift, only Z movement relative to lift_base frame
            case 'u': case 'U':
                arm_base_pose_.position.z += step_size;
                break;
            case 'o': case 'O':
                arm_base_pose_.position.z -= step_size;
                break;

            // Arm end controls - relative to arm_base frame
            case 'i': case 'I':
                arm_end_pose_.position.x += step_size;
                break;
            case 'k': case 'K':
                arm_end_pose_.position.x -= step_size;
                break;
            case 'j': case 'J':
                arm_end_pose_.position.y += step_size;
                break;
            case 'l': case 'L':
                arm_end_pose_.position.y -= step_size;
                break;
            case 't': case 'T':
                arm_end_pose_.position.z += step_size;
                break;
            case 'g': case 'G':
                arm_end_pose_.position.z -= step_size;
                break;
            case 'r': case 'R':
                arm_end_pitch_ += angle_step;
                set_quaternion_from_pitch(arm_end_pose_, arm_end_pitch_);
                break;
            case 'y': case 'Y':
                arm_end_pitch_ -= angle_step;
                set_quaternion_from_pitch(arm_end_pose_, arm_end_pitch_);
                break;

            default:
                pose_changed = false;
                break;
        }

        if (pose_changed) {
            printf("Robot: [%.2f, %.2f, %.2f] | Lift: [%.2f, %.2f, %.2f] | Arm Base: [%.2f, %.2f, %.2f] | Arm End: [%.2f, %.2f, %.2f]\r",
                   robot_pose_.position.x, robot_pose_.position.y, robot_pose_.position.z,
                   lift_base_pose_.position.x, lift_base_pose_.position.y, lift_base_pose_.position.z,
                   arm_base_pose_.position.x, arm_base_pose_.position.y, arm_base_pose_.position.z,
                   arm_end_pose_.position.x, arm_end_pose_.position.y, arm_end_pose_.position.z);
            fflush(stdout);
        }
    }

    void set_quaternion_from_yaw(geometry_msgs::msg::Pose& pose, double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
    }

    void set_quaternion_from_pitch(geometry_msgs::msg::Pose& pose, double pitch)
    {
        tf2::Quaternion q;
        q.setRPY(0, pitch, 0);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
    }

public:
    void key_loop()
    {
        char c;
        struct termios cooked, raw;
        
        tcgetattr(0, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(0, TCSANOW, &raw);
        
        puts("Reading from keyboard");
        puts("Use keys to move the robot components. Press Ctrl-C to exit.");

        for(;;)
        {
            if(read(0, &c, 1) < 0)
            {
                perror("read():");
                break;
            }
            
            update_pose(c);
            
            if(g_shutdown_requested)
                break;
        }

        tcsetattr(0, TCSANOW, &cooked);
    }

private:
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr odom_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr lift_base_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr arm_base_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr arm_end_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Poses
    geometry_msgs::msg::Pose robot_pose_;
    geometry_msgs::msg::Pose lift_base_pose_;
    geometry_msgs::msg::Pose arm_base_pose_;
    geometry_msgs::msg::Pose arm_end_pose_;

    // Angles for tracking rotations
    double robot_yaw_;
    double arm_end_pitch_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    signal(SIGINT, sigint_handler);
    
    auto simulator = std::make_shared<RobotSimulator>();
    
    std::thread spin_thread([&simulator]() {
        rclcpp::spin(simulator);
    });
    
    simulator->key_loop();
    
    g_shutdown_requested = true;
    rclcpp::shutdown();
    spin_thread.join();
    
    return 0;
}