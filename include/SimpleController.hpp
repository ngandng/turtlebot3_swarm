#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SimpleController : public rclcpp::Node
{
public:
    SimpleController();
    ~SimpleController(){};
    void setParameters();

private:
    void timerCallback();
    void odometryCallback(const nav_msgs::msg::Odometry& msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped& msg);

    geometry_msgs::msg::Pose2D pose_;
    std::vector<geometry_msgs::msg::Pose2D> goals_;  // x, y, theta

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPublisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSubscription_;
};
