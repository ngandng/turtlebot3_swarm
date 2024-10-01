#include "SimpleController.hpp"

SimpleController::SimpleController() : Node("simple_controller")
{
    setParameters();
    velPublisher_     = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    odomSubscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 50, std::bind(&SimpleController::odometryCallback, this, _1));

    timer_ = this->create_wall_timer(100ms, std::bind(&SimpleController::timerCallback, this));
}

void SimpleController::setParameters() {}

void SimpleController::timerCallback()
{
    if (goals_.size() == 0)
    {
        geometry_msgs::msg::Twist vel;
        velPublisher_->publish(vel);
        return;
    }

    /* The main controller */
    geometry_msgs::msg::Pose2D goal = goals_[0];

    double xDiff = goal.x - pose_.x;
    double yDiff = goal.y - pose_.y;

    if (sqrt(xDiff * xDiff + yDiff * yDiff) < 0.1)
    {
        RCLCPP_INFO(this->get_logger(), "Reach the target at x: %f (m), y: %f (m), theta: %f (rad)",
                    goal.x, goal.y, goal.theta);
        goals_.erase(goals_.begin());
    }

    double rho   = sqrt(xDiff * xDiff + yDiff * yDiff);
    double alpha = atan2(yDiff, xDiff) - pose_.theta;
    while (alpha > M_PI)
        alpha -= 2 * M_PI;
    while (alpha < -M_PI)
        alpha += 2 * M_PI;
    double beta = goal.theta - pose_.theta - alpha;
    while (beta > M_PI)
        beta -= 2 * M_PI;
    while (beta < -M_PI)
        beta += 2 * M_PI;

    double v = 0.9 * rho;
    double w = 1.5 * alpha + -0.3 * beta;

    if (alpha > M_PI / 2 || alpha < -M_PI / 2) v = -v;

    geometry_msgs::msg::Twist vel;
    vel.linear.x = v;
    vel.angular.z = w;
    velPublisher_->publish(vel);
}

void SimpleController::odometryCallback(const nav_msgs::msg::Odometry& msg)
{
    tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose_.x     = msg.pose.pose.position.x;
    pose_.y     = msg.pose.pose.position.y;
    pose_.theta = yaw;
}

void SimpleController::goalCallback(const geometry_msgs::msg::PoseStamped& msg)
{
    // Add goal to stack
    tf2::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                      msg.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    geometry_msgs::msg::Pose2D goal;
    goal.x     = msg.pose.position.x;
    goal.y     = msg.pose.position.y;
    goal.theta = yaw;

    goals_.push_back(goal);
}