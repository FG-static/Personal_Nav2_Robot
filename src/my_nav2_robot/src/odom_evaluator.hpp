#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sophus/se3.hpp"
#include "eigen3/Eigen/Core"
#include "tf2_eigen/tf2_eigen.hpp"

class OdomEvaluator : public rclcpp::Node {

public:

    OdomEvaluator();
private:

    void compute_error();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    Sophus::SE3d last_odom_, last_gt_;
    bool has_odom_ = false, has_gt_ = false;
};