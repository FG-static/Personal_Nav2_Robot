#include "odom_evaluator.hpp"

OdomEvaluator::OdomEvaluator() : Node("odom_evaluator") {

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {

            Eigen::Isometry3d eigen_pose;
            tf2::fromMsg(msg->pose.pose, eigen_pose);
            last_odom_ = Sophus::SE3d(eigen_pose.rotation(), eigen_pose.translation());
            has_odom_ = true;
        }
    );
    gt_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ground_truth", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {

            Eigen::Isometry3d eigen_pose;
            tf2::fromMsg(msg->pose.pose, eigen_pose);
            last_gt_ = Sophus::SE3d(eigen_pose.rotation(), eigen_pose.translation());
            has_gt_ = true;
        }
    );
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), [this](){
            
            compute_error();
        }
    );
}

void OdomEvaluator::compute_error() {

    if (!has_odom_ || !has_gt_) return;

    Sophus::SE3d T_err = last_gt_.inverse() * last_odom_;

    double trans_err = T_err.translation().norm(),
        rot_err = T_err.so3().log().norm();

    RCLCPP_INFO(this->get_logger(), 
            "误差统计 -> 平移: %.4f m, 旋转: %.4f rad (%.2f deg)", 
            trans_err, rot_err, rot_err * 180.0 / M_PI);
}

int main(int argc, char** argv) {
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomEvaluator>());
    rclcpp::shutdown();
    return 0;
}