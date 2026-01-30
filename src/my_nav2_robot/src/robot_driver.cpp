#include "robot_driver.hpp"

RobotDriver::RobotDriver() : Node("robot_driver") {

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, 
        [this](const geometry_msgs::msg::Twist::SharedPtr msg){

            linear_v_ = msg->linear.x;
            angular_w_ = msg->angular.z;
        }
    );
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(20ms, std::bind(&RobotDriver::update_state, this));
    x_ = y_ = theta_ = linear_v_ = angular_w_ = 0.0;
    last_time_ = this->get_clock()->now();
}

void RobotDriver::update_state() {

    auto cur_time = this->get_clock()->now();
    double dt = (cur_time - last_time_).seconds();
    last_time_ = cur_time;

    x_ += linear_v_ * cos(theta_) * dt;
    y_ += linear_v_ * sin(theta_) * dt;
    theta_ += angular_w_ * dt;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    // 动态转换odom->base_link 给Rviz看的
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = cur_time;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);

    // 传给规划器的
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = cur_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = linear_v_;
    odom.twist.twist.angular.z = angular_w_;

    // 协方差，1e-9表示更相信模拟数据
    for(int i = 0; i < 36; i ++) if(i == 0 || i == 7 || i == 35) odom.pose.covariance[i] = 1e-9;

    odom_pub_->publish(odom);
}

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDriver>());
    rclcpp::shutdown();
    return 0;
}