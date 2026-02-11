#ifndef MY_NAV2_CONTROLLER__DWA_CONTROLLER
#define MY_NAV2_CONTROLLER__DWA_CONTROLLER

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"

namespace my_nav2_controller {

    class MyDWAController : public nav2_core::Controller {

    public:

        MyDWAController() = default;
        ~MyDWAController() override = default;

        /**
         * @brief 初始化插件
         * @param parent 生命周期节点
         * @param name 加载的插件的名称
         * @param tf 用于检查变换
         * @param costmap_ros 代价地图
         * @return 无
         */
        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
        
        void activate() override;
        void deactivate() override;
        void cleanup() override;

        /**
         * @brief 接受规划器传来的路径
         * @param path 路径消息
         * @return 无
         */
        void setPlan(const nav_msgs::msg::Path &path) override;

        void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

        /**
         * @brief 计算下达给底盘的速度（但不是直接下达给底盘）
         * @param pose 带时间戳的小车位姿
         * @param v 目前小车的速度
         * @param goal_checker 到达目标点检测
         * @return 带时间戳的运动向量类型
         */
        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped &pose,
            const geometry_msgs::msg::Twist &v,
            nav2_core::GoalChecker *goal_checker) override;
    protected:
        
        double cal_score(double v, double w);
        double cal_diff_angle(double inialp, double goalalp);

        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        rclcpp::Logger logger_{rclcpp::get_logger("MyDWAController")};
        rclcpp::Clock::SharedPtr clock_;
        nav_msgs::msg::Path global_plan_;
        std::string plugin_name_;

        // DWA参数
        double alpha, beta, gamma; // heading distance velocity分权重
        double lookahead_dist; // 诱饵点
        double max_v; // 最大速度
        double max_w;
        double lim_a; // 加速度限制
        double lim_aw;
        double sim_time_; // 未来时间
        rclcpp::Duration transform_tolerance_{0, 0};
    };
}

#endif // MY_NAV2_CONTROLLER__DWA_CONTROLLER