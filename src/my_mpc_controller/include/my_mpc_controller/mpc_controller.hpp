#ifndef MY_MPC_CONTROLLER__MPC_CONTROLLER
#define MY_MPC_CONTROLLER__MPC_CONTROLLER

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"

namespace my_mpc_controller {

    struct PathPoint {

        double 
            x,
            y,
            theta,
            s; // 累积距离
    };
    struct MPCmatrices {

        Eigen::MatrixXd H; // 二次项矩阵
        Eigen::VectorXd f; // 一次项向量
        double G; // 常数项
    };
    class MyMPCController : public nav2_core::Controller {

    public:

        MyMPCController() = default;
        ~MyMPCController() override = default;

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

        // 线性化
        void updateDiscreteModel(const double v, const double theta, 
            Eigen::Matrix3d &A, Eigen::Matrix2d &B);

        Eigen::VectorXd sampleReferencePath(
            const std::vector<PathPoint> &global_path,
            const Eigen::Vector3d &cur_pose,
            double v_ref,
            int N,
            double dt);

        /**
         * @brief 将 nav_msgs::Path 转换为带有累计距离 s 的 ReferencePoint 向量
         * @param path 订阅到的全局路径
         * @return 处理后的参考点序列
         */
        std::vector<PathPoint> preprocessPath(const nav_msgs::msg::Path &path);

        // 代换矩阵
        void replaceLargeMatrix(const Eigen::Matrix3d &A, const Eigen::Matrix2d &B);

        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        rclcpp::Logger logger_{rclcpp::get_logger("MyMPCController")};
        rclcpp::Clock::SharedPtr clock_;
        nav_msgs::msg::Path global_plan_;
        std::string plugin_name_;
        std::vector<PathPoint> processed_path_;

        // MPC参数
        int N_ = 10; // 预测区间
        double dt_ = 0.1; // 采样时间 - 动态
        rclcpp::Duration transform_tolerance_{0, 0};
        Eigen::Matrix3d Q_; // 状态权重矩阵
        Eigen::Matrix2d R_; // 控制权重矩阵
    };
}

#endif // MY_MPC_CONTROLLER__MPC_CONTROLLER