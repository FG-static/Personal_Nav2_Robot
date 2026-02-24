#ifndef MY_NAV2_SMOOTHER__BSPLINE_SMOOTHER
#define MY_NAV2_SMOOTHER__BSPLINE_SMOOTHER

#include <vector>
#include <memory>
#include <string>
#include "Eigen/Dense"
#include "Eigen/Sparse"

#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_core/smoother.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace my_bspline_smoother {

    class MyBSplineSmoother : public nav2_core::Smoother {

    public:

        MyBSplineSmoother() = default;
        ~MyBSplineSmoother() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
            std::string name, 
            std::shared_ptr<tf2_ros::Buffer> /*tf*/,
            std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
            std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> /*footprint_sub*/) override;

        void cleanup() override;
        void activate() override;
        void deactivate() override;

        bool smooth(
            nav_msgs::msg::Path &path,
            const rclcpp::Duration &/*max_time*/) override;
    private:

        // 平滑算法
        void applyBSplineAlgorithm(nav_msgs::msg::Path &path, const nav_msgs::msg::Path &raw_path);
        bool solveBSplineQP(
            const std::vector<double> &p_ref,
            double w_s,
            double w_g,
            std::vector<double> &p_smooth);

        // bspline参数
        double 
            w_smooth_ = 10.0,
            w_guide_ = 1.0;

        const double Q_data[4][4] = {
            { 0.333333, -0.500000,  0.000000,  0.166667},
            {-0.500000,  1.000000, -0.500000,  0.000000},
            { 0.000000, -0.500000,  1.000000, -0.500000},
            { 0.166667,  0.000000, -0.500000,  0.333333}
        };
        nav2_util::LifecycleNode::SharedPtr node_;
        std::string name_;
        std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
    };
} // my_bspline_smoother

#endif // MY_NAV2_SMOOTHER__BSPLINE_SMOOTHER