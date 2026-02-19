#ifndef MY_NAV2_SMOOTHER__GRADIENT_SMOOTHER
#define MY_NAV2_SMOOTHER__GRADIENT_SMOOTHER

#include <vector>
#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_core/smoother.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace my_nav2_smoother {

    class MyGradientSmoother : public nav2_core::Smoother {

    public:

        MyGradientSmoother() = default;
        ~MyGradientSmoother() override = default;

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
        void applyGradientDescent(nav_msgs::msg::Path &path, const nav_msgs::msg::Path &raw_path);

        // 平滑力、拉回力权重
        double alpha_ = 0.1, beta_ = 0.5;
        nav2_util::LifecycleNode::SharedPtr node_;
        std::string name_;
        int max_iterations_ = 500;
        std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
    };
} // my_nav2_smoother

#endif // MY_NAV2_SMOOTHER__GRADIENT_SMOOTHER