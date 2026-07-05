#ifndef MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_G2O_SMOOTHER
#define MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_G2O_SMOOTHER

#include <memory>
#include <string>

#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

#include "my_nav2_smoother/esdf_g2o_smoother/esdf_g2o_optimizer.hpp"
#include "my_nav2_smoother/esdf_g2o_smoother/esdf_g2o_types.hpp"
#include "my_nav2_smoother/esdf_g2o_smoother/esdf_map.hpp"

namespace my_nav2_smoother {

class EsdfG2oSmoother : public nav2_core::Smoother {

public:

    EsdfG2oSmoother() = default;
    ~EsdfG2oSmoother() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
        std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    bool smooth(
        nav_msgs::msg::Path &path,
        const rclcpp::Duration &max_time) override;

private:

    void declareParameters();
    bool initializeTrajectory(
        const nav_msgs::msg::Path &path,
        TrajectoryPoints &trajectory) const;
    void updatePathFromTrajectory(
        nav_msgs::msg::Path &path,
        const TrajectoryPoints &trajectory) const;

    nav2_util::LifecycleNode::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
    std::string name_;

    EsdfG2oSmootherConfig config_;
    EsdfMap esdf_map_;
    EsdfG2oOptimizer optimizer_;
};

} // namespace my_nav2_smoother

#endif // MY_NAV2_SMOOTHER__ESDF_G2O_SMOOTHER__ESDF_G2O_SMOOTHER
