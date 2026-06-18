#ifndef MY_NAV2_SMOOTHER__BSPLINE_SMOOTHER
#define MY_NAV2_SMOOTHER__BSPLINE_SMOOTHER

#include <nav2_costmap_2d/cost_values.hpp>
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

    struct SegmentViolation {

        int index = -1;
        double ratio = 1.0;
    };

    struct DynamicReport {

        bool ok = true;

        double max_vel = 0.0;
        double max_acc = 0.0;
        double max_jerk = 0.0;

        std::vector<SegmentViolation> vel_vios;
        std::vector<SegmentViolation> acc_vios;
        std::vector<SegmentViolation> jerk_vios;
    };

    struct CorridorBounds {

        std::vector<double>
            lower_x,
            upper_x,
            lower_y,
            upper_y;
    };

    struct GridBox {

        int min_mx = 0,
            max_mx = 0,
            min_my = 0,
            max_my = 0;
    };

    struct CorridorViolation {

        int index = -1;

        double dx = 0.0;
        double dy = 0.0;

        double dist = 0.0;
    };

    struct CorridorReport {

        bool ok = true;
        std::vector<CorridorViolation>
            point_vios,
            segment_vios;
    };

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
        void applyBSplineAlgorithm(
            nav_msgs::msg::Path &path,
            const nav_msgs::msg::Path &raw_path
        );
        bool solveBSplineQP(
            const std::vector<double> &p_ref_x,
            const std::vector<double> &p_ref_y,
            double w_s,
            double w_g,
            std::vector<double> &p_smooth_x,
            std::vector<double> &p_smooth_y);
        bool solveBSplineQPOnce(
            const std::vector<double> &p_ref_x,
            const std::vector<double> &p_ref_y,
            const std::vector<double> &dt_segment,
            const CorridorBounds &bounds,
            double w_s,
            double w_g,
            std::vector<double> &p_smooth_x,
            std::vector<double> &p_smooth_y);

        // 时间分配
        bool computeTimeAllocation(
            const std::vector<double> &p_ref_x,
            const std::vector<double> &p_ref_y,
            std::vector<double> &output
        ) const;
        DynamicReport checkDynamicFeasibility(
            const std::vector<double> &p_x,
            const std::vector<double> &p_y,
            const std::vector<double> &dt_segment
        ) const;
        void inflateTimeAllocation(
            const DynamicReport &report,
            std::vector<double> &dt_segment
        ) const;

        // 几何轨迹超调优化
        bool worldToMap(
            double wx,
            double wy,
            unsigned int &mx,
            unsigned int &my
        ) const;
        bool isCellFree(int mx, int my) const;
        bool isColumnFree(int mx, int min_my, int max_my) const;
        bool isRowFree(int my, int min_mx, int max_mx) const;
        GridBox expandBoxFromCell(
            int seed_mx,
            int seed_my,
            int max_expand_cells
        ) const;

        // box处理
        CorridorReport checkCorridorFeasibility(
            const std::vector<double> &p_x,
            const std::vector<double> &p_y,
            const CorridorBounds &bounds
        ) const;
        bool isSegmentCollisionFree(
            double x0, double y0,
            double x1, double y1
        ) const;
        bool contains(
            const GridBox &a,
            const GridBox &b
        ) const;
        bool hasIntersection(
            const GridBox &a,
            const GridBox &b
        ) const;
        bool overlapRatio(
            const GridBox &a,
            const GridBox &b
        ) const;
        GridBox mergeByIntersection(
            const GridBox &a,
            const GridBox &b
        ) const;
        CorridorBounds buildCorridorBounds(
            const std::vector<double> &p_ref_x,
            const std::vector<double> &p_ref_y
        ) const;

        // bspline参数
        double
            w_smooth_ = 10.0,
            w_guide_ = 1.0,
            max_vel_ = 1.0,
            max_acc_ = 1.0,
            max_jerk_ = 2.0,
            min_dt_ = 0.05,
            max_dt_ = 1.0,
            time_inflation_factor_ = 1.15,
            max_time_scale_ = 2.0;
        int max_outer_iterations_ = 3;
        double
            corridor_max_expand_dist_ = 0.8,
            corridor_min_half_width_ = 0.05,
            corridor_overlap_threshold_ = 0.8,
            corridor_collision_check_resolution_ = 0.03;
        unsigned char corridor_lethal_cost_threshold_ =
            nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

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
