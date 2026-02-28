#ifndef MY_RRTSTAR_PLANNER__RRTSTAR_PLANNER
#define MY_RRTSTAR_PLANNER__RRTSTAR_PLANNER

#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <random>
#include <cmath>

#include "nav2_core/global_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

namespace my_rrtstar_planner {

    // RRT* 节点（使用世界坐标，单位 m）
    struct RRTNode {

        int pos_idx; // map坐标索引，pos_idx = y * width + x
        int parent; // 在 tree 中的索引，-1 为根
        double cost; // 从起点到该节点的累计代价
        RRTNode() : pos_idx(0), parent(-1), cost(0.0) {}
        RRTNode(int pos_idx, int par, double c) : pos_idx(pos_idx), parent(par), cost(c) {}
    };

    class MyRRTStarPlanner : public nav2_core::GlobalPlanner {

    public:

        MyRRTStarPlanner() = default;
        ~MyRRTStarPlanner() override = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    
        void activate() override;
        void deactivate() override;
        void cleanup() override;

        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal,
            std::function<bool()> /*cancel_checker*/) override;
    private:

        bool isCollisionFreePath(int idx1, int idx2); // 检查两个节点之间的路径是否碰撞
        void updateChildrenCost(int parent_idx, double cost_delta); // 递归更新子节点代价

        std::shared_ptr<tf2_ros::Buffer> tf_;
        nav2_util::LifecycleNode::SharedPtr node_;
        nav2_costmap_2d::Costmap2D *costmap_;
        std::string global_frame_, name_;
        
        std::vector<RRTNode> tree; // RRT*树结构

        // 随机生成器
        std::mt19937 rng;
        std::uniform_real_distribution<double> uni_x;
        std::uniform_real_distribution<double> uni_y;

        // RRT*算法参数
        double step_size_ = 0.5; // 扩展步长，单位 m
        int max_iterations_ = 50000; // 最大迭代次数
        int max_iterations_after_goal_ = 1000; // 在找到目标后继续迭代以优化路径的次数
        double 
            search_radius_ = 2.0, // 用于重连
            goal_sample_rate_ = 0.5, // 采样直接采中目标的概率
            goal_tolerance_ = 0.2, // 认为到达目标的距离阈值，单位 m
            collision_check_resolution_ = 0.02; // 碰撞检查的插值分辨率，单位 m
    };
} // namespace my_rrtstar_planner

#endif // MY_RRTSTAR_PLANNER__RRTSTAR_PLANNER