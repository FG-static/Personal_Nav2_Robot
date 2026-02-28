#include "my_nav2_planner/astar_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

#include <cmath>
#include <unordered_map>

namespace my_nav2_planner {

    void MyAStarPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {

        node_ = parent.lock();
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".unknown_cost", rclcpp::ParameterValue(5.0));
        node_->get_parameter(name_ + ".unknown_cost", unknown_cost_);
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
        node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

        RCLCPP_INFO(node_->get_logger(), "自定义A*规划器配置完成");
    }

    void MyAStarPlanner::activate() { RCLCPP_INFO(node_->get_logger(), "插件已激活"); }
    void MyAStarPlanner::deactivate() { RCLCPP_INFO(node_->get_logger(), "插件已停用"); }
    void MyAStarPlanner::cleanup() { RCLCPP_INFO(node_->get_logger(), "插件已清理"); }

    nav_msgs::msg::Path MyAStarPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal,
        std::function<bool()> /*cancel_checker*/) {

        nav_msgs::msg::Path global_path;
        global_path.header.frame_id = global_frame_;
        global_path.header.stamp = node_->now();

        // 坐标转换
        unsigned int mx_start, my_start, mx_goal, my_goal;
        if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start) ||
            !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {

            RCLCPP_ERROR(node_->get_logger(), "Start or Goal is outside of costmap bounds");
            return global_path;
        }

        // 路径规划
        int width = costmap_->getSizeInCellsX(), 
            height = costmap_->getSizeInCellsY();
        int map_size = width * height;
        std::vector<double> g_values(map_size, std::numeric_limits<double>::max()); // 从起点到每个节点的实际代价
        std::vector<int> parent_map(map_size, -1); // 记录每个节点的父节点索引，便于回溯路径

        typedef std::pair<double, int> Node; // A*算法中的节点，包含f值（g+h）和节点索引
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list; // A*算法的优先队列，按照f值（g+h）排序

        int start_idx = my_start * width + mx_start,
            goal_idx = my_goal * width + mx_goal;

        g_values[start_idx] = 0.0;
        open_list.push({0.0, start_idx});

        bool found_path = false;

        // 记录算法耗时
        auto start_time = std::chrono::steady_clock::now();
        // 开始寻路
        while (!open_list.empty()) {

            int cur_idx = open_list.top().second;
            open_list.pop();

            if (cur_idx == goal_idx) {

                found_path = true;
                break;
            }

            int cx = cur_idx % width,
                cy = cur_idx / width;

            for (int dx = -1; dx <= 1; ++ dx) {

                for (int dy = -1; dy <= 1; ++ dy) {

                    if (dx == 0 && dy == 0) continue;
                    int nx = cx + dx,
                        ny = cy + dy;
                    if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

                    int next_idx = ny * width + nx;
                    unsigned char cost = costmap_->getCost(nx, ny);

                    if (cost >= 250 && cost <= 254) continue;

                    double extra_cost = (cost == 255) ? unknown_cost_ : 0.0,
                        step_cost = std::sqrt(dx * dx + dy * dy);
                    double tentative_g = g_values[cur_idx] + step_cost + extra_cost;

                    if (tentative_g < g_values[next_idx]) {

                        g_values[next_idx] = tentative_g;
                        parent_map[next_idx] = cur_idx;

                        double h_cost = std::sqrt(std::pow(nx - (int)mx_goal, 2) + 
                                        std::pow(ny - (int)my_goal, 2));
                        open_list.push({tentative_g + h_cost, next_idx});
                    }
                }
            }
        }
        
        if (found_path) {

            // 记录算法耗时
            auto end_time = std::chrono::steady_clock::now();
            auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            RCLCPP_INFO(node_->get_logger(), "A* planning completed in %ld ms", duration_ms);
            
            std::vector<int> path_;
            int curr_idx = goal_idx;
            while (curr_idx != -1) {

                path_.push_back(curr_idx);
                curr_idx = parent_map[curr_idx];
            }
            std::reverse(path_.begin(), path_.end());

            for (int idx : path_) {

                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = global_frame_;
                pose.header.stamp = node_->now();

                unsigned int mx = idx % width,
                    my = idx / width;
                double wx, wy;
                costmap_->mapToWorld(mx, my, wx, wy);

                pose.pose.position.x = wx;
                pose.pose.position.y = wy;
                pose.pose.orientation = goal.pose.orientation;
                global_path.poses.push_back(pose);
            }
        } else {

            global_path.poses.clear();
            RCLCPP_WARN(node_->get_logger(), "A* failed to find a path from start to goal");
        }
        return global_path;
    } 
}// namespace my_nav2_planner

PLUGINLIB_EXPORT_CLASS(my_nav2_planner::MyAStarPlanner, nav2_core::GlobalPlanner)