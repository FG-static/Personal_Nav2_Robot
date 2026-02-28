#include "my_nav2_planner/rrtstar_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

#include <cmath>
#include <unordered_map>

namespace my_rrtstar_planner {

    void MyRRTStarPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {

        node_ = parent.lock();
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".step_size", rclcpp::ParameterValue(1.0));
        node_->get_parameter(name_ + ".step_size", step_size_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".max_iterations", rclcpp::ParameterValue(50000));
        node_->get_parameter(name_ + ".max_iterations", max_iterations_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".search_radius", rclcpp::ParameterValue(2.0));
        node_->get_parameter(name_ + ".search_radius", search_radius_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".goal_sample_rate", rclcpp::ParameterValue(0.5));
        node_->get_parameter(name_ + ".goal_sample_rate", goal_sample_rate_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".goal_tolerance", rclcpp::ParameterValue(0.2));
        node_->get_parameter(name_ + ".goal_tolerance", goal_tolerance_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".collision_check_resolution", rclcpp::ParameterValue(0.02));
        node_->get_parameter(name_ + ".collision_check_resolution", collision_check_resolution_);

        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".max_iterations_after_goal", rclcpp::ParameterValue(1000));
        node_->get_parameter(name_ + ".max_iterations_after_goal", max_iterations_after_goal_);

        // 初始化随机数生成器
        rng.seed(std::random_device{}());
        double 
            origin_x = costmap_ros->getCostmap()->getOriginX(),
            origin_y = costmap_ros->getCostmap()->getOriginY(),
            size_x = costmap_ros->getCostmap()->getSizeInMetersX(),
            size_y = costmap_ros->getCostmap()->getSizeInMetersY();
        uni_x = std::uniform_real_distribution<double>(origin_x, origin_x + size_x);
        uni_y = std::uniform_real_distribution<double>(origin_y, origin_y + size_y);

        RCLCPP_INFO(node_->get_logger(), "自定义RRT*规划器配置完成");
    }

    void MyRRTStarPlanner::activate() { RCLCPP_INFO(node_->get_logger(), "插件已激活"); }
    void MyRRTStarPlanner::deactivate() { RCLCPP_INFO(node_->get_logger(), "插件已停用"); }
    void MyRRTStarPlanner::cleanup() { RCLCPP_INFO(node_->get_logger(), "插件已清理"); }

    bool MyRRTStarPlanner::isCollisionFreePath(int idx1, int idx2) {

        // 将地图索引转换为地图坐标，再转换为世界坐标进行检查
        unsigned int width = costmap_->getSizeInCellsX();
        unsigned int mx1 = idx1 % width, my1 = idx1 / width;
        unsigned int mx2 = idx2 % width, my2 = idx2 / width;
        
        double wx1, wy1, wx2, wy2;
        costmap_->mapToWorld(mx1, my1, wx1, wy1);
        costmap_->mapToWorld(mx2, my2, wx2, wy2);
        
        // 在世界坐标中进行线性插值检查
        double dist = std::sqrt((wx2 - wx1) * (wx2 - wx1) + (wy2 - wy1) * (wy2 - wy1));
        int steps = std::max(1, static_cast<int>(dist / collision_check_resolution_));
        
        for (int i = 0; i <= steps; ++ i) {

            double t = (steps > 0) ? static_cast<double>(i) / steps : 0.0;
            double 
                wx = (1 - t) * wx1 + t * wx2,
                wy = (1 - t) * wy1 + t * wy2;
            
            unsigned int mx, my;
            if (!costmap_->worldToMap(wx, wy, mx, my)) return false; // 超出地图范围
            
            if (costmap_->getCost(mx, my) >= 250) return false;
        }
        return true;
    }

    nav_msgs::msg::Path MyRRTStarPlanner::createPlan(
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
        // 将米为单位的参数转换为地图格子数，保持单位一致
        double resolution = costmap_->getResolution();
        double step_size_cells = step_size_ / resolution;
        double search_radius_cells = search_radius_ / resolution;
        double goal_tolerance_cells = goal_tolerance_ / resolution;
        // TODO：RRT*算法实现

        int start_idx = my_start * width + mx_start,
            goal_idx = my_goal * width + mx_goal;

        tree.clear();
        tree.emplace_back(start_idx, -1, 0.0); // 将起点加入树中，父节点索引为-1，代价为0
        double best_cost = std::numeric_limits<double>::infinity(); // 记录找到的路径的最优代价
        int goal_node_idx = -1; // 记录找到的目标节点在树中的索引

        bool found_path = false;

        // 开始寻路
        int stop_iter = 0; // 优化迭代计数器
        // 记录算法耗时
        auto start_time = std::chrono::steady_clock::now();
        for (int iter = 0; iter < max_iterations_; ++ iter) {

            // FIXME：RRT*算法有bug，待修复
            // 如果找到更优路径，更新 best_cost 和 found_path
            double r = std::uniform_real_distribution<double>(0.0, 1.0)(rng);
            int sample_idx; // 采样点在地图中的索引
            if (r < goal_sample_rate_) sample_idx = goal_idx; // 以一定概率直接采样目标点
            else {

                double rx = uni_x(rng), ry = uni_y(rng);
                unsigned int mx, my;
                if (!costmap_->worldToMap(rx, ry, mx, my)) continue; // 采样点在地图外，丢弃
                // 如果采样点落在障碍上也丢弃
                if (costmap_->getCost(mx, my) >= 250) continue;
                sample_idx = my * width + mx;
            }

            // 在搜索半径内找到代价最小的节点作为父节点
            int best_parent_idx = 0; // 默认为树的第一个节点（起点）
            double best_parent_cost = std::numeric_limits<double>::infinity();
            
            for (size_t i = 0; i < tree.size(); ++ i) {

                unsigned int mx, my; // 其他节点在地图中的坐标
                my = tree[i].pos_idx / width;
                mx = tree[i].pos_idx % width;
                double 
                    ddx = mx - (sample_idx % width), ddy = my - (sample_idx / width),
                    dist_to_sample = sqrt(ddx * ddx + ddy * ddy);
                
                if (dist_to_sample < search_radius_cells) {
                    double cost_via_this_node = tree[i].cost + dist_to_sample;
                    
                    // 检查这条路径是否碰撞
                    if (!isCollisionFreePath(tree[i].pos_idx, sample_idx)) continue;
                    
                    if (cost_via_this_node < best_parent_cost) {
                        best_parent_cost = cost_via_this_node;
                        best_parent_idx = i;
                    }
                }
            }
            
            unsigned int sample_mx = sample_idx % width;
            unsigned int sample_my = sample_idx / width;

            // 从选择的最优父节点向采样点扩展，在地图坐标中计算方向向量（格子单位）
            unsigned int parent_mx, parent_my;
            parent_my = tree[best_parent_idx].pos_idx / width;
            parent_mx = tree[best_parent_idx].pos_idx % width;

            double 
                dx = sample_mx - parent_mx, dy = sample_my - parent_my,
                dist = sqrt(dx * dx + dy * dy);
            if (dist > step_size_cells && dist > 0.0) {
                dx *= step_size_cells / dist;
                dy *= step_size_cells / dist;
            }

            // 计算新的地图坐标，确保在整数范围内
            unsigned int 
                new_mx = parent_mx + static_cast<int>(round(dx)),
                new_my = parent_my + static_cast<int>(round(dy));
            
            // 检查是否在地图范围内
            if (new_mx >= width || new_my >= height) continue;
                
            int new_idx = new_my * width + new_mx;

            // 如果没有移动则跳过
            if (new_idx == tree[best_parent_idx].pos_idx) continue;

            // 检查新节点是否与障碍物碰撞
            if (costmap_->getCost(new_mx, new_my) >= 250)  continue; // 新节点是障碍物，丢弃

            // 检查从最优父节点到新节点的路径是否碰撞
            if (!isCollisionFreePath(tree[best_parent_idx].pos_idx, new_idx)) continue;

            // 将新节点加入树中
            double cost_to_new_node = tree[best_parent_idx].cost + sqrt(dx * dx + dy * dy);
            tree.emplace_back(new_idx, best_parent_idx, cost_to_new_node);

            // 重连附近节点 Rewire
            for (size_t i = 0; i < tree.size() - 1; ++ i) { // 不包括刚加入的新节点

                unsigned int curr_mx, curr_my;
                curr_my = tree[i].pos_idx / width;
                curr_mx = tree[i].pos_idx % width;

                double 
                    ddx = curr_mx - new_mx, ddy = curr_my - new_my,
                    dist_to_new_node = sqrt(ddx * ddx + ddy * ddy);

                if (dist_to_new_node < search_radius_cells) {

                    double cost_via_new_node = cost_to_new_node + dist_to_new_node;

                    if (cost_via_new_node < tree[i].cost) { // 通过新节点到达该节点更优，尝试重连

                        // TODO：检查从新节点到该节点的路径是否碰撞，如果不碰撞则重连
                        bool is_collision_free_path =
                            isCollisionFreePath(tree[i].pos_idx, new_idx);

                        if (is_collision_free_path) {
                            
                            double old_cost = tree[i].cost;
                            tree[i].parent = tree.size() - 1; // 更新父节点索引
                            tree[i].cost = cost_via_new_node; // 更新代价
                            
                            // 递归更新该节点所有子节点的代价
                            double cost_delta = cost_via_new_node - old_cost;
                            updateChildrenCost(i, cost_delta);
                        }
                    }
                }
            }

            // 检查是否到达目标点附近
            unsigned int goal_mx, goal_my;
            goal_my = goal_idx / width;
            goal_mx = goal_idx % width;

            double ddx_goal = goal_mx - new_mx, ddy_goal = goal_my - new_my;
            double dist_to_goal_sqrd =
            ddx_goal * ddx_goal + ddy_goal * ddy_goal;

            if (dist_to_goal_sqrd < goal_tolerance_cells * goal_tolerance_cells) {

                // 检查到目标的路径是否碰撞
                if (isCollisionFreePath(new_idx, goal_idx)) {

                    found_path = true;
                    if (stop_iter == 0) RCLCPP_INFO(node_->get_logger(), "RRT* found a path to the goal in %d iterations, now optimizing...", iter + 1);
                    RCLCPP_INFO(node_->get_logger(), "RRT* successfully found a path to the goal in %d iterations", iter + 1);
                    double cost_to_goal = cost_to_new_node + std::sqrt(dist_to_goal_sqrd);
                    if (cost_to_goal < best_cost) {

                        best_cost = cost_to_goal;
                        goal_node_idx = tree.size() - 1; // 记录新节点在树中的索引
                    }
                }
            }
            if (found_path) {

                stop_iter ++;

                if (stop_iter >= max_iterations_after_goal_) {

                    RCLCPP_INFO(node_->get_logger(), "RRT* optimization finished after %d iterations", iter + 1);
                    break;
                }
            }
        }
        // 记录算法耗时
        auto end_time = std::chrono::steady_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        RCLCPP_INFO(node_->get_logger(), "RRT* planning completed in %ld ms", duration_ms);

        if (found_path && goal_node_idx != -1) {

            std::vector<int> path_;
            int curr_idx = goal_node_idx;
            while (curr_idx != -1) {

                path_.push_back(tree[curr_idx].pos_idx);
                curr_idx = tree[curr_idx].parent;
            }
            std::reverse(path_.begin(), path_.end());
            
            // 添加目标点
            path_.push_back(goal_idx);

            // 线性插值生成全局路径
            for (size_t i = 0; i < path_.size() - 1; ++ i) {

                unsigned int mx1 = path_[i] % width, my1 = path_[i] / width;
                unsigned int mx2 = path_[i + 1] % width, my2 = path_[i + 1] / width;

                double wx1, wy1, wx2, wy2;
                costmap_->mapToWorld(mx1, my1, wx1, wy1);
                costmap_->mapToWorld(mx2, my2, wx2, wy2);

                double dist = std::hypot(wx2 - wx1, wy2 - wy1);
                // 根据距离和碰撞检查分辨率计算插值点的数量，确保路径平滑且足够密集进行碰撞检查
                int steps = std::max(1, static_cast<int>(dist / collision_check_resolution_));

                for (int j = 0; j < steps; ++ j) {

                    double t = static_cast<double>(j) / steps;
                    double 
                        wx = (1 - t) * wx1 + t * wx2,
                        wy = (1 - t) * wy1 + t * wy2;

                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = global_frame_;
                    pose.header.stamp = node_->now();
                    pose.pose.position.x = wx;
                    pose.pose.position.y = wy;
                    pose.pose.orientation.w = goal.pose.orientation.w;
                    global_path.poses.push_back(pose);
                }
            }
        } else {

            global_path.poses.clear();
            RCLCPP_WARN(node_->get_logger(), "RRT* failed to find a path from start to goal");
        }
        return global_path;
    }

    /**
     * @brief 递归更新子节点的代价
     * @param parent_idx 父节点在树中的索引
     * @param cost_delta 父节点代价的变化量（新代价 - 旧代价）
     * 这个函数会遍历树中所有以 parent_idx 为父节点的节点，更新它们的代价，并递归更新它们的子节点的代价
     * @return void
     */
    void MyRRTStarPlanner::updateChildrenCost(int parent_idx, double cost_delta) {
        
        // 递归遍历所有子节点，更新它们的代价
        for (size_t i = 0; i < tree.size(); ++ i) {

            if (tree[i].parent == parent_idx) {

                tree[i].cost += cost_delta;
                updateChildrenCost(i, cost_delta); // 递归更新其子节点
            }
        }
    }
}// namespace my_rrtstar_planner

PLUGINLIB_EXPORT_CLASS(my_rrtstar_planner::MyRRTStarPlanner, nav2_core::GlobalPlanner)